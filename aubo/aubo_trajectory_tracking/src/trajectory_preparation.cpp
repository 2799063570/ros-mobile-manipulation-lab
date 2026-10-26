#include "tracking_curve.h"
#include <aubo_ros_control/MoveHomeAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <mutex>
#include <sstream>
#include <iomanip>

using Steady = std::chrono::steady_clock;
using JointVector = std::vector<double>;

// Planning never publishes servo commands. An explicit move request uses the existing
// cancellable joint-space executor, retaining its online collision/braking checks.
class Preparation
{
  ros::NodeHandle nh_, pn_{"~"}, tracker_{"/trajectory_tracking"};
  std::string jog_node_, jog_ns_, group_, base_, tool_, jog_state_, status_{"IDLE: 设置参数后搜索推荐起点"};
  std::vector<std::string> names_;
  JointVector lower_, upper_, velocities_, accelerations_, home_, current_, selected_, analyzed_from_;
  double margin_, stop_sigma_, slow_sigma_, timeout_, move_timeout_;
  ros::Time joint_stamp_;
  Steady::time_point joint_time_{}, state_time_{}, move_time_{};
  bool feedback_valid_{false}, moving_{false};
  Curve selected_curve_{};
  std::mutex mutex_, computation_;
  std::atomic<unsigned> epoch_{0};
  ros::Publisher status_pub_, path_pub_;
  ros::Subscriber joints_sub_, state_sub_;
  ros::ServiceServer analyze_srv_, check_srv_, move_srv_, stop_srv_;
  ros::ServiceClient tracker_stop_, disable_;
  ros::WallTimer timer_;
  actionlib::SimpleActionClient<aubo_ros_control::MoveHomeAction> home_client_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> vel_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_;

  struct Assessment {
    bool geometric{false}, feasible{false};
    double sigma{1e9}, room{1e9}, ratio{1.0}, score{-1e9};
    std::string reason;
    nav_msgs::Path path;
  };
  static KDL::JntArray joints(const JointVector& values)
  {
    KDL::JntArray q(values.size());
    for(size_t i=0;i<values.size();++i) q(i)=values[i];
    return q;
  }
  static double difference(const JointVector& a,const JointVector& b)
  {
    double d=0; for(size_t i=0;i<a.size();++i) d=std::max(d,std::abs(a[i]-b[i])); return d;
  }
  void status(const std::string& value) { std::lock_guard<std::mutex> lock(mutex_); status_=value; }
  std::string snapshot(JointVector& q)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(moving_) return "正在移动起点，请等待或停止";
    if(!feedback_valid_ || current_.size()!=names_.size() ||
       std::chrono::duration<double>(Steady::now()-joint_time_).count()>timeout_ ||
       (ros::Time::now()-joint_stamp_).toSec()>timeout_ || (ros::Time::now()-joint_stamp_).toSec() < -0.05)
      return "关节反馈无效或过期，请保持仿真运行并等待反馈";
    if(std::chrono::duration<double>(Steady::now()-state_time_).count()>.35 || jog_state_!="DISABLED")
      return "底层必须处于新鲜的 DISABLED 状态，当前为 "+jog_state_;
    q=current_; return "";
  }
  bool sameCurve(const Curve& a,const Curve& b) const
  {
    return a.shape==b.shape && a.plane==b.plane && a.radius==b.radius && a.amplitude==b.amplitude &&
      a.length==b.length && a.cycles==b.cycles && a.waves==b.waves && a.duration==b.duration &&
      a.ki==b.ki && a.correction_fraction==b.correction_fraction && a.speed==b.speed && a.extent==b.extent && a.kp==b.kp && a.error==b.error &&
      a.tolerance==b.tolerance && a.settle_speed==b.settle_speed && a.settle_timeout==b.settle_timeout;
  }
  bool validState(const KDL::JntArray& q, moveit::core::RobotState& state,
                  const planning_scene::PlanningSceneConstPtr& scene, double& room)
  {
    for(size_t j=0;j<names_.size();++j) {
      if(!std::isfinite(q(j)) || q(j)<lower_[j]+margin_ || q(j)>upper_[j]-margin_) return false;
      room=std::min(room,std::min(q(j)-lower_[j]-margin_,upper_[j]-margin_-q(j)));
      state.setVariablePosition(names_[j],q(j));
    }
    state.update();
    return !scene->isStateColliding(state);
  }
  bool segment(const KDL::JntArray& a,const KDL::JntArray& b,moveit::core::RobotState& state,
               const planning_scene::PlanningSceneConstPtr& scene, double& room, unsigned token)
  {
    double delta=(a.data-b.data).cwiseAbs().maxCoeff();
    const int count=std::max(1,static_cast<int>(std::ceil(delta/.01)));
    for(int i=0;i<=count;++i) {
      if(epoch_.load()!=token || !ros::ok()) return false;
      KDL::JntArray q(a.rows()); q.data=a.data+(b.data-a.data)*(double(i)/count);
      if(!validState(q,state,scene,room)) return false;
    }
    return true;
  }
  Assessment assess(const JointVector& start,const JointVector& from,const Curve& c,
                    const planning_scene::PlanningSceneConstPtr& scene,unsigned token,bool approach)
  {
    Assessment result;
    moveit::core::RobotState state(scene->getCurrentState());
    KDL::JntArray q=joints(start), seed=q;
    if(!validState(q,state,scene,result.room)) {result.reason="起始姿态碰撞或超过关节余量"; return result;}
    if(approach && !segment(joints(from),q,state,scene,result.room,token)) {
      result.reason="到起点的关节直线路径被碰撞/限位阻挡，需先调整当前姿态"; return result;
    }
    KDL::Frame origin;
    fk_->JntToCart(q,origin);
    const double length=c.shape=="circle" ? 2*PI*c.radius*c.cycles : std::hypot(c.length,2*PI*c.waves*c.amplitude);
    const int count=std::min(5000,std::max(300,static_cast<int>(std::ceil(length/.001))));
    const double dt=c.duration/count;
    Eigen::VectorXd prev_velocity=Eigen::VectorXd::Zero(names_.size());
    double velocity_ratio=0,acceleration_ratio=0;
    result.path.header.frame_id=base_; result.path.header.stamp=ros::Time::now();
    for(int i=0;i<=count;++i) {
      if(epoch_.load()!=token || !ros::ok()) {result.reason="已取消检查"; return result;}
      KDL::Vector p,v; c.sample(i*dt,p,v);
      KDL::Frame desired(origin.M,origin.p+p), achieved;
      if(i>0 && ik_->CartToJnt(seed,desired,q)<0) {result.reason="路径 "+std::to_string(i*100/count)+"% 处无连续逆解"; return result;}
      fk_->JntToCart(q,achieved);
      if((achieved.p-desired.p).Norm()>.0002 || KDL::diff(achieved.M,desired.M).Norm()>.002) {
        result.reason="逆解位置或姿态残差过大"; return result;
      }
      if(i>0 && (q.data-seed.data).cwiseAbs().maxCoeff()>.15) {result.reason="逆解发生关节跳变"; return result;}
      if(!segment(seed,q,state,scene,result.room,token)) {result.reason="路径碰撞或关节限位"; return result;}
      KDL::Jacobian jac(names_.size()); jac_->JntToJac(q,jac);
      const double sigma=Eigen::JacobiSVD<Eigen::MatrixXd>(jac.data).singularValues().minCoeff();
      result.sigma=std::min(result.sigma,sigma);
      if(sigma<=stop_sigma_+.002) {result.reason="路径过于接近底层奇异停止阈值"; return result;}
      const double scale=std::min(1.0,(sigma-stop_sigma_)/(slow_sigma_-stop_sigma_));
      if(i>0) {
        const Eigen::VectorXd velocity=(q.data-seed.data)/dt;
        for(size_t j=0;j<names_.size();++j) {
          velocity_ratio=std::max(velocity_ratio,std::abs(velocity(j))/(velocities_[j]*scale));
          acceleration_ratio=std::max(acceleration_ratio,std::abs(velocity(j)-prev_velocity(j))/(dt*accelerations_[j]));
        }
        prev_velocity=velocity;
      }
      geometry_msgs::PoseStamped pose;
      pose.header=result.path.header;
      pose.pose.position.x=desired.p.x(); pose.pose.position.y=desired.p.y(); pose.pose.position.z=desired.p.z();
      desired.M.GetQuaternion(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
      result.path.poses.push_back(pose);
      seed=q;
    }
    for(size_t j=0;j<names_.size();++j)
      acceleration_ratio=std::max(acceleration_ratio,std::abs(prev_velocity(j))/(dt*accelerations_[j]));
    result.geometric=true;
    result.ratio=std::max({1.0,velocity_ratio/.8,std::sqrt(acceleration_ratio/.8)});
    result.feasible=result.ratio<=1.01;
    result.score=result.sigma+.03*std::min(1.0,result.room)-.002*difference(start,from);
    std::ostringstream out; out<<std::fixed<<std::setprecision(1);
    if(!result.feasible) out<<"关节速度/加速度预算不足，建议时长至少 "<<c.duration*result.ratio<<" s";
    else out<<"离散路径检查通过；最小奇异值 "<<std::setprecision(4)<<result.sigma<<"，关节余量 "<<result.room<<" rad";
    result.reason=out.str(); return result;
  }
  planning_scene::PlanningScenePtr sceneSnapshot()
  {
    planning_scene_monitor::LockedPlanningSceneRO lock(scene_);
    return planning_scene::PlanningScene::clone(lock);
  }
  bool check(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    std::unique_lock<std::mutex> compute(computation_,std::try_to_lock);
    if(!compute.owns_lock()) {res.message="正在搜索或检查起点，请等待"; return true;}
    JointVector q; res.message=snapshot(q); if(!res.message.empty()) return true;
    try {
      const auto c=loadCurve(tracker_);
      const auto result=assess(q,q,c,sceneSnapshot(),epoch_.load(),false);
      res.success=result.feasible; res.message=result.reason;
    } catch(const std::exception& e) {res.message=e.what();}
    return true;
  }
  bool analyze(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    std::unique_lock<std::mutex> compute(computation_,std::try_to_lock);
    if(!compute.owns_lock()) {res.message="已有起点检查在进行"; return true;}
    JointVector from; res.message=snapshot(from); if(!res.message.empty()) return true;
    const unsigned token=epoch_.load();
    {std::lock_guard<std::mutex> lock(mutex_); selected_.clear();}
    status("SEARCHING: 正在搜索候选姿态并检查整条曲线，不移动机械臂");
    try {
      const Curve c=loadCurve(tracker_);
      const auto scene=sceneSnapshot();
      std::vector<JointVector> candidates{from,home_};
      for(const auto& seed : std::vector<JointVector>{home_,from}) {
        for(int joint=0;joint<5;++joint) for(double delta : {-.4,.4}) {
          auto candidate=seed; candidate[joint]+=delta; candidates.push_back(candidate);
        }
      }
      Assessment best; JointVector best_q; std::string last_reason;
      double min_time=1e9; int checked=0;
      for(const auto& q : candidates) {
        if(epoch_.load()!=token) break;
        const auto result=assess(q,from,c,scene,token,true); ++checked;
        if(result.geometric) min_time=std::min(min_time,c.duration*result.ratio);
        if(result.feasible && result.score>best.score) {best=result; best_q=q;}
        last_reason=result.reason;
      }
      if(epoch_.load()!=token) {res.message="起点搜索已取消"; return true;}
      if(best_q.empty()) {
        res.message="候选姿态均未通过，"+last_reason;
        if(min_time<1e9) res.message="存在几何可行候选，但时长不足，建议至少 "+std::to_string(min_time)+" s 后重新搜索";
        status("FAILED: "+res.message); return true;
      }
      JointVector now; const auto issue=snapshot(now);
      if(!issue.empty() || difference(now,from)>.01) {res.message="搜索期间机械臂或底层状态改变，请停稳后重新搜索"; status("FAILED: "+res.message); return true;}
      {
        std::lock_guard<std::mutex> lock(mutex_);
        selected_=best_q; selected_curve_=c; analyzed_from_=from;
      }
      path_pub_.publish(best.path);
      KDL::Frame pose; fk_->JntToCart(joints(best_q),pose);
      std::ostringstream out; out<<"已检查 "<<checked<<" 个候选；"<<best.reason<<"；起点 XYZ="<<std::fixed<<std::setprecision(3)
        <<pose.p.x()<<", "<<pose.p.y()<<", "<<pose.p.z()<<" m。点击移动到推荐起点后再预览";
      res.success=true; res.message=out.str(); status("READY: "+res.message);
    } catch(const std::exception& e) {res.message=e.what(); status("FAILED: "+res.message);}
    return true;
  }
  bool move(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    std::unique_lock<std::mutex> compute(computation_,std::try_to_lock);
    if(!compute.owns_lock()) {res.message="请等待检查完成"; return true;}
    JointVector from; res.message=snapshot(from); if(!res.message.empty()) return true;
    const unsigned token=epoch_.load();
    try {
      const auto c=loadCurve(tracker_);
      JointVector target;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if(selected_.empty() || !sameCurve(c,selected_curve_) || difference(from,analyzed_from_)>.01) {
          res.message="参数或当前姿态已改变，请重新搜索起点"; return true;
        }
        target=selected_;
      }
      if(!home_client_.isServerConnected()) {res.message="底层关节移动 action 未就绪"; return true;}
      // Recheck the current scene immediately before the explicitly requested move.
      const auto result=assess(target,from,c,sceneSnapshot(),token,true);
      if(!result.feasible) {res.message=result.reason; return true;}
      JointVector now;
      const auto issue=snapshot(now);
      if(!issue.empty() || difference(now,from)>.01) {res.message="检查期间姿态或底层状态改变，请重新搜索"; return true;}
      std_srvs::Trigger stop_tracker;
      if(!tracker_stop_.call(stop_tracker) || !stop_tracker.response.success) {res.message="无法停止并清除旧跟踪预览"; return true;}
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if(epoch_.load()!=token) {res.message="移动请求已被停止取消"; return true;}
        aubo_ros_control::MoveHomeGoal goal; goal.target_joint_positions=target;
        moving_=true; move_time_=Steady::now(); selected_.clear();
        status_="MOVING: 正在移动到推荐起点，尚未开始画曲线；可随时停止";
        home_client_.sendGoal(goal);
      }
      res.success=true; res.message="已请求移动到推荐起点，请等待到位后更新预览";
    } catch(const std::exception& e) {res.message=e.what();}
    return true;
  }
  bool stop(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++epoch_; selected_.clear();
      if(moving_) home_client_.cancelGoal();
      status_=moving_ ? "STOPPING: 已取消起点移动，等待底层停止" : "IDLE: 已取消起点搜索，请重新搜索或预览";
    }
    std_srvs::SetBool disable; disable.request.data=false;
    res.success=disable_.call(disable) && disable.response.success;
    res.message=res.success ? "已请求停止起点移动" : "底层未确认停止，请检查控制器";
    return true;
  }
  void tick(const ros::WallTimerEvent&)
  {
    std_msgs::String message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(moving_ && home_client_.getState().isDone()) {
        const auto result=home_client_.getResult();
        const bool success=home_client_.getState()==actionlib::SimpleClientGoalState::SUCCEEDED && result && result->success;
        moving_=false;
        status_=success ? "ARRIVED: 推荐起点已到达；请更新预览重新检查实际起点，再开始跟踪" : "FAILED: 起点移动未完成："+(result ? result->message : "底层未返回结果");
      }
      // Cancel an orphaned move if fresh raw feedback/status is lost, even if ROS /clock is paused.
      if(moving_ && (std::chrono::duration<double>(Steady::now()-joint_time_).count()>timeout_ ||
          std::chrono::duration<double>(Steady::now()-state_time_).count()>.5 ||
          std::chrono::duration<double>(Steady::now()-move_time_).count()>move_timeout_+2)) {
        home_client_.cancelGoal(); status_="STOPPING: 移动期间反馈/底层状态超时，已取消";
      }
      message.data=status_;
    }
    status_pub_.publish(message);
  }
public:
  Preparation(): home_client_("/cartesian_jog/move_home",true)
  {
    pn_.param<std::string>("jog_node_ns",jog_node_,"/aubo_cartesian_jog");
    tracker_.param<std::string>("jog_ns",jog_ns_,"/cartesian_jog");
    if(jog_ns_!="/cartesian_jog") throw std::runtime_error("Preparation currently requires jog_ns=/cartesian_jog");
    ros::NodeHandle jog(jog_node_);
    jog.param<std::string>("group_name",group_,"aubo_i5");
    tracker_.param<std::string>("base_link",base_,"base_link"); tracker_.param<std::string>("tool_link",tool_,"tcp_link");
    jog.param("joint_limit_margin",margin_,.08); jog.param("singularity_stop",stop_sigma_,.008); jog.param("singularity_slow",slow_sigma_,.06);
    jog.param("home_timeout",move_timeout_,90.);
    tracker_.param("feedback_timeout",timeout_,.15);
    if(!jog.getParam("joint_names",names_) || names_.size()!=6) throw std::runtime_error("Missing six jog joint names");
    for(auto entry : std::vector<std::pair<std::string,JointVector*>>{{"joint_lower_limits",&lower_},{"joint_upper_limits",&upper_},
        {"joint_velocity_limits",&velocities_},{"joint_acceleration_limits",&accelerations_},{"home_joint_positions",&home_}}) {
      if(!jog.getParam(entry.first,*entry.second) || entry.second->size()!=6) throw std::runtime_error("Invalid jog parameter "+entry.first);
      for(double value:*entry.second) if(!std::isfinite(value)) throw std::runtime_error("Non-finite jog parameter "+entry.first);
    }
    if(!std::isfinite(margin_) || margin_<=0 || stop_sigma_<=0 || slow_sigma_<=stop_sigma_ || timeout_<=0)
      throw std::runtime_error("Invalid preparation limits");
    for(size_t i=0;i<6;++i) if(lower_[i]+2*margin_>=upper_[i] || velocities_[i]<=0 || accelerations_[i]<=0)
      throw std::runtime_error("Invalid joint bounds/dynamic limits");
    std::string xml; nh_.getParam("robot_description",xml); KDL::Tree tree;
    if(!kdl_parser::treeFromString(xml,tree) || !tree.getChain(base_,tool_,chain_) || chain_.getNrOfJoints()!=6)
      throw std::runtime_error("Cannot construct preparation KDL chain");
    size_t index=0;
    for(const auto& segment:chain_.segments) if(segment.getJoint().getType()!=KDL::Joint::None && segment.getJoint().getName()!=names_.at(index++))
      throw std::runtime_error("Preparation joint order differs from jog chain");
    fk_.reset(new KDL::ChainFkSolverPos_recursive(chain_)); jac_.reset(new KDL::ChainJntToJacSolver(chain_));
    vel_.reset(new KDL::ChainIkSolverVel_pinv(chain_));
    auto lo=joints(lower_),hi=joints(upper_); lo.data.array()+=margin_; hi.data.array()-=margin_;
    ik_.reset(new KDL::ChainIkSolverPos_NR_JL(chain_,lo,hi,*fk_,*vel_,150,1e-6));
    scene_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    if(!scene_->getPlanningScene() || !scene_->getRobotModel()->getJointModelGroup(group_)) throw std::runtime_error("Missing planning scene or group");
    std::string scene_topic,initial_service,topic;
    jog.param<std::string>("planning_scene_topic",scene_topic,"/planning_scene"); jog.param<std::string>("initial_scene_service",initial_service,"");
    tracker_.param<std::string>("joint_states_topic",topic,"/cartesian_jog/sdk_joint_states");
    scene_->startSceneMonitor(scene_topic); scene_->startWorldGeometryMonitor("/collision_object","/planning_scene_world",false);
    scene_->startStateMonitor(topic);
    if(!initial_service.empty() && !scene_->requestPlanningSceneState(initial_service)) throw std::runtime_error("Cannot fetch initial planning scene");
    joints_sub_=nh_.subscribe<sensor_msgs::JointState>(topic,1,[this](const sensor_msgs::JointState::ConstPtr& msg){
      std::lock_guard<std::mutex> lock(mutex_);
      feedback_valid_=false;
      const double delay=(ros::Time::now()-msg->header.stamp).toSec();
      if(msg->name.size()!=msg->position.size() || msg->header.stamp.isZero() || delay>timeout_ || delay < -.05 ||
         (!joint_stamp_.isZero() && msg->header.stamp<=joint_stamp_)) return;
      JointVector q;
      for(const auto& name:names_) {
        const auto it=std::find(msg->name.begin(),msg->name.end(),name);
        if(it==msg->name.end() || !std::isfinite(msg->position[it-msg->name.begin()])) return;
        q.push_back(msg->position[it-msg->name.begin()]);
      }
      current_=q; joint_stamp_=msg->header.stamp; joint_time_=Steady::now(); feedback_valid_=true;
    });
    state_sub_=nh_.subscribe<std_msgs::String>(jog_ns_+"/state",1,[this](const std_msgs::String::ConstPtr& msg){
      std::lock_guard<std::mutex> lock(mutex_); jog_state_=msg->data; state_time_=Steady::now();
    });
    tracker_stop_=nh_.serviceClient<std_srvs::Trigger>("/trajectory_tracking/stop");
    disable_=nh_.serviceClient<std_srvs::SetBool>(jog_ns_+"/set_enabled");
    status_pub_=pn_.advertise<std_msgs::String>("state",1,true); path_pub_=pn_.advertise<nav_msgs::Path>("candidate_path",1,true);
    analyze_srv_=pn_.advertiseService("analyze",&Preparation::analyze,this); check_srv_=pn_.advertiseService("check_current",&Preparation::check,this);
    move_srv_=pn_.advertiseService("move",&Preparation::move,this); stop_srv_=pn_.advertiseService("stop",&Preparation::stop,this);
    timer_=nh_.createWallTimer(ros::WallDuration(.1),&Preparation::tick,this);
  }
  ~Preparation() {++epoch_; if(moving_) home_client_.cancelGoal();}
};
int main(int argc,char** argv)
{
  ros::init(argc,argv,"trajectory_preparation");
  try {Preparation node; ros::AsyncSpinner spinner(4); spinner.start(); ros::waitForShutdown();}
  catch(const std::exception& e) {ROS_FATAL("Preparation: %s",e.what()); return 1;}
  return 0;
}
