#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/MarkerArray.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <iomanip>
#include "tracking_curve.h"
#include "tracking_control.h"

using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
static double age(Time t) { return std::chrono::duration<double>(Clock::now()-t).count(); }

class Tracker
{
  ros::NodeHandle nh_, pn_{"~"};
  ros::Publisher cmd_, state_pub_, readiness_pub_, metrics_, samples_pub_, run_start_pub_, reference_pub_, actual_pub_, markers_;
  ros::Subscriber joints_, jog_state_sub_;
  ros::ServiceClient enable_, preflight_;
  bool check_feasibility_{true};
  ros::ServiceServer preview_srv_, start_srv_, stop_srv_;
  ros::WallTimer timer_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::vector<std::string> joint_names_;
  KDL::Frame actual_;
  KDL::Vector origin_, target_, previous_actual_;
  Curve curve_;
  nav_msgs::Path reference_, trace_;
  std::string feedback_issue_{"尚未收到有效关节反馈"};
  std::string base_, tool_, jog_ns_, jog_state_, state_{"IDLE"}, detail_{"Preview a curve to begin"};
  ros::Time run_stamp_, tick_stamp_, sample_stamp_;
  TcpVelocityWindow velocity_window_;
  TrackingCorrection correction_;
  KDL::Vector trend_velocity_{KDL::Vector::Zero()};
  double velocity_window_duration_{0.08}, reported_error_{0};
  Time feedback_time_{}, state_time_{}, tick_time_{Clock::now()}, warmup_time_{}, settle_time_{}, stable_time_{};
  ros::Time feedback_stamp_;
  double feedback_timeout_, state_timeout_, elapsed_{0}, peak_{0}, sum2_{0}, actual_speed_{0};
  size_t samples_{0};
  bool feedback_valid_{false}, have_feedback_{false}, previewed_{false}, active_{false}, owned_{false};

  Curve load() { return loadCurve(pn_); }
  std::string healthIssue() const
  {
    if(!feedback_valid_) return "关节反馈无效："+feedback_issue_;
    if(age(feedback_time_)>feedback_timeout_) return "关节反馈接收超时，请检查反馈话题和 Gazebo 是否暂停";
    const double stamp_age=(ros::Time::now()-feedback_stamp_).toSec();
    if(stamp_age < -0.05) return "关节反馈时间戳超前，请检查 /clock 和 use_sim_time";
    if(stamp_age>feedback_timeout_) return "关节反馈时间戳过期，请检查 /clock 和反馈发布频率";
    if(jog_state_.empty()) return "尚未收到点动控制器状态";
    if(age(state_time_)>state_timeout_) return "点动控制器状态超时，请检查底层节点";
    return "";
  }
  bool healthy() const { return healthIssue().empty(); }
  std::string previewIssue() const
  {
    if(active_ || owned_) return "请先停止当前跟踪，等待底层禁用";
    const auto issue=healthIssue();
    if(!issue.empty()) return issue;
    if(jog_state_!="DISABLED") return "底层状态为 "+jog_state_+"，需等待 DISABLED；故障需在底层排查";
    return "";
  }
  std::string startIssue() const
  {
    const auto issue=previewIssue();
    if(!issue.empty()) return issue;
    if(!previewed_) return "请先点击“更新预览”，成功后再开始跟踪";
    const double distance=(actual_.p-origin_).Norm();
    if(distance>curve_.tolerance) {
      std::ostringstream out;
      out<<std::fixed<<std::setprecision(2)<<"TCP 已偏离预览起点 "<<distance*1000
         <<" mm（允许 "<<curve_.tolerance*1000<<" mm），请重新更新预览";
      return out.str();
    }
    if(cmd_.getNumSubscribers()!=1) return "点动指令订阅者数量为 "+std::to_string(cmd_.getNumSubscribers())+"，应为 1，请检查底层节点";
    return "";
  }
  geometry_msgs::PoseStamped pose(const KDL::Vector& p) const
  {
    geometry_msgs::PoseStamped out; out.header.frame_id=base_; out.header.stamp=ros::Time::now();
    out.pose.position.x=p.x(); out.pose.position.y=p.y(); out.pose.position.z=p.z();
    actual_.M.GetQuaternion(out.pose.orientation.x,out.pose.orientation.y,out.pose.orientation.z,out.pose.orientation.w);
    return out;
  }
  void command(const KDL::Vector& v=KDL::Vector::Zero())
  {
    geometry_msgs::TwistStamped msg; msg.header.frame_id=base_; msg.header.stamp=ros::Time::now();
    msg.twist.linear.x=v.x(); msg.twist.linear.y=v.y(); msg.twist.linear.z=v.z(); cmd_.publish(msg);
  }
  bool disable()
  {
    if(!owned_) return true;
    std_srvs::SetBool srv; srv.request.data=false;
    if(!enable_.call(srv) || !srv.response.success) return false;
    owned_=false; return true;
  }
  void finish(const std::string& state, const std::string& why)
  {
    active_=false; previewed_=false; correction_.reset();
    if(owned_) command();
    actual_pub_.publish(trace_);
    state_=state; detail_=why;
    if(!disable()) { state_="FAULT"; detail_="Disable not acknowledged; retrying: "+why; }
    ROS_INFO_STREAM("[tracking] "<<state_<<": "<<detail_);
  }
  void feedback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    feedback_valid_=false;
    const double delay=(ros::Time::now()-msg->header.stamp).toSec();
    if(msg->header.stamp.isZero()) {feedback_issue_="时间戳为零，等待仿真时钟和反馈就绪"; return;}
    if(delay>feedback_timeout_) {feedback_issue_="时间戳过期，请检查仿真时钟和发布频率"; return;}
    if(delay < -0.05) {feedback_issue_="时间戳超前，请检查仿真时钟"; return;}
    if(msg->name.size()!=msg->position.size()) {feedback_issue_="关节名称与位置数量不一致"; return;}
    if(have_feedback_ && msg->header.stamp<=feedback_stamp_) {feedback_issue_="时间戳重复或倒退，请检查 Gazebo 暂停或仿真重置"; return;}
    KDL::JntArray q(joint_names_.size());
    for(size_t i=0;i<joint_names_.size();++i) {
      auto it=std::find(msg->name.begin(),msg->name.end(),joint_names_[i]);
      if(it==msg->name.end()) {feedback_issue_="缺少关节 "+joint_names_[i]; return;}
      q(i)=msg->position[it-msg->name.begin()]; if(!std::isfinite(q(i))) {feedback_issue_="关节位置包含非有限数值"; return;}
    }
    KDL::Frame frame;
    if(fk_->JntToCart(q,frame)<0) {feedback_issue_="TCP 正运动学计算失败"; return;}
    actual_speed_=have_feedback_ ? (frame.p-previous_actual_).Norm()/(msg->header.stamp-feedback_stamp_).toSec() : 0;
    trend_velocity_=velocity_window_.add(msg->header.stamp.toSec(),frame.p,velocity_window_duration_);
    if(!have_feedback_) target_=frame.p;
    actual_=frame; previous_actual_=frame.p; feedback_stamp_=msg->header.stamp;
    feedback_time_=Clock::now(); have_feedback_=feedback_valid_=true; feedback_issue_.clear();
  }
  bool preview(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    if(active_ || owned_) {res.message="请先停止当前跟踪"; return true;}
    previewed_=false;
    const auto issue=previewIssue();
    if(!issue.empty()) {res.message=issue; return true;}
    try {curve_=load();} catch(const std::exception& e) {res.message=e.what(); return true;}
    if(check_feasibility_) {
      std_srvs::Trigger check;
      if(!preflight_.exists() || !preflight_.call(check)) {res.message="起点检查服务未就绪，请检查 trajectory_preparation 节点"; return true;}
      if(!check.response.success) {res.message="整圈检查未通过："+check.response.message; return true;}
    }
    origin_=actual_.p; target_=origin_;
    reference_.header.frame_id=trace_.header.frame_id=base_; reference_.header.stamp=ros::Time::now();
    reference_.poses.clear(); trace_.poses.clear();
    for(int i=0;i<=1000;++i) {KDL::Vector p,v; curve_.sample(curve_.duration*i/1000.0,p,v); reference_.poses.push_back(pose(origin_+p));}
    reference_pub_.publish(reference_); actual_pub_.publish(trace_);
    elapsed_=peak_=sum2_=reported_error_=0; samples_=0;
    correction_.reset(); sample_stamp_=ros::Time();
    samples_pub_.publish(std_msgs::Float64MultiArray());
    run_start_pub_.publish(std_msgs::Header());
    previewed_=true; state_="PREVIEW"; detail_="Preview only; Start explicitly to move";
    res.success=true; res.message="预览成功：已记录当前 TCP 起点和参数，请检查曲线后点击开始跟踪"; return true;
  }
  bool start(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    const auto issue=startIssue();
    if(!issue.empty()) {res.message=issue; return true;}
    std_srvs::SetBool srv; srv.request.data=true;
    // A lost service response does not prove the request was never applied.
    owned_=true;
    if(!enable_.call(srv) || !srv.response.success) {
      finish("FAULT","Jog enable failed: "+srv.response.message); res.message=detail_; return true;
    }
    owned_=active_=true; previewed_=false; state_="ARMING"; detail_="Sending neutral commands; waiting for fresh enable confirmation";
    correction_.reset(); warmup_time_=Clock::now(); tick_time_=Clock::now(); tick_stamp_=ros::Time::now(); command(); res.success=true; res.message=detail_; return true;
  }
  bool stop(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res)
  {
    finish("STOPPED","Stop requested; queued SDK points and deceleration remain");
    res.success=!owned_; res.message=detail_; return true;
  }
  void visualize()
  {
    std_msgs::String status; status.data=state_+": "+detail_; state_pub_.publish(status);
    std_msgs::String readiness;
    const auto issue=previewIssue();
    if(!issue.empty()) readiness.data="WAITING: "+issue;
    else {
      const auto start_issue=startIssue();
      readiness.data=start_issue.empty() ? "START_READY: 预览就绪，可开始跟踪" : "PREVIEW_READY: "+start_issue;
    }
    readiness_pub_.publish(readiness);
    if(!have_feedback_) return;
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker m; m.header.frame_id=base_; m.header.stamp=ros::Time::now(); m.ns="tracking";
    m.action=m.ADD; m.pose.orientation.w=1; m.id=0; m.type=m.SPHERE; m.pose.position=pose(target_).pose.position;
    m.scale.x=m.scale.y=m.scale.z=0.009; m.color.r=1; m.color.g=0.7; m.color.a=1; arr.markers.push_back(m);
    m.id=1; m.type=m.LINE_LIST; m.pose.position=geometry_msgs::Point(); m.scale.x=0.002;
    m.color.r=1; m.color.g=0.25; m.points={pose(target_).pose.position,pose(actual_.p).pose.position}; arr.markers.push_back(m);
    m.id=2; m.type=m.TEXT_VIEW_FACING; m.points.clear(); m.pose.position=pose(actual_.p+KDL::Vector(0,0,0.10)).pose.position;
    m.scale.z=0.025; m.color.r=m.color.g=m.color.b=1;
    m.text=state_+"  "+std::to_string(static_cast<int>(1000*(target_-actual_.p).Norm()))+" mm"; arr.markers.push_back(m); markers_.publish(arr);
    std_msgs::Float64MultiArray metrics;
    metrics.data={elapsed_,reported_error_,peak_,samples_?std::sqrt(sum2_/samples_):0,actual_speed_}; metrics_.publish(metrics);
  }
  void tick(const ros::WallTimerEvent&)
  {
    const double wall_dt=age(tick_time_); tick_time_=Clock::now();
    const ros::Time now=ros::Time::now();
    const double dt=tick_stamp_.isZero() ? 0.0 : (now-tick_stamp_).toSec(); tick_stamp_=now;
    if(!active_) { if(owned_) {command(); disable();} visualize(); return; }
    if(wall_dt>0.10 || dt<0 || dt>0.10 || !healthy()) {finish("FAULT","Tick stalled or feedback/controller state stale"); visualize(); return;}
    if(state_=="ARMING") {
      command();
      if(jog_state_.find("FAULT")==0 || age(warmup_time_)>1.0) finish("FAULT","Enable confirmation failed");
      else if(age(warmup_time_)>=0.3 && (jog_state_=="ENABLED_IDLE" || jog_state_=="JOGGING") && state_time_>warmup_time_) {
        run_stamp_=now; std_msgs::Header run; run.stamp=now; run_start_pub_.publish(run); state_="RUNNING"; detail_="Tracking with feedforward + TCP position correction";
      }
      visualize(); return;
    }
    if(jog_state_!="ENABLED_IDLE" && jog_state_!="JOGGING") {finish("FAULT","Jog controller left enabled state: "+jog_state_); visualize(); return;}
    elapsed_=std::min(curve_.duration,(now-run_stamp_).toSec());
    if(dt<=0) {visualize(); return;}
    if(elapsed_>=curve_.duration && state_!="SETTLING") {
      state_="SETTLING"; settle_time_=Clock::now(); stable_time_=Time{};
    }
    KDL::Vector p,v,a; curve_.sample(elapsed_,p,v,&a); target_=origin_+p;
    const KDL::Vector error=target_-actual_.p; const double e=error.Norm();
    if(e>curve_.error) {finish("FAULT","TCP tracking error exceeded limit"); visualize(); return;}
    const auto feedforward=accelerationFeedforward(v,a,curve_.acceleration_ff_time,curve_.speed);
    const auto output=correction_.step(feedforward,error,dt,curve_.kp,curve_.ki,curve_.speed,
                                      curve_.correction_fraction,state_=="SETTLING",curve_.tolerance,v-trend_velocity_,curve_.kd);
    command(output);
    // Compare measured TCP to the reference at the feedback's timestamp, never
    // to a newer target. Count each physical feedback sample once in RMS/trace.
    if(feedback_stamp_>sample_stamp_ && feedback_stamp_>=run_stamp_) {
      const double sample_t=(feedback_stamp_-run_stamp_).toSec();
      KDL::Vector measured_p,measured_v;
      curve_.sample(sample_t,measured_p,measured_v);
      const KDL::Vector measured_target=origin_+measured_p;
      const auto measured_error=measured_target-actual_.p;
      reported_error_=measured_error.Norm();
      peak_=std::max(peak_,reported_error_); sum2_+=reported_error_*reported_error_; ++samples_;
      const auto tangent=measured_v.Norm()>1e-8 ? measured_v/measured_v.Norm() : KDL::Vector::Zero();
      const double along=KDL::dot(measured_error,tangent);
      const double normal=(measured_error-tangent*along).Norm();
      std_msgs::Float64MultiArray sample;
      sample.data={sample_t,measured_target.x(),measured_target.y(),measured_target.z(),
        actual_.p.x(),actual_.p.y(),actual_.p.z(),reported_error_,actual_speed_,
        measured_v.Norm(),output.Norm(),trend_velocity_.Norm(),(now-feedback_stamp_).toSec(),
        sample_t>=curve_.duration ? 1.0 : 0.0,along,normal};
      samples_pub_.publish(sample); sample_stamp_=feedback_stamp_;
      trace_.header.stamp=feedback_stamp_; auto point=pose(actual_.p); point.header.stamp=feedback_stamp_;
      trace_.poses.push_back(point);
      if(trace_.poses.size()>10000) trace_.poses.erase(trace_.poses.begin());
      if(samples_%5==0) actual_pub_.publish(trace_);
    }
    if(elapsed_>=curve_.duration) {
      const bool stable=e<=curve_.tolerance && std::max(actual_speed_,trend_velocity_.Norm())<=curve_.settle_speed;
      if(!stable) stable_time_=Time{};
      else if(stable_time_==Time{}) stable_time_=Clock::now();
      if(stable && age(stable_time_)>=0.5) finish("COMPLETED","Final TCP within tolerance for 0.5 s; disable requested");
      else if(age(settle_time_)>curve_.settle_timeout) finish("FAULT","Final TCP convergence timed out");
    }
    visualize();
  }
public:
  Tracker()
  {
    pn_.param<std::string>("base_link",base_,"base_link"); pn_.param<std::string>("tool_link",tool_,"tcp_link");
    pn_.param<std::string>("jog_ns",jog_ns_,"/cartesian_jog");
    pn_.param("velocity_window",velocity_window_duration_,0.08);
    if(!std::isfinite(velocity_window_duration_) || velocity_window_duration_<0.02 || velocity_window_duration_>0.25)
      throw std::runtime_error("velocity_window must be in 0.02..0.25 s");
    pn_.param("check_feasibility",check_feasibility_,true);
    preflight_=nh_.serviceClient<std_srvs::Trigger>("/trajectory_preparation/check_current");
    pn_.param("feedback_timeout",feedback_timeout_,0.15); pn_.param("state_timeout",state_timeout_,0.35);
    if(!std::isfinite(feedback_timeout_) || feedback_timeout_<=0 || !std::isfinite(state_timeout_) || state_timeout_<=0)
      throw std::runtime_error("Invalid watchdog timeouts");
    std::string xml; nh_.getParam("robot_description",xml); KDL::Tree tree;
    if(!kdl_parser::treeFromString(xml,tree) || !tree.getChain(base_,tool_,chain_)) throw std::runtime_error("Cannot construct base-to-TCP KDL chain");
    for(unsigned i=0;i<chain_.getNrOfSegments();++i) if(chain_.getSegment(i).getJoint().getType()!=KDL::Joint::None)
      joint_names_.push_back(chain_.getSegment(i).getJoint().getName());
    fk_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    cmd_=nh_.advertise<geometry_msgs::TwistStamped>(jog_ns_+"/command",1);
    state_pub_=pn_.advertise<std_msgs::String>("state",1,true);
    readiness_pub_=pn_.advertise<std_msgs::String>("readiness",1,true); metrics_=pn_.advertise<std_msgs::Float64MultiArray>("metrics",1);
    run_start_pub_=pn_.advertise<std_msgs::Header>("run_start",1,true);
    samples_pub_=pn_.advertise<std_msgs::Float64MultiArray>("samples",50);
    reference_pub_=pn_.advertise<nav_msgs::Path>("reference_path",1,true); actual_pub_=pn_.advertise<nav_msgs::Path>("actual_path",1,true);
    markers_=pn_.advertise<visualization_msgs::MarkerArray>("markers",1);
    std::string topic; pn_.param<std::string>("joint_states_topic",topic,"/cartesian_jog/sdk_joint_states");
    joints_=nh_.subscribe(topic,1,&Tracker::feedback,this);
    jog_state_sub_=nh_.subscribe<std_msgs::String>(jog_ns_+"/state",1,[this](const std_msgs::String::ConstPtr& s){jog_state_=s->data; state_time_=Clock::now();});
    enable_=nh_.serviceClient<std_srvs::SetBool>(jog_ns_+"/set_enabled");
    preview_srv_=pn_.advertiseService("preview",&Tracker::preview,this); start_srv_=pn_.advertiseService("start",&Tracker::start,this);
    stop_srv_=pn_.advertiseService("stop",&Tracker::stop,this); timer_=nh_.createWallTimer(ros::WallDuration(0.02),&Tracker::tick,this);
  }
  ~Tracker() { if(owned_) {command(); disable();} }
};
int main(int argc,char** argv)
{
  ros::init(argc,argv,"trajectory_tracking");
  try {Tracker node; ros::spin();} catch(const std::exception& e) {ROS_FATAL("%s",e.what()); return 1;}
  return 0;
}
