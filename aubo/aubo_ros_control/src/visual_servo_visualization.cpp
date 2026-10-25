// Passive observer. No SDK, motion publishers, or enable/reset service clients.
#include <aubo_ros_control/VisualServoStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>

namespace aubo_ros_control {
class VisualServoVisualization {
  using Clock = std::chrono::steady_clock;
public:
  VisualServoVisualization() : private_("~"), listener_(buffer_) {
    std::string controller_ns;
    private_.param<std::string>("controller_ns", controller_ns, "/aubo_visual_servo");
    controller_.reset(new ros::NodeHandle(controller_ns));
    controller_->param<std::string>("base_link", base_, "base_link");
    controller_->param<std::string>("control_link", tool_, "tcp_link");
    controller_->param<std::string>("camera_link", camera_, "camera_color_optical_frame");
    controller_->param<std::string>("servo_mode", mode_, "eye_in_hand");
    controller_->param("hybrid_enabled", hybrid_, false);
    std::string description;
    KDL::Tree tree;
    if (!nh_.getParam("robot_description", description) ||
        !kdl_parser::treeFromString(description, tree) ||
        !tree.getChain(base_, tool_, chain_))
      throw std::runtime_error("visualization: missing robot model or TCP chain");
    if (!controller_->getParam("joint_names", names_) || names_.size()!=chain_.getNrOfJoints())
      throw std::runtime_error("visualization: joint_names do not match TCP chain");
    solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    joints_.resize(names_.size());
    refreshParameters();
    std::string joint_topic, target_topic, state_topic;
    controller_->param<std::string>("joint_states_topic", joint_topic, "/joint_states");
    controller_->param<std::string>("target_topic", target_topic, "/visual_servo/target_pose");
    controller_->param<std::string>("state_topic", state_topic, "/visual_servo/state");
    joint_sub_ = nh_.subscribe(joint_topic, 1, &VisualServoVisualization::joint, this);
    target_sub_ = nh_.subscribe(target_topic, 1, &VisualServoVisualization::target, this);
    state_sub_ = nh_.subscribe(state_topic, 1, &VisualServoVisualization::state, this);
    status_pub_ = nh_.advertise<VisualServoStatus>("/visual_servo/status", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visual_servo/markers", 1);
    timer_ = nh_.createWallTimer(ros::WallDuration(.1), &VisualServoVisualization::tick, this);
    params_timer_ = nh_.createWallTimer(ros::WallDuration(1), [this](const ros::WallTimerEvent&) { refreshParameters(); });
    ROS_INFO("[visual_servo/view] 只读显示：%s；反馈=%s，目标=%s", mode_.c_str(), joint_topic.c_str(), target_topic.c_str());
  }

private:
  static geometry_msgs::Point point(const KDL::Vector& v) {
    geometry_msgs::Point p; p.x=v.x(); p.y=v.y(); p.z=v.z(); return p;
  }
  static bool finite(const geometry_msgs::Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }
  static double age(ros::Time source, Clock::time_point received) {
    if (received==Clock::time_point{}) return -1;
    return std::max((ros::Time::now()-source).toSec(),
                    std::chrono::duration<double>(Clock::now()-received).count());
  }
  void refreshParameters() {
    controller_->param("target_timeout", target_timeout_, .2);
    controller_->param("feedback_timeout", feedback_timeout_, .15);
    controller_->param("position_deadband", deadband_, .004);
    if (!std::isfinite(target_timeout_) || target_timeout_<=0) target_timeout_=.2;
    if (!std::isfinite(feedback_timeout_) || feedback_timeout_<=0) feedback_timeout_=.15;
    if (!std::isfinite(deadband_) || deadband_<0) deadband_=.004;
    std::vector<double> desired(3,0), offset(3,0);
    controller_->getParam("desired_target_position", desired);
    controller_->getParam("target_offset", offset);
    if (desired.size()!=3) desired.assign(3,0);
    if (offset.size()!=3) offset.assign(3,0);
    const char* axes[]={"x","y","z"};
    for (unsigned i=0;i<3;++i) {
      controller_->param(std::string("desired_target_")+axes[i], desired[i], desired[i]);
      controller_->param(std::string("target_offset_")+axes[i], offset[i], offset[i]);
    }
    desired_ = KDL::Vector(desired[0],desired[1],desired[2]);
    offset_ = KDL::Vector(offset[0],offset[1],offset[2]);
  }
  void joint(const sensor_msgs::JointState::ConstPtr& msg) {
    KDL::JntArray ordered(names_.size());
    for (std::size_t i=0;i<names_.size();++i) {
      auto found=std::find(msg->name.begin(),msg->name.end(),names_[i]);
      if (found==msg->name.end()) return;
      const auto index=std::distance(msg->name.begin(),found);
      if (static_cast<std::size_t>(index)>=msg->position.size() || !std::isfinite(msg->position[index])) return;
      ordered(i)=msg->position[index];
    }
    joints_=ordered;
    joint_stamp_=msg->header.stamp.isZero()?ros::Time::now():msg->header.stamp;
    joint_received_=Clock::now();
  }
  bool acceptTarget(const geometry_msgs::PoseStamped& source, Clock::time_point received) {
    geometry_msgs::PoseStamped converted;
    try {
      const auto& destination=mode_=="eye_in_hand"?tool_:base_;
      converted=source.header.frame_id==destination ? source :
          buffer_.transform(source,destination,ros::Duration(0));
    } catch (const tf2::TransformException&) { return false; }
    if (!finite(converted.pose.position)) return false;
    if (target_received_!=Clock::time_point{} && source.header.stamp<target_stamp_) return true;
    target_=converted.pose.position;
    target_stamp_=source.header.stamp;
    target_received_=received;
    return true;
  }
  void target(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!finite(msg->pose.position)) return;
    auto source=*msg;
    if (source.header.frame_id.empty()) {
      if (mode_!="eye_in_hand") return;
      source.header.frame_id=camera_;
    }
    if (source.header.stamp.isZero() || source.header.stamp>ros::Time::now())
      source.header.stamp=ros::Time::now();
    const auto received=Clock::now();
    if (!acceptTarget(source,received)) {
      pending_.push_back({source,received});
      if (pending_.size()>10) pending_.pop_front();
    }
  }
  void state(const std_msgs::String::ConstPtr& msg) { state_=msg->data; }

  void tick(const ros::WallTimerEvent&) {
    // Retry with zero wait: TF may arrive just after the camera pose. Bounded
    // history and all computation live in this observer, never the controller.
    for (auto i=pending_.rbegin();i!=pending_.rend();++i)
      if (acceptTarget(i->first,i->second)) break;
    while (!pending_.empty() && std::chrono::duration<double>(Clock::now()-pending_.front().second).count()>target_timeout_)
      pending_.pop_front();
    VisualServoStatus s;
    s.header.stamp=ros::Time::now(); s.header.frame_id=base_;
    s.servo_mode=mode_; s.state=state_;
    s.error_frame=mode_=="eye_in_hand"?tool_:base_;
    s.feedback_age=age(joint_stamp_,joint_received_);
    s.target_age=age(target_stamp_,target_received_);
    s.feedback_valid=s.feedback_age>=0 && s.feedback_age<=feedback_timeout_;
    s.target_valid=s.target_age>=0 && s.target_age<=target_timeout_;
    s.position_deadband=deadband_;
    KDL::Frame tcp;
    if (s.feedback_valid && solver_->JntToCart(joints_,tcp)<0) s.feedback_valid=false;
    if (s.feedback_valid) {
      s.tcp_position=point(tcp.p);
      if (trail_.empty() || (tcp.p-KDL::Vector(trail_.back().x,trail_.back().y,trail_.back().z)).Norm()>.0005)
        trail_.push_back(s.tcp_position);
      if (trail_.size()>400) trail_.pop_front();
    }
    s.error_valid=s.feedback_valid && s.target_valid && !hybrid_ &&
                  (mode_=="eye_in_hand" || mode_=="eye_to_hand");
    if (s.error_valid) {
      KDL::Vector observed(target_.x,target_.y,target_.z), error;
      if (mode_=="eye_in_hand") {
        error=observed-desired_;
        s.observed_position=point(tcp*observed);
        s.goal_position=point(tcp.p+tcp.M*error);
      } else {
        s.observed_position=target_;
        s.goal_position=point(observed+offset_);
        error=observed+offset_-tcp.p;
      }
      s.position_error.x=error.x(); s.position_error.y=error.y(); s.position_error.z=error.z();
      s.position_error_norm=error.Norm();
      s.error_valid=finite(s.goal_position) && finite(s.observed_position) && std::isfinite(s.position_error_norm);
    }
    s.detail=hybrid_?"混合控制：不将独立PBVS几何误差作为规划误差":
      !s.feedback_valid?"反馈话题缺失或过期":!s.target_valid?"目标缺失、过期或TF尚未就绪":
      !s.error_valid?"几何计算无效":"只读几何参考；不代表控制器接收或运动许可";
    status_pub_.publish(s);
    publishMarkers(s);
  }

  void publishMarkers(const VisualServoStatus& s) {
    visualization_msgs::MarkerArray array;
    using Marker=visualization_msgs::Marker;
    for (int id=0;id<6;++id) {
      Marker m;
      m.header=s.header; m.ns="visual_servo_geometry"; m.id=id;
      m.pose.orientation.w=1; m.lifetime=ros::Duration(.5);
      m.color.a=1;
      m.action=Marker::ADD;
      if (id<=2) {
        m.type=Marker::SPHERE; m.scale.x=m.scale.y=m.scale.z=.018;
        m.pose.position=id==0?s.tcp_position:id==1?s.observed_position:s.goal_position;
        m.color.r=id==1?1:0; m.color.g=1; m.color.b=id==2?1:0;
        if ((id==0 && !s.feedback_valid) || (id>0 && !s.error_valid)) m.action=Marker::DELETE;
      } else if (id==3) {
        m.type=Marker::ARROW; m.scale.x=.004; m.scale.y=.009; m.scale.z=.012;
        m.color.r=1; m.color.g=.3;
        m.points={s.tcp_position,s.goal_position};
        if (!s.error_valid || s.position_error_norm<1e-6) m.action=Marker::DELETE;
      } else if (id==4) {
        m.type=Marker::LINE_STRIP; m.scale.x=.003;
        m.color.r=.2; m.color.g=.55; m.color.b=1;
        m.points.assign(trail_.begin(),trail_.end());
        if (m.points.size()<2) m.action=Marker::DELETE;
      } else {
        m.type=Marker::TEXT_VIEW_FACING; m.scale.z=.025;
        m.pose.position=s.tcp_position; m.pose.position.z+=.12;
        m.color.r=s.error_valid?.8:1; m.color.g=s.error_valid?1:.5; m.color.b=.9;
        m.text=s.state+"\n"+(s.error_valid?std::to_string(s.position_error_norm*1000)+" mm":s.detail);
      }
      array.markers.push_back(m);
    }
    marker_pub_.publish(array);
  }

  ros::NodeHandle nh_,private_;
  std::unique_ptr<ros::NodeHandle> controller_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_;
  KDL::JntArray joints_;
  std::string base_,tool_,camera_,mode_,state_{"WAITING"};
  std::vector<std::string> names_;
  KDL::Vector desired_,offset_;
  geometry_msgs::Point target_;
  ros::Time target_stamp_,joint_stamp_;
  Clock::time_point target_received_{},joint_received_{};
  double target_timeout_{.2},feedback_timeout_{.15},deadband_{.004};
  bool hybrid_{false};
  std::deque<std::pair<geometry_msgs::PoseStamped,Clock::time_point>> pending_;
  std::deque<geometry_msgs::Point> trail_;
  ros::Subscriber joint_sub_,target_sub_,state_sub_;
  ros::Publisher status_pub_,marker_pub_;
  ros::WallTimer timer_,params_timer_;
};
}

int main(int argc,char** argv) {
  setlocale(LC_ALL,"");
  ros::init(argc,argv,"visual_servo_visualization");
  try { aubo_ros_control::VisualServoVisualization observer; ros::spin(); }
  catch (const std::exception& e) { ROS_ERROR("[visual_servo/view] %s",e.what()); return 1; }
  return 0;
}
