/**
 * AUBO i5 眼在手上/眼在手外统一 PBVS 视觉伺服实现。
 *
 * 两种安装方式共用感知与控制核心，通过参数选择位姿误差模型；Gazebo 和真实
 * 机械臂后端分别负责仿真话题下发与 SDK TCP2CANBUS 连续轨迹下发。
 */

#include <aubo_ros_control/direct_sdk_backend.h>
#include <aubo_ros_control/visual_servo.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <clocale>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace {
using aubo_ros_control::visual_servo_internal::applyDeadband;
using aubo_ros_control::visual_servo_internal::clampValue;
using aubo_ros_control::visual_servo_internal::CommandQueue;
using aubo_ros_control::visual_servo_internal::DirectSdkBackend;
using aubo_ros_control::visual_servo_internal::finitePoint;
using aubo_ros_control::visual_servo_internal::JointPoint;
using aubo_ros_control::visual_servo_internal::kDof;

Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd &jacobian,
                                    double lambda) {
  // 阻尼最小二乘伪逆，在机械臂接近奇异位形时保持数值稳定。
  const Eigen::MatrixXd regularized =
      jacobian * jacobian.transpose() +
      lambda * lambda *
          Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows());// P*P^T + λ^2*I
  return jacobian.transpose() *
         regularized.ldlt().solve(
             Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows()));// P(P*P^T + λ^2*I)^-1
}

Eigen::Matrix3d kdlRotationToEigen(const KDL::Rotation &rotation) {
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result(row, col) = rotation(row, col);
  return result;
}

Eigen::Vector3d rotationLog(const Eigen::Matrix3d &rotation) {
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || !angle_axis.axis().allFinite())
    return Eigen::Vector3d::Zero();
  return angle_axis.axis() * angle_axis.angle();
}

template <typename T>
bool getSixValues(ros::NodeHandle &nh, const std::string &name,
                  std::array<T, kDof> &values) {
  std::vector<double> parameter;
  if (!nh.getParam(name, parameter) || parameter.size() != kDof) {
    ROS_ERROR("[visual_servo] 参数 ~%s 必须包含 6 个数", name.c_str());
    return false;
  }
  for (std::size_t i = 0; i < kDof; ++i)
    values[i] = static_cast<T>(parameter[i]);
  return true;
}

} // namespace

namespace aubo_ros_control {

// 将内部状态转换成对外发布和日志使用的稳定字符串。
const char *VisualServo::stateName(ServoState state) {
  switch (state) {
  case ServoState::DISABLED:
    return "DISABLED";
  case ServoState::WAITING:
    return "WAITING";
  case ServoState::SEARCH_INITIAL:
    return "SEARCH_INITIAL";
  case ServoState::TRACKING:
    return "TRACKING";
  case ServoState::ALIGNED:
    return "ALIGNED";
  case ServoState::COAST:
    return "COAST";
  case ServoState::SEARCH_RECOVERY:
    return "SEARCH_RECOVERY";
  case ServoState::HOLD:
    return "HOLD";
  }
  return "UNKNOWN";
}

VisualServo::VisualServo()
    : nh_(), private_nh_("~"), tf_listener_(tf_buffer_),
      queue_(readQueueCapacity()) {
  valid_ = loadParameters() && initializeKinematics();
  if (!valid_)
    return;
  state_ = enabled_ ? ServoState::WAITING : ServoState::DISABLED;

  state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
  legacy_state_pub_ = private_nh_.advertise<std_msgs::String>("state", 1, true);
  target_sub_ =
      nh_.subscribe(target_topic_, 1, &VisualServo::targetCallback, this);
  enable_service_ = nh_.advertiseService("/visual_servo/set_enabled",
                                         &VisualServo::setEnabled, this);
  reset_service_ =
      nh_.advertiseService("/visual_servo/reset", &VisualServo::reset, this);

  if (backend_ == "gazebo") {
    // 对于gazebo模式 直接借助于ros_control 通过关节状态控制器和关节位置控制器
    joint_sub_ = nh_.subscribe(joint_states_topic_, 2,
                               &VisualServo::jointStateCallback, this); // 订阅关节位置话题
    if (!setupGazeboPublishers()) {   // 设置六个关节位置的话题发布者
      valid_ = false;
      return;
    }
    output_timer_ = nh_.createTimer(ros::Duration(1.0 / output_rate_),
                                    &VisualServo::gazeboOutput, this);  // 创建输出定时器
  } else if (backend_ == "sdk") {
    // 对于sdk实机模式 直接通过SDK下发关节位置 缓冲队列+限速限加速度+向控制柜发送的频率
    sdk_.reset(new DirectSdkBackend(queue_, velocity_limits_,
                                    acceleration_limits_, output_rate_));
    if (!sdk_->connect(private_nh_)) {
      valid_ = false;
      return;
    }
    sdk_joint_pub_ =
        nh_.advertise<sensor_msgs::JointState>(joint_states_topic_, 2);
    sdk_state_timer_ = nh_.createTimer(ros::Duration(0.02),
                                       &VisualServo::sdkStateUpdate, this);
    // 同步读取一次初始状态，防止积分器从全零关节位置开始运行。
    JointPoint initial{};
    if (sdk_->readState(initial))
      setJointFeedback(initial);
  } else {
    ROS_ERROR("[visual_servo] backend 只能是 gazebo 或 sdk，当前为 '%s'",
              backend_.c_str());
    valid_ = false;
    return;
  }

  setupDynamicReconfigure();
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                   &VisualServo::controlLoop, this);
  publishState();
  ROS_INFO("[visual_servo] 初始化完成：mode=%s, backend=%s, chain=%s -> %s",
           servo_mode_.c_str(), backend_.c_str(), base_link_.c_str(),
           control_link_.c_str());
}

bool VisualServo::valid() const { return valid_; }

bool VisualServo::healthy() const {
  return backend_ != "sdk" || (sdk_ && sdk_->healthy());
}

std::size_t VisualServo::readQueueCapacity() {
  int capacity = 80;
  private_nh_.param<int>("command_queue_capacity", capacity, 80);// 获取参数服务器中 控制队列的长度
  return static_cast<std::size_t>(std::max(2, capacity));
}

bool VisualServo::loadParameters() {
  private_nh_.param<std::string>("backend", backend_, "gazebo");
  private_nh_.param<std::string>("servo_mode", servo_mode_, "eye_in_hand");
  private_nh_.param<std::string>("base_link", base_link_, "base_link");// 基坐标系
  private_nh_.param<std::string>("camera_link", camera_link_,
                                 "camera_color_optical_frame");// 相机坐标系
  private_nh_.param<std::string>("control_link", control_link_,
                                 "tcp_link");// 控制坐标系：夹爪 TCP，也是雅可比链末端
  private_nh_.param<std::string>("target_topic", target_topic_,
                                 "/visual_servo/target_pose");
  private_nh_.param<std::string>("state_topic", state_topic_,
                                 "/visual_servo/state");
  private_nh_.param<std::string>("joint_states_topic", joint_states_topic_,
                                 "/joint_states");
  private_nh_.param<std::string>("loss_strategy", loss_strategy_,
                                 "coast_then_open");
  private_nh_.param<double>("control_rate", control_rate_, 100.0);
  private_nh_.param<double>("output_rate", output_rate_, 200.0);
  private_nh_.param<double>("linear_gain", linear_gain_, 0.8);
  private_nh_.param<double>("angular_gain", angular_gain_, 0.5);
  if (!private_nh_.getParam("max_linear_velocity", max_linear_velocity_))
    private_nh_.param<double>("max_camera_linear_velocity",
                              max_linear_velocity_, 0.08);
  if (!private_nh_.getParam("max_angular_velocity", max_angular_velocity_))
    private_nh_.param<double>("max_camera_angular_velocity",
                              max_angular_velocity_, 0.2);
  private_nh_.param<double>("position_deadband", position_deadband_, 0.004);
  private_nh_.param<double>("orientation_deadband", orientation_deadband_,
                            0.02);
  private_nh_.param<double>("alignment_hold_time", alignment_hold_time_, 0.35);
  private_nh_.param<double>("alignment_release_multiplier",
                            alignment_release_multiplier_, 2.0);
  private_nh_.param<double>("dls_lambda", dls_lambda_, 0.04);
  private_nh_.param<double>("joint_limit_margin", joint_limit_margin_, 0.08);
  private_nh_.param<double>("target_timeout", target_timeout_, 0.2);
  private_nh_.param<double>("minimum_safe_target_distance",
                            minimum_safe_target_distance_, 0.0);
  private_nh_.param<double>("coast_duration", coast_duration_, 0.35);
  private_nh_.param<double>("coast_decay_time", coast_decay_time_, 0.18);
  private_nh_.param<double>("recovery_delay", recovery_delay_, 1.0);
  private_nh_.param<double>("reacquire_hold_time", reacquire_hold_time_, 0.3);
  private_nh_.param<double>("search_timeout", search_timeout_, 8.0);
  private_nh_.param<double>("open_posture_gain", open_posture_gain_, 0.7);
  private_nh_.param<double>("search_velocity_limit", search_velocity_limit_,
                            0.2);
  private_nh_.param<double>("feedback_blend", feedback_blend_, 0.02);
  private_nh_.param<bool>("use_orientation_control", use_orientation_control_,
                          false);// 是否使用姿态控制(计算角度误差)
  private_nh_.param<bool>("initial_search_enabled", initial_search_enabled_,
                          servo_mode_ == "eye_in_hand");// 是否机械臂初始化到观察姿态
  private_nh_.param<bool>("start_enabled", enabled_, false);// 是否启动视觉伺服器
  
  if (servo_mode_ != "eye_in_hand" && servo_mode_ != "eye_to_hand") {
    ROS_ERROR("[visual_servo] servo_mode 必须是 eye_in_hand 或 eye_to_hand");
    return false;
  }

  if (control_rate_ <= 0.0 || output_rate_ < control_rate_ ||
      output_rate_ > 500.0) {
    ROS_ERROR("[visual_servo] 频率要求 0 < control_rate <= output_rate <= 500");
    return false;
  }
  if (loss_strategy_ != "stop" && loss_strategy_ != "coast" &&
      loss_strategy_ != "coast_then_open") {
    ROS_ERROR(
        "[visual_servo] loss_strategy 必须是 stop、coast 或 coast_then_open");
    return false;
  }
  if (minimum_safe_target_distance_ < 0.0) {
    ROS_ERROR("[visual_servo] minimum_safe_target_distance 不能为负数");
    return false;
  }
  if (orientation_deadband_ < 0.0 || alignment_hold_time_ < 0.0 ||
      alignment_release_multiplier_ < 1.0) {
    ROS_ERROR("[visual_servo] 对齐死区/保持时间不能为负，退出倍数必须 >= 1");
    return false;
  }
  if (recovery_delay_ < 0.0 || reacquire_hold_time_ < 0.0) {
    ROS_ERROR("[visual_servo] 恢复等待和重捕获稳定时间不能为负数");
    return false;
  }

  if (!private_nh_.getParam("joint_names", joint_names_) ||
      joint_names_.size() != kDof) {
    ROS_ERROR("[visual_servo] ~joint_names 必须包含 6 个关节名");
    return false;
  }
  if (!getSixValues(private_nh_, "joint_lower_limits", lower_limits_) ||
      !getSixValues(private_nh_, "joint_upper_limits", upper_limits_) ||
      !getSixValues(private_nh_, "joint_velocity_limits", velocity_limits_) ||
      !getSixValues(private_nh_, "joint_acceleration_limits",
                    acceleration_limits_) ||
      !getSixValues(private_nh_, "open_posture", open_posture_))
    return false;
  // 兼容旧参数 open_posture，同时允许初始搜索和丢失恢复使用不同观察姿态。
  initial_search_posture_ = open_posture_;
  recovery_posture_ = open_posture_;
  if (private_nh_.hasParam("initial_search_posture") &&
      !getSixValues(private_nh_, "initial_search_posture",
                    initial_search_posture_))
    return false;
  if (private_nh_.hasParam("recovery_posture") &&
      !getSixValues(private_nh_, "recovery_posture", recovery_posture_))
    return false;

  std::vector<double> desired_position, desired_rpy;
  if (!private_nh_.getParam("desired_target_position", desired_position) ||
      desired_position.size() != 3 ||
      !private_nh_.getParam("desired_target_rpy", desired_rpy) ||
      desired_rpy.size() != 3) {
    ROS_ERROR("[visual_servo] desired_target_position/desired_target_rpy "
              "必须各含 3 个数");
    return false;
  }
  desired_position_ = Eigen::Vector3d(desired_position[0], desired_position[1],
                                      desired_position[2]);
  std::vector<double> target_offset;
  if (!private_nh_.getParam("target_offset", target_offset))
    target_offset = {0.0, 0.0, 0.0};
  if (target_offset.size() != 3) {
    ROS_ERROR("[visual_servo] target_offset 必须包含 3 个数");
    return false;
  }
  target_offset_ =
      Eigen::Vector3d(target_offset[0], target_offset[1], target_offset[2]);
  desired_rpy_ =
      Eigen::Vector3d(desired_rpy[0], desired_rpy[1], desired_rpy[2]);
  desired_rotation_ =
      (Eigen::AngleAxisd(desired_rpy_.z(), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(desired_rpy_.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(desired_rpy_.x(), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return true;
}

void VisualServo::setupDynamicReconfigure() {
  // dynamic_reconfigure::Server 构造时从参数服务器读取初值。先写入 YAML
  // 已解析出的实际值，避免眼在手上/眼在手外配置被 .cfg 默认值覆盖。
  private_nh_.setParam("linear_gain", linear_gain_);
  private_nh_.setParam("angular_gain", angular_gain_);
  private_nh_.setParam("max_linear_velocity", max_linear_velocity_);
  private_nh_.setParam("max_angular_velocity", max_angular_velocity_);
  private_nh_.setParam("position_deadband", position_deadband_);
  private_nh_.setParam("orientation_deadband", orientation_deadband_);
  private_nh_.setParam("alignment_hold_time", alignment_hold_time_);
  private_nh_.setParam("alignment_release_multiplier",
                       alignment_release_multiplier_);
  private_nh_.setParam("dls_lambda", dls_lambda_);
  private_nh_.setParam("feedback_blend", feedback_blend_);
  private_nh_.setParam("desired_target_x", desired_position_.x());
  private_nh_.setParam("desired_target_y", desired_position_.y());
  private_nh_.setParam("desired_target_z", desired_position_.z());
  private_nh_.setParam("target_offset_x", target_offset_.x());
  private_nh_.setParam("target_offset_y", target_offset_.y());
  private_nh_.setParam("target_offset_z", target_offset_.z());
  private_nh_.setParam("desired_roll", desired_rpy_.x());
  private_nh_.setParam("desired_pitch", desired_rpy_.y());
  private_nh_.setParam("desired_yaw", desired_rpy_.z());
  private_nh_.setParam("use_orientation_control", use_orientation_control_);
  private_nh_.setParam("minimum_safe_target_distance",
                       minimum_safe_target_distance_);
  private_nh_.setParam("loss_strategy", loss_strategy_);
  private_nh_.setParam("target_timeout", target_timeout_);
  private_nh_.setParam("coast_duration", coast_duration_);
  private_nh_.setParam("coast_decay_time", coast_decay_time_);
  private_nh_.setParam("recovery_delay", recovery_delay_);
  private_nh_.setParam("reacquire_hold_time", reacquire_hold_time_);
  private_nh_.setParam("search_timeout", search_timeout_);
  private_nh_.setParam("open_posture_gain", open_posture_gain_);
  private_nh_.setParam("search_velocity_limit", search_velocity_limit_);
  private_nh_.setParam("initial_search_enabled", initial_search_enabled_);

  reconfigure_server_.reset(new ReconfigureServer(private_nh_));
  ReconfigureServer::CallbackType callback = [this](VisualServoConfig &config,
                                                    uint32_t level) {
    reconfigureCallback(config, level);
  };
  reconfigure_server_->setCallback(callback);
}

void VisualServo::reconfigureCallback(VisualServoConfig &config,
                                      uint32_t level) {
  (void)level;
  std::lock_guard<std::mutex> control_lock(control_mutex_);

  linear_gain_ = config.linear_gain;
  angular_gain_ = config.angular_gain;
  max_linear_velocity_ = config.max_linear_velocity;
  max_angular_velocity_ = config.max_angular_velocity;
  position_deadband_ = config.position_deadband;
  orientation_deadband_ = config.orientation_deadband;
  alignment_hold_time_ = config.alignment_hold_time;
  alignment_release_multiplier_ = config.alignment_release_multiplier;
  dls_lambda_ = config.dls_lambda;
  feedback_blend_ = config.feedback_blend;

  desired_position_ =
      Eigen::Vector3d(config.desired_target_x, config.desired_target_y,
                      config.desired_target_z);
  target_offset_ = Eigen::Vector3d(
      config.target_offset_x, config.target_offset_y, config.target_offset_z);
  desired_rpy_ = Eigen::Vector3d(config.desired_roll, config.desired_pitch,
                                 config.desired_yaw);
  desired_rotation_ =
      (Eigen::AngleAxisd(desired_rpy_.z(), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(desired_rpy_.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(desired_rpy_.x(), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  use_orientation_control_ = config.use_orientation_control;
  minimum_safe_target_distance_ = config.minimum_safe_target_distance;

  if (config.loss_strategy == "stop" || config.loss_strategy == "coast" ||
      config.loss_strategy == "coast_then_open") {
    loss_strategy_ = config.loss_strategy;
  } else {
    ROS_WARN("[visual_servo] 忽略无效的动态丢失策略 '%s'",
             config.loss_strategy.c_str());
    config.loss_strategy = loss_strategy_;
  }
  target_timeout_ = config.target_timeout;
  coast_duration_ = config.coast_duration;
  coast_decay_time_ = config.coast_decay_time;
  recovery_delay_ = config.recovery_delay;
  reacquire_hold_time_ = config.reacquire_hold_time;
  search_timeout_ = config.search_timeout;
  open_posture_gain_ = config.open_posture_gain;
  search_velocity_limit_ = config.search_velocity_limit;
  initial_search_enabled_ = config.initial_search_enabled;

  // 丢弃按照旧参数计算、但还没有输出的关节点，让新参数从下一周期开始生效。
  queue_.clear();
  ROS_INFO_THROTTLE(1.0, "[visual_servo] rqt_reconfigure 参数已更新");
}

bool VisualServo::initializeKinematics() {
  std::string robot_description;
  if (!nh_.getParam("robot_description", robot_description)) {
    ROS_ERROR("[visual_servo] 缺少 /robot_description");
    return false;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_description, tree) ||
      !tree.getChain(base_link_, control_link_, chain_)) {
    ROS_ERROR("[visual_servo] 无法建立 KDL 链 %s -> %s", base_link_.c_str(),
              control_link_.c_str());
    return false;
  }
  if (chain_.getNrOfJoints() != kDof) {
    ROS_ERROR("[visual_servo] 控制链应有 6 个活动关节，实际为 %u",
              chain_.getNrOfJoints());
    return false;
  }
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
  jacobian_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
  return true;
}

bool VisualServo::setupGazeboPublishers() {
  if (!private_nh_.getParam("gazebo_command_topics", gazebo_topics_) ||
      gazebo_topics_.size() != kDof) {
    ROS_ERROR("[visual_servo] Gazebo 后端需要 6 个 ~gazebo_command_topics");
    return false;
  }
  for (const std::string &topic : gazebo_topics_)
    gazebo_publishers_.push_back(nh_.advertise<std_msgs::Float64>(topic, 1));
  return true;
}

void VisualServo::targetCallback(
    const geometry_msgs::PoseStamped::ConstPtr &message) {
  // 根据message的frame_id和servo_mode_，将目标位姿变换到控制坐标系下，并进行有效性检查。
  geometry_msgs::PoseStamped source_target = *message;
  geometry_msgs::PoseStamped control_target;
  geometry_msgs::PoseStamped camera_target;
  try {
    if (source_target.header.frame_id.empty()) {
      if (servo_mode_ == "eye_to_hand") {
        ROS_WARN_THROTTLE(1.0, "[visual_servo] 眼在手外目标必须提供 frame_id");
        return;
      }
      // 眼在手上感知默认输出相机光学坐标。
      source_target.header.frame_id = camera_link_;
    }

    if (servo_mode_ == "eye_in_hand") {
      // 相机只是测量坐标系。先保留相机深度作安全检查，
      // 再把目标变换到夹爪/TCP 坐标系计算对齐误差。
      camera_target = source_target.header.frame_id == camera_link_
                          ? source_target
                          : tf_buffer_.transform(source_target, camera_link_,
                                                 ros::Duration(0.03));// 变换到相机坐标系
      control_target = camera_target.header.frame_id == control_link_
                           ? camera_target
                           : tf_buffer_.transform(camera_target, control_link_,
                                                  ros::Duration(0.03));// 变换到夹爪/TCP 坐标系
    } else if (source_target.header.frame_id == base_link_) {
      control_target = source_target;// 眼在手外感知直接使用基坐标系下的目标
    } else {
      control_target =
          tf_buffer_.transform(source_target, base_link_, ros::Duration(0.03));
    }
  } catch (const tf2::TransformException &exception) {
    ROS_WARN_THROTTLE(1.0, "[visual_servo] 目标无法变换到 %s: %s",
                      servo_mode_ == "eye_in_hand" ? control_link_.c_str()
                                                   : base_link_.c_str(),
                      exception.what());
    return;
  }
  if (!std::isfinite(control_target.pose.position.x) ||
      !std::isfinite(control_target.pose.position.y) ||
      !std::isfinite(control_target.pose.position.z) ||
      (servo_mode_ == "eye_in_hand" &&
       (!std::isfinite(camera_target.pose.position.z) ||
        camera_target.pose.position.z <= 0.0))) {
    ROS_WARN_THROTTLE(1.0, "[visual_servo] 忽略无效或位于相机后方的目标");
    return;
  }
  double minimum_safe_distance = 0.0;
  {
    std::lock_guard<std::mutex> control_lock(control_mutex_);
    minimum_safe_distance = minimum_safe_target_distance_;// 获取安全距离阈值
  }
  if (servo_mode_ == "eye_in_hand" && minimum_safe_distance > 0.0 &&
      camera_target.pose.position.z < minimum_safe_distance) {// 如果目标距离小于安全阈值，锁存停止 
    safety_stop_.store(true);
    queue_.clear();
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      have_target_ = false;
    }
    ROS_ERROR_THROTTLE(1.0,
                       "[visual_servo] 目标距离 %.3f m 小于安全阈值 %.3f "
                       "m，锁存停止；请排除危险后复位",
                       camera_target.pose.position.z, minimum_safe_distance);
    return;
  }
  const ros::Time now = ros::Time::now();
  ros::Time measurement_time =
      message->header.stamp.isZero() ? now : message->header.stamp;
  if (measurement_time > now)
    measurement_time = now;
  std::lock_guard<std::mutex> lock(target_mutex_);// 上锁 存入target位置、timestamp、have_target_标志
  target_pose_ = control_target.pose;
  last_target_time_ = measurement_time;
  have_target_ = true;
}

bool VisualServo::setEnabled(std_srvs::SetBool::Request &request,
                             std_srvs::SetBool::Response &response) {
  // 接收rviz端发送的服务请求
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  enabled_ = request.data;
  safety_stop_.store(false);
  queue_.clear();
  last_tracking_velocity_.setZero();
  alignment_candidate_ = false;
  aligned_latched_ = false;
  search_reacquire_pending_ = false;
  reacquire_candidate_ = false;
  if (!enabled_)
    transitionTo(ServoState::DISABLED);
  else {
    // 每次重新启用后必须等待新的相机测量，不能复用旧目标。
    std::lock_guard<std::mutex> target_lock(target_mutex_);
    have_target_ = false;
    have_ever_tracked_ = false;
    transitionTo(ServoState::WAITING);
  }
  response.success = true;
  response.message = enabled_
                         ? "visual servo enabled; waiting for a fresh target"
                         : "visual servo disabled; holding position";
  return true;
}

bool VisualServo::reset(std_srvs::Trigger::Request &,
                        std_srvs::Trigger::Response &response) {
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  {
    std::lock_guard<std::mutex> target_lock(target_mutex_);
    have_target_ = false;
    have_ever_tracked_ = false;
  }
  queue_.clear();
  safety_stop_.store(false);
  last_tracking_velocity_.setZero();
  alignment_candidate_ = false;
  aligned_latched_ = false;
  search_reacquire_pending_ = false;
  reacquire_candidate_ = false;
  transitionTo(enabled_ ? ServoState::WAITING : ServoState::DISABLED);
  response.success = true;
  response.message = "visual target and loss-recovery state cleared";
  return true;
}

void VisualServo::jointStateCallback(
    const sensor_msgs::JointState::ConstPtr &message) {
  JointPoint ordered{};
  for (std::size_t i = 0; i < kDof; ++i) {
    const auto found =
        std::find(message->name.begin(), message->name.end(), joint_names_[i]);
    if (found == message->name.end())
      return;
    const std::size_t index =
        static_cast<std::size_t>(std::distance(message->name.begin(), found));
    if (index >= message->position.size())
      return;
    ordered[i] = message->position[index];
  }
  if (finitePoint(ordered))
    setJointFeedback(ordered);
}

void VisualServo::setJointFeedback(const JointPoint &position) {
  std::lock_guard<std::mutex> lock(joint_mutex_);
  feedback_position_ = position;
  have_joint_state_ = true;
  last_joint_time_ = ros::Time::now();
}

void VisualServo::sdkStateUpdate(const ros::TimerEvent &) {
  JointPoint position{};
  if (!sdk_ || !sdk_->readState(position)) {
    ROS_ERROR_THROTTLE(1.0, "[visual_servo/sdk] 关节状态读取失败");
    return;
  }
  setJointFeedback(position);
  sensor_msgs::JointState message;
  message.header.stamp = ros::Time::now();
  message.name = joint_names_;
  message.position.assign(position.begin(), position.end());
  message.velocity.assign(kDof, 0.0);
  message.effort.assign(kDof, 0.0);
  sdk_joint_pub_.publish(message);
}

VisualServo::ServoState VisualServo::selectState(bool fresh_target,
                                                 double target_age) {
  if (!enabled_)
    return ServoState::DISABLED;
  if (safety_stop_.load())
    return ServoState::HOLD;
  if (fresh_target)
    return aligned_latched_ ? ServoState::ALIGNED : ServoState::TRACKING;
  // 已对齐后的短时遮挡不应触发回退搜索。
  if (aligned_latched_)
    return ServoState::HOLD;
  if (!have_ever_tracked_)
    return initial_search_enabled_ ? ServoState::SEARCH_INITIAL
                                   : ServoState::WAITING;
  const double lost_for = std::max(0.0, target_age - target_timeout_);
  if (loss_strategy_ == "stop")
    return ServoState::HOLD;
  if (lost_for < coast_duration_)
    return ServoState::COAST;
  // 短时漏检先原地保持，不要立刻抬臂回观察位。
  if (lost_for < coast_duration_ + recovery_delay_)
    return ServoState::HOLD;
  if (loss_strategy_ == "coast_then_open" &&
      lost_for < coast_duration_ + recovery_delay_ + search_timeout_)
    return ServoState::SEARCH_RECOVERY;
  return ServoState::HOLD;
}

Eigen::VectorXd
VisualServo::trackingVelocity(const JointPoint &position,
                              const geometry_msgs::Pose &target,
                              Eigen::Vector3d *raw_position_error,
                              Eigen::Vector3d *raw_angular_error) {
  KDL::JntArray joints(kDof);
  for (std::size_t i = 0; i < kDof; ++i)
    joints(i) = position[i];

  KDL::Frame base_control;                  // 计算控制坐标系在基坐标系下的位姿 位置 p   旋转矩阵 M
  KDL::Jacobian kdl_jacobian(kDof);         // 计算控制坐标系在基坐标系下的雅可比矩阵
  if (fk_solver_->JntToCart(joints, base_control) < 0 ||
      jacobian_solver_->JntToJac(joints, kdl_jacobian) < 0)
    return Eigen::VectorXd::Zero(kDof);

  Eigen::Vector3d base_linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d base_angular = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d base_from_control = kdlRotationToEigen(base_control.M);// 转化格式
  if (servo_mode_ == "eye_in_hand") {
    const Eigen::Vector3d observed(target.position.x, target.position.y,
                                   target.position.z);// 眼在手上：目标在夹爪/TCP 坐标系下的位置
    Eigen::Vector3d position_error = observed - desired_position_;
    if (raw_position_error)
      *raw_position_error = position_error;
    // 三轴独立的软死区会屏蔽像素/深度小噪声，并让速度在
    // 死区边界连续衰减到零，避免夹爪在目标两侧来回切换。
    for (int axis = 0; axis < 3; ++axis)
      position_error(axis) =
          applyDeadband(position_error(axis), position_deadband_);
    const Eigen::Vector3d control_linear =
        linear_gain_ * position_error;// 移动夹爪时，目标在夹爪系中反向移动
    base_linear = base_from_control * control_linear;// 转化为基坐标系下的速度
    if (use_orientation_control_) {
      const Eigen::Quaterniond observed_q(
          target.orientation.w, target.orientation.x, target.orientation.y,
          target.orientation.z);
      if (observed_q.norm() > 1e-6) {
        const Eigen::Matrix3d observed_rotation =
            observed_q.normalized().toRotationMatrix();
        const Eigen::Vector3d angular_error =
            rotationLog(desired_rotation_ * observed_rotation.transpose());
        if (raw_angular_error)
          *raw_angular_error = angular_error;
        base_angular =
            base_from_control *
            (-angular_gain_ *     // 相机自己正方向旋转时，目标在相机坐标系里看起来会反方向旋转
             angular_error);// 计算旋转误差 R_d * R_o^T
      }
    }
  } else {
    const Eigen::Vector3d target_in_base(target.position.x, target.position.y,
                                         target.position.z);// 眼在手外：目标位置的基坐标系下的位置
    const Eigen::Vector3d current_tcp(base_control.p.x(), base_control.p.y(),
                                      base_control.p.z());
    Eigen::Vector3d position_error =
        target_in_base + target_offset_ - current_tcp;
    if (raw_position_error)
      *raw_position_error = position_error;
    for (int axis = 0; axis < 3; ++axis)
      position_error(axis) =
          applyDeadband(position_error(axis), position_deadband_);
    base_linear = linear_gain_ * position_error;
    if (use_orientation_control_) {
      const Eigen::Vector3d angular_error =
          rotationLog(desired_rotation_ * base_from_control.transpose());
      if (raw_angular_error)
        *raw_angular_error = angular_error;
      base_angular = angular_gain_ * angular_error;
    }
  }
  if (base_linear.norm() > max_linear_velocity_)
    base_linear *= max_linear_velocity_ / base_linear.norm();
  if (base_angular.norm() > max_angular_velocity_)
    base_angular *= max_angular_velocity_ / base_angular.norm();

  Eigen::VectorXd base_twist(6);
  base_twist.head<3>() = base_linear;
  base_twist.tail<3>() = base_angular;

  Eigen::MatrixXd jacobian(6, kDof);
  for (int row = 0; row < 6; ++row)
    for (int col = 0; col < static_cast<int>(kDof); ++col)
      jacobian(row, col) = kdl_jacobian(row, col);
  return dampedPseudoInverse(jacobian, dls_lambda_) * base_twist;// 雅可比矩阵最小二乘解
}

Eigen::VectorXd VisualServo::desiredVelocity(ServoState state,
                                             const JointPoint &position,
                                             const geometry_msgs::Pose &target,
                                             double target_age) {
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(kDof);
  if (state == ServoState::TRACKING) {      // 正在跟踪中
    velocity = trackingVelocity(position, target, nullptr, nullptr);    // 计算跟踪速度
    last_tracking_velocity_ = velocity;
    have_ever_tracked_ = true;
  } else if (state == ServoState::COAST) {    // 失去目标 继续沿着上一次的速度运动一段时间
    const double lost_for = std::max(0.0, target_age - target_timeout_);
    const double decay =
        std::exp(-lost_for / std::max(0.01, coast_decay_time_));
    velocity = decay * last_tracking_velocity_;
  } else if (state == ServoState::SEARCH_INITIAL ||
             state == ServoState::SEARCH_RECOVERY) {
    const JointPoint &search_posture = state == ServoState::SEARCH_INITIAL
                                           ? initial_search_posture_
                                           : recovery_posture_;
    for (std::size_t i = 0; i < kDof; ++i)
      velocity(i) =
          clampValue(open_posture_gain_ * (search_posture[i] - position[i]),
                     -search_velocity_limit_, search_velocity_limit_);
  }
  return velocity;
}

void VisualServo::transitionTo(ServoState next) {
  if (next == state_)
    return;
  ROS_WARN("[visual_servo] 状态 %s -> %s", stateName(state_), stateName(next));
  state_ = next;
  // 状态切换时清除旧状态生成的命令；加速度限制器仍保证首条新命令连续。
  queue_.clear();
  publishState();
}

void VisualServo::publishState() {
  std_msgs::String message;   // 把当前的状态发布给rviz端
  message.data = stateName(state_);
  state_pub_.publish(message);
  legacy_state_pub_.publish(message);
}

void VisualServo::controlLoop(const ros::TimerEvent &event) {
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  JointPoint feedback{};
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    if (!have_joint_state_ ||
        (ros::Time::now() - last_joint_time_).toSec() > 0.5) {
      ROS_WARN_THROTTLE(1.0, "[visual_servo] 等待有效关节状态");
      return;
    }
    feedback = feedback_position_;// 获取当前关节位置 
  }
  if (!integrator_initialized_) {   // 初始化速度到位置积分器
    command_position_ = feedback;
    command_velocity_.fill(0.0);
    backend_velocity_.fill(0.0);
    last_output_ = feedback;
    integrator_initialized_ = true;
  }

  geometry_msgs::Pose target;
  bool fresh_target = false;
  double target_age = 1e9;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (have_target_) {
      target = target_pose_;
      target_age = (ros::Time::now() - last_target_time_).toSec();// 距离上次获取到目标之间的时间间隔
      fresh_target = target_age <= target_timeout_;// 如果目标年龄小于超时时间，认为目标新鲜
    }
  }

  ServoState next = selectState(fresh_target, target_age);// 选择下一个状态
  bool entering_reacquire_hold = false;
  if (next == ServoState::SEARCH_INITIAL ||
      next == ServoState::SEARCH_RECOVERY) {    // 如果机械臂要进入观察位置
    if (!search_reacquire_pending_)     // 如果不是重捕获阶段，说明是初始搜索阶段
      reacquire_candidate_ = false;     // 还未进入重捕获阶段，重捕获候选标志置为false
    search_reacquire_pending_ = true;
  }

  // 搜索运动中看到一帧目标时先刹停。只有目标连续稳定存在一段时间，
  // 才退出搜索并恢复闭环，避免“看到就下、丢失就上”的状态振荡。
  if (fresh_target && search_reacquire_pending_) {
    const ros::Time now = ros::Time::now();
    if (!reacquire_candidate_) {    // 在重捕获阶段 发现了新的目标 捕获候选标志置为true
      reacquire_candidate_ = true;
      reacquire_candidate_since_ = now; // 记录重捕获阶段 发现目标的时间
    }
    if ((now - reacquire_candidate_since_).toSec() < reacquire_hold_time_) {
      next = ServoState::HOLD;
      entering_reacquire_hold = state_ != ServoState::HOLD; // 一定时间段内先保持静止
    } else {
      search_reacquire_pending_ = false;// 关闭重捕获阶段，进入跟踪阶段
      reacquire_candidate_ = false;
      next = aligned_latched_ ? ServoState::ALIGNED : ServoState::TRACKING;
    }
  } else if (!fresh_target) {
    reacquire_candidate_ = false;
  }

  Eigen::VectorXd requested = Eigen::VectorXd::Zero(kDof);
  if (fresh_target &&
      (next == ServoState::TRACKING || next == ServoState::ALIGNED)) {
    Eigen::Vector3d raw_position_error =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d raw_angular_error =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    const Eigen::VectorXd tracking_request = trackingVelocity(
        feedback, target, &raw_position_error, &raw_angular_error);// 计算速度和误差
    const double release_scale =
        aligned_latched_ ? alignment_release_multiplier_ : 1.0;// 引入迟滞因子 对于对齐状态 这个位置误差的阈值会放大
    const bool position_aligned =
        raw_position_error.allFinite() &&
        (raw_position_error.array().abs() <=
         position_deadband_ * release_scale).all(); // 判断位置误差是否在死区阈值内
    const bool orientation_aligned =
        !use_orientation_control_ ||
        (raw_angular_error.allFinite() &&
         raw_angular_error.norm() <= orientation_deadband_ * release_scale); // 判断角度误差是否在死区阈值内
    const bool inside_alignment_window =
        position_aligned && orientation_aligned;
    const ros::Time now = ros::Time::now();

    if (inside_alignment_window) {
      if (aligned_latched_) {
        next = ServoState::ALIGNED;
      } else {
        if (!alignment_candidate_) {
          alignment_candidate_ = true;
          alignment_candidate_since_ = now;
        }
        if ((now - alignment_candidate_since_).toSec() >=
            alignment_hold_time_) {
          aligned_latched_ = true;
          next = ServoState::ALIGNED;
        } else {
          next = ServoState::TRACKING;
        }
      }
    } else {
      alignment_candidate_ = false;
      aligned_latched_ = false;
      next = ServoState::TRACKING;
    }
    // 上述程序 就是判断是否是在 TRACKING状态下对齐还是ALIGNED状态下对齐 设置不同的迟滞因子
    // 然后根据是否对齐 判断是否需要保持对齐状态 还是继续跟踪

    if (next == ServoState::TRACKING)
      requested = tracking_request;
    last_tracking_velocity_ = requested;
    have_ever_tracked_ = true;
  } else {
    alignment_candidate_ = false;
    requested = desiredVelocity(next, feedback, target, target_age);
  }

  const bool entering_aligned =
      next == ServoState::ALIGNED && state_ != ServoState::ALIGNED;
  transitionTo(next);
  if (entering_aligned || entering_reacquire_hold) {
    // 到达后立即以实际反馈位置作为保持点，清除积分和
    // Gazebo 输出端的残留速度；重捕获确认阶段也立即停止搜索运动。
    queue_.clear();
    command_position_ = feedback;
    command_velocity_.fill(0.0);
    backend_velocity_.fill(0.0);
    last_output_ = feedback;
  }
  if (!requested.allFinite())
    requested.setZero();

  // 将开环积分位置缓慢拉回反馈位置，避免引入突跳；两种后端共用此状态。
  for (std::size_t i = 0; i < kDof; ++i)
    command_position_[i] +=
        feedback_blend_ * (feedback[i] - command_position_[i]);

  const double callback_dt =
      clampValue((event.current_real - event.last_real).toSec(),
                 0.5 / control_rate_, 2.0 / control_rate_);// 测量真实的控制循环时间
  const int substeps =
      std::max(1, static_cast<int>(std::round(callback_dt * output_rate_)));// 计算控制周期内需要产生几个输出点
  const double dt = callback_dt / substeps;// 每个点之间的时间间隔
  for (int step = 0; step < substeps; ++step) {
    for (std::size_t i = 0; i < kDof; ++i) {
      const double wanted =
          clampValue(requested(i), -velocity_limits_[i], velocity_limits_[i]);// 速度限位
      command_velocity_[i] = clampValue(
          wanted, command_velocity_[i] - acceleration_limits_[i] * dt,
          command_velocity_[i] + acceleration_limits_[i] * dt);// 加速度限位

      const double low = lower_limits_[i] + joint_limit_margin_;
      const double high = upper_limits_[i] - joint_limit_margin_;
      const double integrated =
          command_position_[i] + command_velocity_[i] * dt;// 前向积分计算下一时刻位置
      command_position_[i] = clampValue(integrated, low, high);// 位置限位
      if (command_position_[i] == low || command_position_[i] == high)
        command_velocity_[i] = 0.0;
    }
    queue_.push(command_position_);
  }
}

void VisualServo::gazeboOutput(const ros::TimerEvent &) {
  JointPoint requested{};
  if (queue_.pop(requested)) {
    // 在消费端再次限速限加速度，即使队列丢弃过期点也能保持指令连续。
    const double dt = 1.0 / output_rate_;
    for (std::size_t i = 0; i < kDof; ++i) {
      const double requested_velocity =
          clampValue((requested[i] - last_output_[i]) / dt,
                     -velocity_limits_[i], velocity_limits_[i]);
      backend_velocity_[i] =
          clampValue(requested_velocity,
                     backend_velocity_[i] - acceleration_limits_[i] * dt,
                     backend_velocity_[i] + acceleration_limits_[i] * dt);
      last_output_[i] = clampValue(last_output_[i] + backend_velocity_[i] * dt,
                                   lower_limits_[i] + joint_limit_margin_,
                                   upper_limits_[i] - joint_limit_margin_);
    }
  }
  if (!integrator_initialized_)
    return;
  for (std::size_t i = 0; i < kDof; ++i) {
    std_msgs::Float64 message;
    message.data = last_output_[i];
    gazebo_publishers_[i].publish(message);
  }
}

} // namespace aubo_ros_control

int main(int argc, char **argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "visual_servo");
  aubo_ros_control::VisualServo servo;
  if (!servo.valid())
    return 1;
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::Rate monitor(10.0);
  while (ros::ok() && servo.healthy())
    monitor.sleep();
  if (ros::ok())
    ROS_ERROR("[visual_servo] 后端失效，节点退出");
  ros::shutdown();
  return 0;
}
