/**
 * Eye-in-hand PBVS for AUBO i5.
 *
 * One controller core is shared by two deliberately separate backends:
 *   gazebo: bounded queue -> six JointPositionController command topics
 *   sdk:    bounded queue -> validation/optimization -> AUBO TCP2CANBUS SDK
 *
 * Input target poses are interpreted in the moving camera optical frame.  A
 * TF transform is applied when the detector publishes in another frame.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
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

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <clocale>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
constexpr std::size_t kDof = 6;
using JointPoint = std::array<double, kDof>;

double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

bool finitePoint(const JointPoint& point)
{
  return std::all_of(point.begin(), point.end(),
                     [](double value) { return std::isfinite(value); });
}

class CommandQueue
{
public:
  explicit CommandQueue(std::size_t capacity) : capacity_(std::max<std::size_t>(2, capacity)) {}

  void push(const JointPoint& point)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Bound latency.  The oldest unexecuted point is less useful than the
    // newest visual correction when the producer briefly outruns the backend.
    if (queue_.size() >= capacity_)
      queue_.pop_front();
    queue_.push_back(point);
    condition_.notify_one();
  }

  bool pop(JointPoint& point)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty())
      return false;
    point = queue_.front();
    queue_.pop_front();
    return true;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

private:
  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<JointPoint> queue_;
};

Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& jacobian, double lambda)
{
  // J^T (J J^T + lambda^2 I)^-1 is stable near singular configurations.
  const Eigen::MatrixXd regularized =
      jacobian * jacobian.transpose() +
      lambda * lambda * Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows());
  return jacobian.transpose() * regularized.ldlt().solve(
      Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows()));
}

Eigen::Matrix3d kdlRotationToEigen(const KDL::Rotation& rotation)
{
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result(row, col) = rotation(row, col);
  return result;
}

Eigen::Vector3d rotationLog(const Eigen::Matrix3d& rotation)
{
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || !angle_axis.axis().allFinite())
    return Eigen::Vector3d::Zero();
  return angle_axis.axis() * angle_axis.angle();
}

template <typename T>
bool getSixValues(ros::NodeHandle& nh, const std::string& name, std::array<T, kDof>& values)
{
  std::vector<double> parameter;
  if (!nh.getParam(name, parameter) || parameter.size() != kDof)
  {
    ROS_ERROR("[visual_servo] 参数 ~%s 必须包含 6 个数", name.c_str());
    return false;
  }
  for (std::size_t i = 0; i < kDof; ++i)
    values[i] = static_cast<T>(parameter[i]);
  return true;
}

class DirectSdkBackend
{
public:
  DirectSdkBackend(CommandQueue& queue,
                   const JointPoint& velocity_limits,
                   const JointPoint& acceleration_limits,
                   double output_rate)
      : queue_(queue), velocity_limits_(velocity_limits),
        acceleration_limits_(acceleration_limits), period_(1.0 / output_rate)
  {
    last_position_.fill(0.0);
    last_velocity_.fill(0.0);
  }

  ~DirectSdkBackend() { shutdown(); }

  bool connect(ros::NodeHandle& nh)
  {
    nh.param<std::string>("robot_ip", host_, "192.168.1.2");
    nh.param<int>("server_port", port_, 8899);
    nh.param<std::string>("username", username_, "aubo");
    nh.param<std::string>("password", password_, "123456");
    nh.param<int>("collision_class", collision_class_, 6);
    nh.param<bool>("require_real_robot", require_real_robot_, true);
    nh.param<int>("sdk_mac_buffer_target", mac_buffer_target_, 60);

    const std::string sdk_path = ros::package::getPath("aubo_sdk");
    if (!sdk_path.empty() && ::chdir(sdk_path.c_str()) != 0)
      ROS_WARN("[visual_servo/sdk] 无法切换到 aubo_sdk 目录，继续使用当前目录");

    if (!login(control_service_) || !login(state_service_) || !login(mac_service_))
    {
      ROS_ERROR("[visual_servo/sdk] SDK 三连接登录失败");
      shutdown();
      return false;
    }

    bool real_robot = false;
    state_service_.robotServiceGetIsRealRobotExist(real_robot);
    if (require_real_robot_ && !real_robot)
    {
      ROS_ERROR("[visual_servo/sdk] 控制柜未报告真实机械臂，拒绝下发运动");
      shutdown();
      return false;
    }

    aubo_robot_namespace::ToolDynamicsParam tool{};
    aubo_robot_namespace::ROBOT_SERVICE_STATE startup_state;
    int result = control_service_.rootServiceRobotStartup(
        tool, static_cast<uint8>(collision_class_),
        true, true, 1000, startup_state);
    if (result != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      ROS_ERROR("[visual_servo/sdk] 机器人启动失败，错误码 %d", result);
      shutdown();
      return false;
    }
    result = control_service_.robotServiceRobotHandShake(true);
    if (result != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      ROS_ERROR("[visual_servo/sdk] 握手失败，错误码 %d", result);
      shutdown();
      return false;
    }

    bool ready = false;
    for (int attempt = 0; attempt < 100; ++attempt)
    {
      aubo_robot_namespace::RobotDiagnosis diagnosis{};
      if (state_service_.robotServiceGetRobotDiagnosisInfo(diagnosis) ==
              aubo_robot_namespace::InterfaceCallSuccCode &&
          diagnosis.armPowerStatus && !diagnosis.softEmergency &&
          !diagnosis.remoteEmergency && !diagnosis.robotCollision)
      {
        ready = true;
        break;
      }
      usleep(100000);
    }
    if (!ready)
    {
      ROS_ERROR("[visual_servo/sdk] 机器人在 10 秒内未进入安全就绪状态");
      shutdown();
      return false;
    }

    aubo_robot_namespace::wayPoint_S waypoint{};
    if (state_service_.robotServiceGetCurrentWaypointInfo(waypoint) !=
        aubo_robot_namespace::InterfaceCallSuccCode)
    {
      ROS_ERROR("[visual_servo/sdk] 无法读取初始关节位置");
      shutdown();
      return false;
    }
    for (std::size_t i = 0; i < kDof; ++i)
      last_position_[i] = waypoint.jointpos[i];
    have_last_position_ = true;

    // Pre-fill a stationary lead-in.  This gives the controller enough points
    // to absorb ordinary ROS scheduling jitter before live servo points arrive.
    const int lead_in_points = std::max(4, mac_buffer_target_ / 6);
    for (int i = 0; i < lead_in_points; ++i)
      queue_.push(last_position_);

    result = control_service_.robotServiceEnterTcp2CanbusMode();
    if (result == aubo_robot_namespace::ErrCode_ResponseReturnError)
    {
      control_service_.robotServiceLeaveTcp2CanbusMode();
      usleep(200000);
      result = control_service_.robotServiceEnterTcp2CanbusMode();
    }
    if (result != aubo_robot_namespace::InterfaceCallSuccCode)
    {
      ROS_ERROR("[visual_servo/sdk] 无法进入 TCP2CANBUS 模式，错误码 %d", result);
      shutdown();
      return false;
    }

    in_stream_mode_ = true;
    running_.store(true);
    output_thread_ = std::thread(&DirectSdkBackend::outputLoop, this);
    ROS_INFO("[visual_servo/sdk] 已连接 %s:%d，启动直接 SDK 队列下发", host_.c_str(), port_);
    return true;
  }

  bool readState(JointPoint& position)
  {
    if (!connected_.load())
      return false;
    aubo_robot_namespace::wayPoint_S waypoint{};
    if (state_service_.robotServiceGetCurrentWaypointInfo(waypoint) !=
        aubo_robot_namespace::InterfaceCallSuccCode)
      return false;
    for (std::size_t i = 0; i < kDof; ++i)
      position[i] = waypoint.jointpos[i];
    return finitePoint(position);
  }

  bool healthy() const { return connected_.load() && running_.load(); }

  void shutdown()
  {
    if (shutdown_started_.exchange(true))
      return;
    running_.store(false);
    if (output_thread_.joinable())
      output_thread_.join();
    if (in_stream_mode_)
      control_service_.robotServiceLeaveTcp2CanbusMode();
    control_service_.robotServiceLogout();
    state_service_.robotServiceLogout();
    mac_service_.robotServiceLogout();
    connected_.store(false);
    in_stream_mode_ = false;
  }

private:
  bool login(ServiceInterface& service)
  {
    for (int attempt = 1; attempt <= 5; ++attempt)
    {
      if (service.robotServiceLogin(host_.c_str(), port_, username_.c_str(), password_.c_str()) ==
          aubo_robot_namespace::InterfaceCallSuccCode)
      {
        connected_.store(true);
        return true;
      }
      ROS_WARN("[visual_servo/sdk] 第 %d 次登录失败", attempt);
      usleep(500000);
    }
    return false;
  }

  JointPoint optimize(const JointPoint& requested)
  {
    if (!have_last_position_)
      return requested;
    JointPoint optimized = last_position_;
    for (std::size_t i = 0; i < kDof; ++i)
    {
      const double requested_velocity =
          clampValue((requested[i] - last_position_[i]) / period_,
                     -velocity_limits_[i], velocity_limits_[i]);
      const double velocity =
          clampValue(requested_velocity,
                     last_velocity_[i] - acceleration_limits_[i] * period_,
                     last_velocity_[i] + acceleration_limits_[i] * period_);
      optimized[i] = last_position_[i] + velocity * period_;
      last_velocity_[i] = velocity;
    }
    last_position_ = optimized;
    return optimized;
  }

  void outputLoop()
  {
    while (running_.load() && ros::ok())
    {
      aubo_robot_namespace::RobotDiagnosis diagnosis{};
      const int result = mac_service_.robotServiceGetRobotDiagnosisInfo(diagnosis);
      if (result != aubo_robot_namespace::InterfaceCallSuccCode ||
          diagnosis.softEmergency || diagnosis.remoteEmergency || diagnosis.robotCollision)
      {
        ROS_ERROR("[visual_servo/sdk] 诊断失败或安全状态触发，停止 SDK 下发");
        running_.store(false);
        connected_.store(false);
        break;
      }

      if (diagnosis.macTargetPosDataSize < mac_buffer_target_)
      {
        const int wanted = std::max(1, static_cast<int>(std::ceil(
            (mac_buffer_target_ - diagnosis.macTargetPosDataSize) / 6.0)));
        std::vector<aubo_robot_namespace::wayPoint_S> batch;
        batch.reserve(static_cast<std::size_t>(wanted));
        for (int index = 0; index < wanted; ++index)
        {
          JointPoint requested{};
          if (!queue_.pop(requested))
            break;
          if (!finitePoint(requested))
            continue;
          const JointPoint safe = optimize(requested);
          aubo_robot_namespace::wayPoint_S waypoint{};
          std::copy(safe.begin(), safe.end(), waypoint.jointpos);
          batch.push_back(waypoint);
        }
        if (!batch.empty() &&
            mac_service_.robotServiceSetRobotPosData2Canbus(batch) !=
                aubo_robot_namespace::InterfaceCallSuccCode)
        {
          ROS_ERROR("[visual_servo/sdk] 控制柜轨迹点下发失败");
          running_.store(false);
          connected_.store(false);
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
  }

  CommandQueue& queue_;
  JointPoint velocity_limits_;
  JointPoint acceleration_limits_;
  double period_;
  std::string host_, username_, password_;
  int port_{8899}, collision_class_{6}, mac_buffer_target_{60};
  bool require_real_robot_{true};
  ServiceInterface control_service_, state_service_, mac_service_;
  std::atomic<bool> connected_{false}, running_{false}, shutdown_started_{false};
  bool in_stream_mode_{false}, have_last_position_{false};
  std::thread output_thread_;
  JointPoint last_position_, last_velocity_;
};

enum class ServoState { WAITING, TRACKING, COAST, SEARCH_OPEN, HOLD };

const char* stateName(ServoState state)
{
  switch (state)
  {
    case ServoState::WAITING: return "WAITING";
    case ServoState::TRACKING: return "TRACKING";
    case ServoState::COAST: return "COAST";
    case ServoState::SEARCH_OPEN: return "SEARCH_OPEN";
    case ServoState::HOLD: return "HOLD";
  }
  return "UNKNOWN";
}

class EyeInHandVisualServo
{
public:
  EyeInHandVisualServo()
      : nh_(), private_nh_("~"), tf_listener_(tf_buffer_), queue_(readQueueCapacity())
  {
    valid_ = loadParameters() && initializeKinematics();
    if (!valid_)
      return;

    state_pub_ = private_nh_.advertise<std_msgs::String>("state", 1, true);
    target_sub_ = nh_.subscribe(target_topic_, 1, &EyeInHandVisualServo::targetCallback, this);

    if (backend_ == "gazebo")
    {
      joint_sub_ = nh_.subscribe(joint_states_topic_, 2,
                                 &EyeInHandVisualServo::jointStateCallback, this);
      if (!setupGazeboPublishers())
      {
        valid_ = false;
        return;
      }
      output_timer_ = nh_.createTimer(ros::Duration(1.0 / output_rate_),
                                      &EyeInHandVisualServo::gazeboOutput, this);
    }
    else if (backend_ == "sdk")
    {
      sdk_.reset(new DirectSdkBackend(queue_, velocity_limits_, acceleration_limits_, output_rate_));
      if (!sdk_->connect(private_nh_))
      {
        valid_ = false;
        return;
      }
      sdk_joint_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_states_topic_, 2);
      sdk_state_timer_ = nh_.createTimer(ros::Duration(0.02),
                                         &EyeInHandVisualServo::sdkStateUpdate, this);
      // Prime state synchronously so integration never starts from zero.
      JointPoint initial{};
      if (sdk_->readState(initial))
        setJointFeedback(initial);
    }
    else
    {
      ROS_ERROR("[visual_servo] backend 只能是 gazebo 或 sdk，当前为 '%s'", backend_.c_str());
      valid_ = false;
      return;
    }

    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                     &EyeInHandVisualServo::controlLoop, this);
    publishState();
    ROS_INFO("[visual_servo] 初始化完成：backend=%s, eye-in-hand chain=%s -> %s",
             backend_.c_str(), base_link_.c_str(), camera_link_.c_str());
  }

  bool valid() const { return valid_; }
  bool healthy() const { return backend_ != "sdk" || (sdk_ && sdk_->healthy()); }

private:
  std::size_t readQueueCapacity()
  {
    int capacity = 80;
    private_nh_.param<int>("command_queue_capacity", capacity, 80);
    return static_cast<std::size_t>(std::max(2, capacity));
  }

  bool loadParameters()
  {
    private_nh_.param<std::string>("backend", backend_, "gazebo");
    private_nh_.param<std::string>("base_link", base_link_, "base_link");
    private_nh_.param<std::string>("camera_link", camera_link_, "hand_camera_optical_frame");
    private_nh_.param<std::string>("target_topic", target_topic_, "/visual_servo/target_pose");
    private_nh_.param<std::string>("joint_states_topic", joint_states_topic_, "/joint_states");
    private_nh_.param<std::string>("loss_strategy", loss_strategy_, "coast_then_open");
    private_nh_.param<double>("control_rate", control_rate_, 100.0);
    private_nh_.param<double>("output_rate", output_rate_, 200.0);
    private_nh_.param<double>("linear_gain", linear_gain_, 0.8);
    private_nh_.param<double>("angular_gain", angular_gain_, 0.5);
    private_nh_.param<double>("max_camera_linear_velocity", max_linear_velocity_, 0.08);
    private_nh_.param<double>("max_camera_angular_velocity", max_angular_velocity_, 0.2);
    private_nh_.param<double>("position_deadband", position_deadband_, 0.004);
    private_nh_.param<double>("dls_lambda", dls_lambda_, 0.04);
    private_nh_.param<double>("joint_limit_margin", joint_limit_margin_, 0.08);
    private_nh_.param<double>("target_timeout", target_timeout_, 0.2);
    private_nh_.param<double>("coast_duration", coast_duration_, 0.35);
    private_nh_.param<double>("coast_decay_time", coast_decay_time_, 0.18);
    private_nh_.param<double>("search_timeout", search_timeout_, 8.0);
    private_nh_.param<double>("open_posture_gain", open_posture_gain_, 0.7);
    private_nh_.param<double>("search_velocity_limit", search_velocity_limit_, 0.2);
    private_nh_.param<double>("feedback_blend", feedback_blend_, 0.02);
    private_nh_.param<bool>("use_orientation_control", use_orientation_control_, false);

    if (control_rate_ <= 0.0 || output_rate_ < control_rate_ || output_rate_ > 500.0)
    {
      ROS_ERROR("[visual_servo] 频率要求 0 < control_rate <= output_rate <= 500");
      return false;
    }
    if (loss_strategy_ != "stop" && loss_strategy_ != "coast" &&
        loss_strategy_ != "coast_then_open")
    {
      ROS_ERROR("[visual_servo] loss_strategy 必须是 stop、coast 或 coast_then_open");
      return false;
    }

    if (!private_nh_.getParam("joint_names", joint_names_) || joint_names_.size() != kDof)
    {
      ROS_ERROR("[visual_servo] ~joint_names 必须包含 6 个关节名");
      return false;
    }
    if (!getSixValues(private_nh_, "joint_lower_limits", lower_limits_) ||
        !getSixValues(private_nh_, "joint_upper_limits", upper_limits_) ||
        !getSixValues(private_nh_, "joint_velocity_limits", velocity_limits_) ||
        !getSixValues(private_nh_, "joint_acceleration_limits", acceleration_limits_) ||
        !getSixValues(private_nh_, "open_posture", open_posture_))
      return false;

    std::vector<double> desired_position, desired_rpy;
    if (!private_nh_.getParam("desired_target_position", desired_position) ||
        desired_position.size() != 3 ||
        !private_nh_.getParam("desired_target_rpy", desired_rpy) || desired_rpy.size() != 3)
    {
      ROS_ERROR("[visual_servo] desired_target_position/desired_target_rpy 必须各含 3 个数");
      return false;
    }
    desired_position_ = Eigen::Vector3d(desired_position[0], desired_position[1], desired_position[2]);
    desired_rotation_ =
        (Eigen::AngleAxisd(desired_rpy[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(desired_rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(desired_rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    return true;
  }

  bool initializeKinematics()
  {
    std::string robot_description;
    if (!nh_.getParam("robot_description", robot_description))
    {
      ROS_ERROR("[visual_servo] 缺少 /robot_description");
      return false;
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_description, tree) ||
        !tree.getChain(base_link_, camera_link_, chain_))
    {
      ROS_ERROR("[visual_servo] 无法建立 KDL 链 %s -> %s",
                base_link_.c_str(), camera_link_.c_str());
      return false;
    }
    if (chain_.getNrOfJoints() != kDof)
    {
      ROS_ERROR("[visual_servo] 相机链应有 6 个活动关节，实际为 %u", chain_.getNrOfJoints());
      return false;
    }
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    jacobian_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
    return true;
  }

  bool setupGazeboPublishers()
  {
    if (!private_nh_.getParam("gazebo_command_topics", gazebo_topics_) ||
        gazebo_topics_.size() != kDof)
    {
      ROS_ERROR("[visual_servo] Gazebo 后端需要 6 个 ~gazebo_command_topics");
      return false;
    }
    for (const std::string& topic : gazebo_topics_)
      gazebo_publishers_.push_back(nh_.advertise<std_msgs::Float64>(topic, 1));
    return true;
  }

  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    geometry_msgs::PoseStamped camera_target;
    try
    {
      if (message->header.frame_id.empty() || message->header.frame_id == camera_link_)
      {
        camera_target = *message;
        camera_target.header.frame_id = camera_link_;
      }
      else
      {
        camera_target = tf_buffer_.transform(*message, camera_link_, ros::Duration(0.03));
      }
    }
    catch (const tf2::TransformException& exception)
    {
      ROS_WARN_THROTTLE(1.0, "[visual_servo] 目标无法变换到相机系: %s", exception.what());
      return;
    }
    if (!std::isfinite(camera_target.pose.position.x) ||
        !std::isfinite(camera_target.pose.position.y) ||
        !std::isfinite(camera_target.pose.position.z) || camera_target.pose.position.z <= 0.0)
    {
      ROS_WARN_THROTTLE(1.0, "[visual_servo] 忽略无效或位于相机后方的目标");
      return;
    }
    const ros::Time now = ros::Time::now();
    ros::Time measurement_time = message->header.stamp.isZero() ? now : message->header.stamp;
    if (measurement_time > now)
      measurement_time = now;
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_pose_ = camera_target.pose;
    last_target_time_ = measurement_time;
    have_target_ = true;
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& message)
  {
    JointPoint ordered{};
    for (std::size_t i = 0; i < kDof; ++i)
    {
      const auto found = std::find(message->name.begin(), message->name.end(), joint_names_[i]);
      if (found == message->name.end())
        return;
      const std::size_t index = static_cast<std::size_t>(std::distance(message->name.begin(), found));
      if (index >= message->position.size())
        return;
      ordered[i] = message->position[index];
    }
    if (finitePoint(ordered))
      setJointFeedback(ordered);
  }

  void setJointFeedback(const JointPoint& position)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    feedback_position_ = position;
    have_joint_state_ = true;
    last_joint_time_ = ros::Time::now();
  }

  void sdkStateUpdate(const ros::TimerEvent&)
  {
    JointPoint position{};
    if (!sdk_ || !sdk_->readState(position))
    {
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

  ServoState selectState(bool fresh_target, double target_age)
  {
    if (fresh_target)
      return ServoState::TRACKING;
    if (!have_ever_tracked_)
      return ServoState::WAITING;
    const double lost_for = std::max(0.0, target_age - target_timeout_);
    if (loss_strategy_ == "stop")
      return ServoState::HOLD;
    if (lost_for < coast_duration_)
      return ServoState::COAST;
    if (loss_strategy_ == "coast_then_open" && lost_for < coast_duration_ + search_timeout_)
      return ServoState::SEARCH_OPEN;
    return ServoState::HOLD;
  }

  Eigen::VectorXd trackingVelocity(const JointPoint& position,
                                   const geometry_msgs::Pose& target)
  {
    KDL::JntArray joints(kDof);
    for (std::size_t i = 0; i < kDof; ++i)
      joints(i) = position[i];

    KDL::Frame base_camera;
    KDL::Jacobian kdl_jacobian(kDof);
    if (fk_solver_->JntToCart(joints, base_camera) < 0 ||
        jacobian_solver_->JntToJac(joints, kdl_jacobian) < 0)
      return Eigen::VectorXd::Zero(kDof);

    Eigen::Vector3d current(target.position.x, target.position.y, target.position.z);
    Eigen::Vector3d camera_linear = linear_gain_ * (current - desired_position_);
    if ((current - desired_position_).norm() < position_deadband_)
      camera_linear.setZero();
    if (camera_linear.norm() > max_linear_velocity_)
      camera_linear *= max_linear_velocity_ / camera_linear.norm();

    Eigen::Vector3d camera_angular = Eigen::Vector3d::Zero();
    if (use_orientation_control_)
    {
      const Eigen::Quaterniond observed_q(target.orientation.w, target.orientation.x,
                                          target.orientation.y, target.orientation.z);
      if (observed_q.norm() > 1e-6)
      {
        const Eigen::Matrix3d observed = observed_q.normalized().toRotationMatrix();
        // Eye-in-hand sign: rotating the camera produces the opposite rotation
        // in the observed object pose.
        camera_angular = -angular_gain_ * rotationLog(desired_rotation_ * observed.transpose());
        if (camera_angular.norm() > max_angular_velocity_)
          camera_angular *= max_angular_velocity_ / camera_angular.norm();
      }
    }

    const Eigen::Matrix3d base_from_camera = kdlRotationToEigen(base_camera.M);
    Eigen::VectorXd base_twist(6);
    base_twist.head<3>() = base_from_camera * camera_linear;
    base_twist.tail<3>() = base_from_camera * camera_angular;

    Eigen::MatrixXd jacobian(6, kDof);
    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < static_cast<int>(kDof); ++col)
        jacobian(row, col) = kdl_jacobian(row, col);
    return dampedPseudoInverse(jacobian, dls_lambda_) * base_twist;
  }

  Eigen::VectorXd desiredVelocity(ServoState state, const JointPoint& position,
                                  const geometry_msgs::Pose& target, double target_age)
  {
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero(kDof);
    if (state == ServoState::TRACKING)
    {
      velocity = trackingVelocity(position, target);
      last_tracking_velocity_ = velocity;
      have_ever_tracked_ = true;
    }
    else if (state == ServoState::COAST)
    {
      const double lost_for = std::max(0.0, target_age - target_timeout_);
      const double decay = std::exp(-lost_for / std::max(0.01, coast_decay_time_));
      velocity = decay * last_tracking_velocity_;
    }
    else if (state == ServoState::SEARCH_OPEN)
    {
      for (std::size_t i = 0; i < kDof; ++i)
        velocity(i) = clampValue(open_posture_gain_ * (open_posture_[i] - position[i]),
                                 -search_velocity_limit_, search_velocity_limit_);
    }
    return velocity;
  }

  void transitionTo(ServoState next)
  {
    if (next == state_)
      return;
    ROS_WARN("[visual_servo] 状态 %s -> %s", stateName(state_), stateName(next));
    state_ = next;
    // Remove commands computed for the previous visual state. The acceleration
    // limiter still makes the first command in the new state continuous.
    queue_.clear();
    publishState();
  }

  void publishState()
  {
    std_msgs::String message;
    message.data = stateName(state_);
    state_pub_.publish(message);
  }

  void controlLoop(const ros::TimerEvent& event)
  {
    std::lock_guard<std::mutex> control_lock(control_mutex_);
    JointPoint feedback{};
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      if (!have_joint_state_ || (ros::Time::now() - last_joint_time_).toSec() > 0.5)
      {
        ROS_WARN_THROTTLE(1.0, "[visual_servo] 等待有效关节状态");
        return;
      }
      feedback = feedback_position_;
    }
    if (!integrator_initialized_)
    {
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
      if (have_target_)
      {
        target = target_pose_;
        target_age = (ros::Time::now() - last_target_time_).toSec();
        fresh_target = target_age <= target_timeout_;
      }
    }

    const ServoState next = selectState(fresh_target, target_age);
    transitionTo(next);
    Eigen::VectorXd requested = desiredVelocity(state_, feedback, target, target_age);
    if (!requested.allFinite())
      requested.setZero();

    // Gently anchor the open-loop integral to measured state without injecting
    // a discontinuity. The same state is used in both backends.
    for (std::size_t i = 0; i < kDof; ++i)
      command_position_[i] += feedback_blend_ * (feedback[i] - command_position_[i]);

    const double callback_dt = clampValue((event.current_real - event.last_real).toSec(),
                                          0.5 / control_rate_, 2.0 / control_rate_);
    const int substeps = std::max(1, static_cast<int>(std::round(callback_dt * output_rate_)));
    const double dt = callback_dt / substeps;
    for (int step = 0; step < substeps; ++step)
    {
      for (std::size_t i = 0; i < kDof; ++i)
      {
        const double wanted = clampValue(requested(i), -velocity_limits_[i], velocity_limits_[i]);
        command_velocity_[i] = clampValue(
            wanted,
            command_velocity_[i] - acceleration_limits_[i] * dt,
            command_velocity_[i] + acceleration_limits_[i] * dt);

        const double low = lower_limits_[i] + joint_limit_margin_;
        const double high = upper_limits_[i] - joint_limit_margin_;
        const double integrated = command_position_[i] + command_velocity_[i] * dt;
        command_position_[i] = clampValue(integrated, low, high);
        if (command_position_[i] == low || command_position_[i] == high)
          command_velocity_[i] = 0.0;
      }
      queue_.push(command_position_);
    }
  }

  void gazeboOutput(const ros::TimerEvent&)
  {
    JointPoint requested{};
    if (queue_.pop(requested))
    {
      // Revalidate at the consumer boundary too. This keeps the command
      // continuous even if a full bounded queue has discarded stale points.
      const double dt = 1.0 / output_rate_;
      for (std::size_t i = 0; i < kDof; ++i)
      {
        const double requested_velocity = clampValue(
            (requested[i] - last_output_[i]) / dt,
            -velocity_limits_[i], velocity_limits_[i]);
        backend_velocity_[i] = clampValue(
            requested_velocity,
            backend_velocity_[i] - acceleration_limits_[i] * dt,
            backend_velocity_[i] + acceleration_limits_[i] * dt);
        last_output_[i] = clampValue(
            last_output_[i] + backend_velocity_[i] * dt,
            lower_limits_[i] + joint_limit_margin_,
            upper_limits_[i] - joint_limit_margin_);
      }
    }
    if (!integrator_initialized_)
      return;
    for (std::size_t i = 0; i < kDof; ++i)
    {
      std_msgs::Float64 message;
      message.data = last_output_[i];
      gazebo_publishers_[i].publish(message);
    }
  }

  ros::NodeHandle nh_, private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  CommandQueue queue_;
  bool valid_{false};

  std::string backend_, base_link_, camera_link_, target_topic_, joint_states_topic_, loss_strategy_;
  std::vector<std::string> joint_names_, gazebo_topics_;
  double control_rate_{100.0}, output_rate_{200.0};
  double linear_gain_{0.8}, angular_gain_{0.5};
  double max_linear_velocity_{0.08}, max_angular_velocity_{0.2};
  double position_deadband_{0.004}, dls_lambda_{0.04}, joint_limit_margin_{0.08};
  double target_timeout_{0.2}, coast_duration_{0.35}, coast_decay_time_{0.18};
  double search_timeout_{8.0}, open_posture_gain_{0.7}, search_velocity_limit_{0.2};
  double feedback_blend_{0.02};
  bool use_orientation_control_{false};
  JointPoint lower_limits_{}, upper_limits_{}, velocity_limits_{}, acceleration_limits_{};
  JointPoint open_posture_{}, feedback_position_{}, command_position_{}, command_velocity_{};
  JointPoint backend_velocity_{}, last_output_{};
  Eigen::Vector3d desired_position_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d desired_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::VectorXd last_tracking_velocity_{Eigen::VectorXd::Zero(kDof)};

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::unique_ptr<DirectSdkBackend> sdk_;

  std::mutex target_mutex_, joint_mutex_, control_mutex_;
  geometry_msgs::Pose target_pose_;
  ros::Time last_target_time_, last_joint_time_;
  bool have_target_{false}, have_joint_state_{false}, integrator_initialized_{false};
  bool have_ever_tracked_{false};
  ServoState state_{ServoState::WAITING};

  ros::Subscriber target_sub_, joint_sub_;
  ros::Publisher state_pub_, sdk_joint_pub_;
  std::vector<ros::Publisher> gazebo_publishers_;
  ros::Timer control_timer_, output_timer_, sdk_state_timer_;
};
}  // namespace

int main(int argc, char** argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "eye_in_hand_visual_servo");
  EyeInHandVisualServo servo;
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
