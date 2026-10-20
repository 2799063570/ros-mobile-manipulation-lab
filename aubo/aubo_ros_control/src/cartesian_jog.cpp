#include <aubo_ros_control/cartesian_jog_common.h>
#include <aubo_ros_control/direct_sdk_backend.h>
#include <aubo_ros_control/MoveHomeAction.h>
#include <actionlib/server/action_server.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

namespace aubo_ros_control
{
using visual_servo_internal::JointPoint;
using visual_servo_internal::kDof;
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;

namespace
{
double elapsed(Time now, Time then) { return std::chrono::duration<double>(now - then).count(); }
constexpr double kPeriod = 0.005;  // SDK 透传点的执行周期，不能通过 ROS 调频改变它。
volatile std::sig_atomic_t stop_requested = 0;
void requestShutdown(int) { stop_requested = 1; }
}

class CartesianJog
{
  using HomeServer = actionlib::ActionServer<MoveHomeAction>;
  using HomeHandle = HomeServer::GoalHandle;
public:
  CartesianJog() : private_nh_("~"), io_("/cartesian_jog"), unused_queue_(2)
  {
    parameters();
    initializeModel();
    state_pub_ = io_.advertise<std_msgs::String>("state", 1, true);
    command_sub_ = io_.subscribe("command", 1, &CartesianJog::command, this,
                                 ros::TransportHints().tcpNoDelay());
    enable_service_ = io_.advertiseService("set_enabled", &CartesianJog::enable, this);
    reset_service_ = io_.advertiseService("reset", &CartesianJog::reset, this);

    if (backend_ == "sdk")
    {
      joint_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_topic_, 1);
      sdk_.reset(new visual_servo_internal::DirectSdkBackend(unused_queue_, velocity_limits_,
                                                            acceleration_limits_, 200.0));
      sdk_->setPointGenerator([this](JointPoint& point) { return generate(point); });
      if (!sdk_->connect(private_nh_))
        throw std::runtime_error("SDK connection failed");
      feedback_timer_ = nh_.createWallTimer(ros::WallDuration(0.02), &CartesianJog::readSdk, this);
    }
    else
    {
      std::vector<std::string> topics;
      if (!private_nh_.getParam("gazebo_command_topics", topics) || topics.size() != kDof)
        throw std::runtime_error("Expected six Gazebo command topics");
      for (const auto& topic : topics)
        joint_commands_.push_back(nh_.advertise<std_msgs::Float64>(topic, 1));
      feedback_sub_ = nh_.subscribe(joint_topic_, 1, &CartesianJog::readGazebo, this);
      output_timer_ = nh_.createTimer(ros::Duration(kPeriod), &CartesianJog::gazeboOutput, this);
    }
    status_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), &CartesianJog::publishStatus, this);
    home_server_.reset(new HomeServer(io_, "move_home",
        [this](HomeHandle goal) { startHome(goal); },
        [this](HomeHandle goal) { cancelHome(goal); }, false));
    home_server_->start();
    ROS_INFO("[cartesian_jog] %s 后端已启动，默认禁用；先发送零指令，再按住点动", backend_.c_str());
  }

  ~CartesianJog()
  {
    if (home_goal_)
    {
      MoveHomeResult result;
      result.message = "controller_exiting; stop not confirmed";
      home_goal_->setAborted(result);
    }
    // 正常退出在 main 中先请求减速；SDK 故障退出不承诺平滑停车。
    if (sdk_)
      sdk_->shutdown();
  }

  bool healthy()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return !fatal_ && (!sdk_ || sdk_->healthy());
  }

  void requestStop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = false;
    shutting_down_ = true;
    requested_.setZero();
    stopHome();
  }

  bool stopped()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return settled();
  }

private:
  void parameters()
  {
    private_nh_.param<std::string>("backend", backend_, "gazebo");
    private_nh_.param<std::string>("base_link", base_, "base_link");
    private_nh_.param<std::string>("tool_link", tool_, "tcp_link");
    private_nh_.param<std::string>("group_name", group_, "aubo_i5");
    private_nh_.param<std::string>("joint_states_topic", joint_topic_, "/joint_states");
    private_nh_.param<std::string>("planning_scene_topic", scene_topic_, "/planning_scene");
    private_nh_.param<std::string>("initial_scene_service", initial_scene_service_, "");
    private_nh_.param("max_linear_velocity", max_linear_, 0.02);
    private_nh_.param("command_timeout", command_timeout_, 0.15);
    private_nh_.param("feedback_timeout", feedback_timeout_, 0.15);
    private_nh_.param("max_following_error", following_error_, 0.12);
    private_nh_.param("joint_limit_margin", limit_margin_, 0.08);
    private_nh_.param("dls_lambda", damping_, 0.02);
    private_nh_.param("singularity_stop", singular_stop_, 0.008);
    private_nh_.param("singularity_slow", singular_slow_, 0.06);
    private_nh_.param("collision_resolution", collision_resolution_, 0.002);
    private_nh_.param("home_joint_velocity", home_velocity_, 0.10);
    private_nh_.param("home_timeout", home_timeout_, 90.0);
    if ((backend_ != "sdk" && backend_ != "gazebo") || base_ == tool_ ||
        !positive(max_linear_) || max_linear_ > 0.10 || !positive(command_timeout_) ||
        command_timeout_ > 0.5 || !positive(feedback_timeout_) || feedback_timeout_ > 0.5 ||
        !positive(following_error_) || !positive(limit_margin_) || !positive(damping_) ||
        !positive(singular_stop_) || !positive(singular_slow_) || singular_slow_ <= singular_stop_ ||
        !positive(collision_resolution_) || collision_resolution_ > 0.01 ||
        !positive(home_velocity_) || home_velocity_ > 0.25 || !positive(home_timeout_))
      throw std::runtime_error("Invalid Cartesian jog parameters");

    if (!private_nh_.getParam("joint_names", names_) || names_.size() != kDof)
      throw std::runtime_error("Expected six joint names");
    readArray("joint_lower_limits", lower_, false);
    readArray("joint_upper_limits", upper_, false);
    readArray("joint_velocity_limits", velocity_limits_, true);
    readArray("joint_acceleration_limits", acceleration_limits_, true);
    readArray("joint_jerk_limits", jerk_limits_, true);
    readArray("home_joint_positions", home_position_, false);
    for (std::size_t i = 0; i < kDof; ++i)
    {
      if (lower_[i] + 2 * limit_margin_ >= upper_[i])
        throw std::runtime_error("Joint limit margin leaves no working range");
      if (home_position_[i] < lower_[i] + limit_margin_ ||
          home_position_[i] > upper_[i] - limit_margin_)
        throw std::runtime_error("Home position exceeds joint limits");
      filters_.emplace_back(velocity_limits_[i], acceleration_limits_[i], jerk_limits_[i], kPeriod);
    }
  }

  static bool positive(double value) { return std::isfinite(value) && value > 0.0; }

  void readArray(const std::string& name, JointPoint& out, bool positive_only)
  {
    std::vector<double> data;
    if (!private_nh_.getParam(name, data) || data.size() != kDof)
      throw std::runtime_error("Invalid array: " + name);
    for (std::size_t i = 0; i < kDof; ++i)
    {
      if (!std::isfinite(data[i]) || (positive_only && data[i] <= 0))
        throw std::runtime_error("Invalid value: " + name);
      out[i] = data[i];
    }
  }

  void initializeModel()
  {
    std::string description;
    KDL::Tree tree;
    if (!nh_.getParam("robot_description", description) ||
        !kdl_parser::treeFromString(description, tree) ||
        !tree.getChain(base_, tool_, chain_) || chain_.getNrOfJoints() != kDof)
      throw std::runtime_error("Invalid robot_description/KDL chain");
    std::size_t index = 0;
    for (const auto& segment : chain_.segments)
      if (segment.getJoint().getType() != KDL::Joint::None &&
          segment.getJoint().getName() != names_.at(index++))
        throw std::runtime_error("joint_names must follow KDL chain order");
    fk_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    jacobian_.reset(new KDL::ChainJntToJacSolver(chain_));

    // 只使用 MoveIt 的机器人模型和碰撞场景，不调用规划器或轨迹执行器。
    // 必须加载对应 SRDF，否则相邻连杆可能被误判为自碰撞。
    if (!nh_.hasParam("robot_description_semantic"))
      throw std::runtime_error("Missing robot_description_semantic");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    scene_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_buffer_));
    if (!scene_->getPlanningScene() || !scene_->getRobotModel()->getJointModelGroup(group_))
      throw std::runtime_error("Could not create collision scene/group");
    for (std::size_t i = 0; i < kDof; ++i)
    {
      const auto* joint = scene_->getRobotModel()->getJointModel(names_[i]);
      if (!joint || joint->getVariableCount() != 1)
        throw std::runtime_error("Unknown/non-scalar joint: " + names_[i]);
      const auto& bounds = joint->getVariableBounds().front();
      if ((bounds.position_bounded_ &&
           (lower_[i] < bounds.min_position_ || upper_[i] > bounds.max_position_)) ||
          (bounds.velocity_bounded_ && velocity_limits_[i] > bounds.max_velocity_) ||
          (bounds.acceleration_bounded_ && acceleration_limits_[i] > bounds.max_acceleration_))
        throw std::runtime_error("Jog limits exceed model limits: " + names_[i]);
    }
    scene_->startSceneMonitor(scene_topic_);
    // 场景通过 planning_scene/world/collision_object 输入；不在点动节点里启动点云建图。
    scene_->startWorldGeometryMonitor("/collision_object", "/planning_scene_world", false);
    scene_->startStateMonitor(joint_topic_);
    scene_->setStateUpdateFrequency(50.0);
    // 与已有规划场景连接时显式要求完整快照，不能假定之前发布的 diff 仍在话题里。
    if (!initial_scene_service_.empty() &&
        !scene_->requestPlanningSceneState(initial_scene_service_))
      throw std::runtime_error("Could not fetch initial planning scene");
  }

  bool settled() const
  {
    for (const auto& filter : filters_)
      if (!filter.settled())
        return false;
    return true;
  }

  void fault(const std::string& reason)
  {
    if (fault_.empty())
    {
      fault_ = reason;
      ROS_ERROR("[cartesian_jog] %s；锁存故障并减速，需 reset 后重新使能", reason.c_str());
    }
    enabled_ = false;
    requested_.setZero();
    if (home_goal_)
      home_stopping_ = true;
  }

  void startHome(HomeHandle goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    MoveHomeResult result;
    if (goal.getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLING)
    {
      result.message = "Cancelled before motion";
      goal.setCanceled(result);
      return;
    }
    if (home_goal_ || enabled_ || shutting_down_ || !initialized_ || !fault_.empty() ||
        !settled() || elapsed(Clock::now(), last_feedback_) > feedback_timeout_ ||
        (sdk_ && !sdk_->healthy()) || !safeStop(position_, filters_))
    {
      result.message = "Disable jog first; feedback, stopped state and collision check must be healthy";
      goal.setRejected(result);
      return;
    }
    for (std::size_t i = 0; i < kDof; ++i)
      if (std::abs(position_[i] - feedback_[i]) > 0.01)
      {
        result.message = "Robot has not reached the held joint position";
        goal.setRejected(result);
        return;
      }
    // 不接管其他执行器，不调用阻塞式 SDK moveJoint；沿现有 5 ms 流生成点。
    home_goal_.reset(new HomeHandle(goal));
    home_stopping_ = false;
    home_cancelled_ = false;
    home_started_ = Clock::now();
    home_goal_->setAccepted("Moving to home with joint-space collision and braking checks");
  }

  void stopHome()
  {
    if (home_goal_)
    {
      home_cancelled_ = true;
      home_stopping_ = true;
    }
  }

  void cancelHome(HomeHandle goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (home_goal_ && goal == *home_goal_)
      stopHome();
  }

  Eigen::Matrix<double, 6, 1> homeVelocity()
  {
    Eigen::Matrix<double, 6, 1> velocity = Eigen::Matrix<double, 6, 1>::Zero();
    if (elapsed(Clock::now(), home_started_) > home_timeout_)
      fault("home_timeout");
    double error = 0.0;
    for (std::size_t i = 0; i < kDof; ++i)
      error = std::max(error, std::abs(home_position_[i] - position_[i]));
    if (error < 0.001)
      home_stopping_ = true;
    if (home_stopping_)
      return velocity;
    // 同比例关节误差回授，接近目标时降速；继续经过相同的 A/J 平滑与制动检查。
    double scale = 1.0;
    for (std::size_t i = 0; i < kDof; ++i)
    {
      velocity(i) = 0.8 * (home_position_[i] - position_[i]);
      if (std::abs(velocity(i)) > 0)
        scale = std::min(scale, std::min(home_velocity_, velocity_limits_[i]) / std::abs(velocity(i)));
    }
    return velocity * scale;
  }

  void updateHome()
  {
    if (!home_goal_)
      return;
    MoveHomeFeedback feedback;
    feedback.current_joint_positions.assign(feedback_.begin(), feedback_.end());
    feedback.target_joint_positions.assign(home_position_.begin(), home_position_.end());
    feedback.stopping = home_stopping_;
    double following = 0.0;
    for (std::size_t i = 0; i < kDof; ++i)
    {
      feedback.max_joint_error = std::max(feedback.max_joint_error, std::abs(home_position_[i] - feedback_[i]));
      following = std::max(following, std::abs(position_[i] - feedback_[i]));
    }
    home_goal_->publishFeedback(feedback);
    // 结果在平滑器停稳、反馈新鲜且实机反馈速度也接近零后才返回。
    if (!home_stopping_ || !settled() || following > 0.01 || feedback_speed_ > 0.01 ||
        elapsed(Clock::now(), last_feedback_) > feedback_timeout_)
      return;
    MoveHomeResult result;
    if (!fault_.empty())
    {
      result.message = fault_;
      home_goal_->setAborted(result);
    }
    else if (home_cancelled_)
    {
      result.message = "Cancelled and stopped; jog remains disabled";
      home_goal_->setCanceled(result);
    }
    else if (feedback.max_joint_error <= 0.005)
    {
      result.success = true;
      result.message = "Home reached; jog remains disabled";
      home_goal_->setSucceeded(result);
    }
    else
    {
      result.message = "Stopped outside home tolerance; jog remains disabled";
      home_goal_->setAborted(result);
    }
    home_goal_.reset();
  }

  void command(const geometry_msgs::TwistStamped::ConstPtr& message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_)
      return;
    const auto& t = message->twist;
    Eigen::Vector3d value(t.linear.x, t.linear.y, t.linear.z);
    const double age = (ros::Time::now() - message->header.stamp).toSec();
    if (!value.allFinite() || !std::isfinite(t.angular.x) || !std::isfinite(t.angular.y) ||
        !std::isfinite(t.angular.z) || t.angular.x != 0 || t.angular.y != 0 || t.angular.z != 0 ||
        (message->header.frame_id != base_ && message->header.frame_id != tool_) ||
        message->header.stamp.isZero() || age > command_timeout_ || age < -0.05 ||
        (!last_stamp_.isZero() && message->header.stamp < last_stamp_))
    {
      fault("invalid_or_stale_command");
      return;
    }
    // Gazebo 的 /clock 分辨率可能低于 GUI 事件频率；同时间戳消息直接丢弃，
    // 不刷新看门狗，避免重复旧消息维持运动，也避免正常双击/松手误触发故障。
    if (!last_stamp_.isZero() && message->header.stamp == last_stamp_)
      return;
    // 每次使能必须先收到零指令，避免按键已按住时使能导致立即运动。
    if (!neutral_seen_ && value.norm() > 1e-12)
      return;
    neutral_seen_ = true;
    if (value.norm() > max_linear_)
      value *= max_linear_ / value.norm();
    requested_ = value;
    tool_frame_ = message->header.frame_id == tool_;
    last_command_ = Clock::now();
    last_stamp_ = message->header.stamp;
  }

  bool enable(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response.success = true;
    if (!request.data)
    {
      enabled_ = false;
      requested_.setZero();
      stopHome();
      response.message = "已请求平滑停止；控制柜已有点仍需执行";
      return true;
    }
    if (enabled_)
    {
      response.message = "已使能";
      return true;
    }
    if (home_goal_ || shutting_down_ || !initialized_ || !fault_.empty() || !settled() ||
        elapsed(Clock::now(), last_feedback_) > feedback_timeout_ ||
        (sdk_ && !sdk_->healthy()) || !safeStop(position_, filters_))
    {
      response.success = false;
      response.message = "未就绪、尚未停稳、位姿不安全或故障未复位：" + fault_;
      return true;
    }
    enabled_ = true;
    neutral_seen_ = false;
    capture_orientation_ = true;
    requested_.setZero();
    last_stamp_ = ros::Time();
    last_command_ = Clock::now();
    response.message = "已使能，请先发送零指令，然后持续发送点动速度";
    return true;
  }

  bool reset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response.success = initialized_ && !enabled_ && !home_goal_ && !fatal_ && settled() &&
                       elapsed(Clock::now(), last_feedback_) <= feedback_timeout_ &&
                       (!sdk_ || sdk_->healthy()) && safeStop(position_, filters_);
    for (std::size_t i = 0; i < kDof; ++i)
      response.success = response.success && std::abs(position_[i] - feedback_[i]) < following_error_;
    if (response.success)
      fault_.clear();
    response.message = response.success ? "故障已复位，仍保持禁用" : "复位失败：检查反馈、碰撞和停止状态";
    return true;
  }

  void feedback(const JointPoint& point)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!visual_servo_internal::finitePoint(point))
    {
      fault("invalid_feedback");
      return;
    }
    const Time now = Clock::now();
    const double dt = elapsed(now, last_feedback_);
    feedback_speed_ = 0.0;
    if (have_feedback_ && dt > 1e-6)
      for (std::size_t i = 0; i < kDof; ++i)
        feedback_speed_ = std::max(feedback_speed_, std::abs(point[i] - feedback_[i]) / dt);
    feedback_ = point;
    have_feedback_ = true;
    last_feedback_ = Clock::now();
  }

  void readSdk(const ros::WallTimerEvent&)
  {
    JointPoint point;
    if (!sdk_->readState(point))
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fault("sdk_feedback_failed");
      return;
    }
    feedback(point);
    sensor_msgs::JointState message;
    message.header.stamp = ros::Time::now();
    message.name = names_;
    message.position.assign(point.begin(), point.end());
    joint_pub_.publish(message);
  }

  void readGazebo(const sensor_msgs::JointState::ConstPtr& message)
  {
    JointPoint point;
    for (std::size_t i = 0; i < kDof; ++i)
    {
      const auto found = std::find(message->name.begin(), message->name.end(), names_[i]);
      const auto index = static_cast<std::size_t>(found - message->name.begin());
      if (found == message->name.end() || index >= message->position.size())
        return;
      point[i] = message->position[index];
    }
    feedback(point);
  }

  Eigen::Matrix<double, 6, 1> desiredVelocity()
  {
    Eigen::Matrix<double, 6, 1> result = Eigen::Matrix<double, 6, 1>::Zero();
    if (!enabled_ || requested_.norm() < 1e-12)
      return result;
    KDL::JntArray q(kDof);
    for (std::size_t i = 0; i < kDof; ++i)
      q(i) = position_[i];
    KDL::Frame frame;
    KDL::Jacobian jac(kDof);
    if (fk_->JntToCart(q, frame) < 0 || jacobian_->JntToJac(q, jac) < 0)
    {
      fault("kinematics_failed");
      return result;
    }
    if (capture_orientation_)
    {
      hold_rotation_ = frame.M;
      capture_orientation_ = false;
    }
    KDL::Vector linear(requested_.x(), requested_.y(), requested_.z());
    if (tool_frame_)
      linear = frame.M * linear;
    const KDL::Vector rotation_error = KDL::diff(frame.M, hold_rotation_);
    Eigen::Matrix<double, 6, 1> twist;
    for (int i = 0; i < 3; ++i)
    {
      twist(i) = linear(i);
      twist(i + 3) = jog::VelocitySmoother::clamp(2.0 * rotation_error(i), -0.1, 0.1);
    }
    const Eigen::Matrix<double, 6, 6> matrix = jac.data;
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(matrix);
    const double smallest = svd.singularValues().minCoeff();
    if (smallest <= singular_stop_)
    {
      fault("singularity_stop");
      return result;
    }
    result = matrix.transpose() *
             (matrix * matrix.transpose() + damping_ * damping_ * Eigen::Matrix<double, 6, 6>::Identity())
                 .ldlt().solve(twist);
    if (!result.allFinite())
    {
      fault("invalid_joint_velocity");
      return Eigen::Matrix<double, 6, 1>::Zero();
    }
    double scale = std::min(1.0, (smallest - singular_stop_) / (singular_slow_ - singular_stop_));
    for (std::size_t i = 0; i < kDof; ++i)
    {
      if (std::abs(result(i)) > velocity_limits_[i])
        scale = std::min(scale, velocity_limits_[i] / std::abs(result(i)));
      // 整体缩放六轴速度，避免单轴截断造成末端方向突变。
      const double room = result(i) >= 0 ? upper_[i] - limit_margin_ - position_[i]
                                         : position_[i] - lower_[i] - limit_margin_;
      scale = std::min(scale, jog::VelocitySmoother::clamp(
          (room - filters_[i].stoppingDistance()) / 0.10, 0.0, 1.0));
    }
    return result * scale;
  }

  bool safeStop(const JointPoint& start, const std::vector<jog::VelocitySmoother>& filters)
  {
    // 检查候选点及其零指令减速尾段；不只检查“下一点没撞”。
    // 按最大关节增量采样碰撞，所有 5 ms 预测点都检查关节限位。
    // 这是离散模型检查，不提供连续碰撞或未知障碍物的安全保证。
    planning_scene_monitor::LockedPlanningSceneRO scene(scene_);
    moveit::core::RobotState state(scene->getCurrentState());
    JointPoint q = start, last_checked = start;
    auto braking = filters;
    for (int step = 0; step <= 600; ++step)
    {
      bool done = true;
      double travel = 0;
      for (std::size_t i = 0; i < kDof; ++i)
      {
        if (!std::isfinite(q[i]) || q[i] < lower_[i] + limit_margin_ || q[i] > upper_[i] - limit_margin_)
          return false;
        travel = std::max(travel, std::abs(q[i] - last_checked[i]));
        done = done && braking[i].settled();
      }
      if (step == 0 || done || travel >= collision_resolution_ || step == 600)
      {
        for (std::size_t i = 0; i < kDof; ++i)
          state.setVariablePosition(names_[i], q[i]);
        state.update();
        if (scene->isStateColliding(state))
          return false;
        last_checked = q;
      }
      if (done)
        return true;
      for (std::size_t i = 0; i < kDof; ++i)
        q[i] += braking[i].step(0.0) * kPeriod;
    }
    return false;  // 配置过于缓慢，3 秒内无法停稳时也拒绝运动。
  }

  bool generate(JointPoint& output)
  {
    const Time started = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_feedback_ || fatal_)
      return false;
    if (!initialized_)
    {
      // SDK 传入上一个实际下发点；不要用稍晚读取的反馈来重置轨迹起点。
      position_ = backend_ == "sdk" ? output : feedback_;
      initialized_ = true;
      last_generated_ = started;
    }
    if (elapsed(started, last_feedback_) > feedback_timeout_)
      fault("feedback_timeout");
    if (elapsed(started, last_generated_) > 0.08)
      fault("stream_schedule_gap");
    last_generated_ = started;
    if (enabled_ && elapsed(started, last_command_) > command_timeout_)
      fault("command_timeout");
    for (std::size_t i = 0; i < kDof; ++i)
      if (std::abs(position_[i] - feedback_[i]) > following_error_)
        fault("following_error");

    const auto desired = home_goal_ ? homeVelocity() : desiredVelocity();
    auto next_filters = filters_;
    JointPoint next = position_;
    for (std::size_t i = 0; i < kDof; ++i)
      next[i] += next_filters[i].step(desired(i)) * kPeriod;
    if (!safeStop(next, next_filters))
    {
      fault("collision_or_stopping_limit");
      next_filters = filters_;
      next = position_;
      // 上周期已经验证过这一减速尾段。故障时保持衔接，不清空轨迹或跳回反馈。
      for (std::size_t i = 0; i < kDof; ++i)
        next[i] += next_filters[i].step(0.0) * kPeriod;
    }
    for (std::size_t i = 0; i < kDof; ++i)
      if (!std::isfinite(next[i]) || next[i] < lower_[i] || next[i] > upper_[i])
      {
        fault("hard_joint_limit");
        fatal_ = true;
        return false;
      }
    position_ = next;
    filters_ = std::move(next_filters);
    output = position_;
    if (elapsed(Clock::now(), started) > 0.02)
    {
      fault("control_computation_overrun");
      // 已经超出供点预算时不再发送本次新点，禁止停顿后恢复成一次突发补点。
      fatal_ = true;
      return false;
    }
    return true;
  }

  void gazeboOutput(const ros::TimerEvent&)
  {
    JointPoint point{};
    if (!generate(point))
      return;
    for (std::size_t i = 0; i < kDof; ++i)
    {
      std_msgs::Float64 message;
      message.data = point[i];
      joint_commands_[i].publish(message);
    }
  }

  void publishStatus(const ros::WallTimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    updateHome();
    std_msgs::String message;
    if (!fault_.empty())
      message.data = "FAULT: " + fault_ + (settled() ? " | STOPPED" : " | STOPPING");
    else if (home_goal_)
      message.data = home_stopping_ ? "HOME_STOPPING" : "HOME_MOVING";
    else if (!initialized_)
      message.data = "WAITING_FOR_FEEDBACK";
    else if (!enabled_)
      message.data = settled() ? "DISABLED" : "STOPPING";
    else
      message.data = settled() && requested_.norm() < 1e-12 ? "ENABLED_IDLE" : "JOGGING";
    state_pub_.publish(message);
  }

  ros::NodeHandle nh_, private_nh_, io_;
  ros::Publisher state_pub_, joint_pub_;
  ros::Subscriber command_sub_, feedback_sub_;
  ros::ServiceServer enable_service_, reset_service_;
  ros::Timer output_timer_;
  ros::WallTimer feedback_timer_, status_timer_;
  std::vector<ros::Publisher> joint_commands_;
  std::string backend_, base_, tool_, group_, joint_topic_;
  std::string scene_topic_, initial_scene_service_;
  std::vector<std::string> names_;
  double max_linear_, command_timeout_, feedback_timeout_, following_error_, limit_margin_;
  double damping_, singular_stop_, singular_slow_, collision_resolution_;
  JointPoint lower_{}, upper_{}, velocity_limits_{}, acceleration_limits_{}, jerk_limits_{};
  JointPoint feedback_{}, position_{};
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_;
  visual_servo_internal::CommandQueue unused_queue_;
  std::vector<jog::VelocitySmoother> filters_;
  std::mutex mutex_;
  bool enabled_{false}, neutral_seen_{false}, initialized_{false}, have_feedback_{false};
  bool tool_frame_{false}, capture_orientation_{true}, fatal_{false};
  bool shutting_down_{false};
  Eigen::Vector3d requested_{Eigen::Vector3d::Zero()};
  KDL::Rotation hold_rotation_;
  Time last_feedback_{}, last_command_{}, last_generated_{};
  ros::Time last_stamp_;
  std::string fault_;
  std::unique_ptr<HomeServer> home_server_;
  std::unique_ptr<HomeHandle> home_goal_;
  JointPoint home_position_{};
  double home_velocity_, home_timeout_, feedback_speed_{0.0};
  bool home_stopping_{false}, home_cancelled_{false};
  Time home_started_{};
  // 放在最后：构造抛异常时也先销毁 SDK 输出线程，再释放它引用的其他成员。
  std::unique_ptr<visual_servo_internal::DirectSdkBackend> sdk_;
};
}  // namespace aubo_ros_control

int main(int argc, char** argv)
{
  // 保留 spinner 直到平滑停车完成；SIGINT 默认 ros::shutdown 会直接结束 SDK 喂点。
  ros::init(argc, argv, "aubo_cartesian_jog", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, aubo_ros_control::requestShutdown);
  std::signal(SIGTERM, aubo_ros_control::requestShutdown);
  try
  {
    aubo_ros_control::CartesianJog jog;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::WallRate rate(50);
    while (ros::ok() && jog.healthy() && !aubo_ros_control::stop_requested)
      rate.sleep();
    jog.requestStop();
    const auto deadline = aubo_ros_control::Clock::now() + std::chrono::seconds(3);
    while (ros::ok() && jog.healthy() && !jog.stopped() && aubo_ros_control::Clock::now() < deadline)
      rate.sleep();
    // 给名义 30 ms 的控制柜缓冲留出排空时间；异常断线无法执行此平滑退出保证。
    if (ros::ok() && jog.healthy())
      ros::WallDuration(0.10).sleep();
    const bool normal = jog.healthy();
    spinner.stop();
    return normal ? 0 : 1;
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("[cartesian_jog] %s", error.what());
    return 1;
  }
  return 0;
}
