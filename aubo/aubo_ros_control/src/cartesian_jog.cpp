#include <clocale>
#include <aubo_ros_control/cartesian_jog_common.h>
#include <aubo_ros_control/direct_sdk_backend.h>
#include <aubo_ros_control/MoveHomeAction.h>
#include <actionlib/server/action_server.h>
#include <controller_manager_msgs/ListControllers.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/callback_queue.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

#include <chrono>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <utility>

namespace aubo_ros_control
{
using visual_servo_internal::JointPoint;
using visual_servo_internal::kDof;
using visual_servo_internal::MotionTimingTrace;
using visual_servo_internal::PointTiming;
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
  using JointVelocity = Eigen::Matrix<double, 6, 1>;
public:
  CartesianJog() : private_nh_("~"), io_("/cartesian_jog"), unused_queue_(2)
  {
    parameters();
    std::string trace_path;
    private_nh_.param<std::string>("timing_trace_path", trace_path, "");
    if (!trace_path.empty())
    {
      int capacity = 200000;
      private_nh_.param("timing_trace_capacity", capacity, capacity);
      if (backend_ != "sdk" || capacity <= 0)
        throw std::invalid_argument("Timing trace requires SDK backend and positive capacity");
      timing_trace_.reset(new MotionTimingTrace(trace_path, capacity));
      ROS_INFO("[cartesian_jog/trace] 内存采集已启用，退出后写入 %s；容量 %d 条事件",
               trace_path.c_str(), capacity);
    }
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
      sdk_->setTimingTrace(timing_trace_);
      sdk_->setPointGenerator([this](JointPoint& point, PointTiming* timing) { return generate(point, timing); });
      if (!sdk_->connect(private_nh_))
        throw std::runtime_error("SDK connection failed");
      feedback_timer_ = nh_.createWallTimer(ros::WallDuration(0.02), &CartesianJog::readSdk, this);
    }
    else
    {
      if (backend_ == "ros_control")
      {
        // Feedback must not queue behind scene updates, IK/output timers or
        // synchronous readiness service calls on the default callback queue.
        feedback_nh_.setCallbackQueue(&feedback_queue_);
        trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(controller_command_topic_, 1);
        controller_list_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>(
            controller_manager_service_);
      }
      else
      {
        std::vector<std::string> topics;
        if (!private_nh_.getParam("gazebo_command_topics", topics) || topics.size() != kDof)
          throw std::runtime_error("Expected six Gazebo command topics");
        for (const auto& topic : topics)
          joint_commands_.push_back(nh_.advertise<std_msgs::Float64>(topic, 1));
      }
      feedback_sub_ = feedback_nh_.subscribe(joint_topic_, 1, &CartesianJog::readGazebo, this,
                                    ros::TransportHints().tcpNoDelay());
      output_timer_ = nh_.createTimer(ros::Duration(kPeriod), &CartesianJog::gazeboOutput, this);
    }
    status_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), &CartesianJog::publishStatus, this);
    home_server_.reset(new HomeServer(io_, "move_home",
        [this](HomeHandle goal) { startHome(goal); },
        [this](HomeHandle goal) { cancelHome(goal); }, false));
    home_server_->start();
    if (backend_ == "ros_control")
    {
      feedback_spinner_.reset(new ros::AsyncSpinner(1, &feedback_queue_));
      feedback_spinner_->start();
    }
    ROS_INFO("[cartesian_jog] %s 后端已启动，默认禁用；先发送零指令，再按住点动", backend_.c_str());
    if (backend_ != "ros_control")
    {
      ROS_INFO("[cartesian_jog] 目标更新 %.1f Hz，按 5 ms 周期平滑插补；SDK 按需取点，不经过 JTC",
               1.0 / control_period_);
      control_running_.store(true);
      control_thread_ = std::thread(&CartesianJog::controlLoop, this);
    }
  }

  ~CartesianJog()
  {
    control_running_.store(false);
    control_wakeup_.notify_all();
    if (control_thread_.joinable())
      control_thread_.join();
    feedback_sub_.shutdown();
    if (feedback_spinner_)
      feedback_spinner_->stop();
    if (home_goal_)
    {
      MoveHomeResult result;
      result.message = "controller_exiting; stop not confirmed";
      home_goal_->setAborted(result);
    }
    // 正常退出在 main 中先请求减速；SDK 故障退出不承诺平滑停车。
    if (sdk_)
      sdk_->shutdown();
    // main has already stopped the spinner; the control and SDK writers are
    // now joined. Never serialize this buffer on a motion/callback thread.
    if (timing_trace_)
    {
      if (!timing_trace_->dumpAfterJoin())
        ROS_ERROR("[cartesian_jog/trace] 写入采集文件失败");
      else
        ROS_INFO("[cartesian_jog/trace] 已保存；容量外丢弃 %zu 条事件",
                 timing_trace_->droppedAfterJoin());
    }
  }

  bool healthy()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return !fatal_ && (!sdk_ || sdk_->healthy());
  }

  void requestStop()
  {
    ++stop_epoch_;
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = false;
    shutting_down_ = true;
    requested_.setZero();
    invalidateControlTarget();
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
    private_nh_.param<std::string>("scene_joint_states_topic", scene_joint_topic_, joint_topic_);
    private_nh_.param<std::string>("controller_command_topic", controller_command_topic_,
                                  "/aubo_i5/aubo_i5_controller/command");
    private_nh_.param<std::string>("controller_manager_service", controller_manager_service_,
                                  "/aubo_i5/controller_manager/list_controllers");
    private_nh_.param<std::string>("controller_name", controller_name_, "aubo_i5_controller");
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
    double control_frequency = 50.0;
    private_nh_.param("control_frequency", control_frequency, 50.0);
    control_steps_ = jog::interpolationSteps(control_frequency, kPeriod);
    control_period_ = control_steps_ * kPeriod;
    if ((backend_ != "sdk" && backend_ != "gazebo" && backend_ != "ros_control") ||
        (backend_ == "ros_control" && (controller_command_topic_.empty() || joint_topic_.empty() ||
         controller_command_topic_ == joint_topic_ || controller_manager_service_.empty() ||
         controller_name_.empty())) || base_ == tool_ ||
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
    // The scene may include explicitly fixed gripper joints in a merged topic.
    // Control freshness is still checked only against joint_topic_ raw samples.
    scene_->startStateMonitor(scene_joint_topic_);
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

  bool sdkFeedbackHealthy()
  {
    if (!sdk_)
      return true;
    JointPoint point;
    // readState checks the age of the actual SDK callback, not the last ROS
    // timer that copied it. It reads a short cache lock; it performs no RPC.
    return sdk_->healthy() && sdk_->readState(point);
  }

  void fault(const std::string& reason)
  {
    if (fault_.empty())
    {
      fault_ = reason;
      if (reason.compare(0, 18, "sdk_backend_failed") == 0)
        ROS_ERROR("[cartesian_jog] %s；后端已停流，无法继续生成减速点，需重启节点", reason.c_str());
      else
        ROS_ERROR("[cartesian_jog] %s；锁存故障并减速，需 reset 后重新使能", reason.c_str());
    }
    enabled_ = false;
    requested_.setZero();
    invalidateControlTarget();
    if (home_goal_)
      home_stopping_ = true;
  }

  bool controllerRunning(std::string& reason)
  {
    if (backend_ != "ros_control")
      return true;
    // The controller may subscribe while stopped. Ask its manager rather than
    // treating a connected topic subscriber as permission to start motion.
    // Callers must do this before locking mutex_: an unavailable ROS service
    // must not block the generator, disable or cancellation callbacks.
    controller_manager_msgs::ListControllers service;
    if (!controller_list_client_.call(service))
    {
      reason = "无法查询控制器状态：" + controller_manager_service_;
      return false;
    }
    for (const auto& controller : service.response.controller)
      if (controller.name == controller_name_ && controller.state == "running")
        return true;
    reason = "控制器未处于 running 状态：" + controller_name_;
    return false;
  }

  void startHome(HomeHandle goal)
  {
    const auto stop_epoch = stop_epoch_.load();
    std::string controller_error;
    const bool controller_ready = controllerRunning(controller_error);
    std::lock_guard<std::mutex> lock(mutex_);
    MoveHomeResult result;
    if (goal.getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLING)
    {
      result.message = "Cancelled before motion";
      goal.setCanceled(result);
      return;
    }
    if (!controller_ready)
    {
      result.message = controller_error;
      goal.setRejected(result);
      return;
    }
    if (stop_epoch != stop_epoch_.load())
    {
      result.message = "查询控制器期间已收到停止请求，请重新请求 home";
      goal.setRejected(result);
      return;
    }
    if (home_goal_ || enabled_ || shutting_down_ || !initialized_ || !fault_.empty() ||
        !settled() || elapsed(Clock::now(), last_feedback_) > feedback_timeout_ ||
        !sdkFeedbackHealthy() || !safeStop(position_, filters_))
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
    if (sdk_ && !sdk_->armMotion())
    {
      result.message = "SDK diagnosis not ready; wait for fresh empty MAC observations and request home again";
      goal.setRejected(result);
      return;
    }
    invalidateControlTarget();
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
      invalidateControlTarget();
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
    const auto received = timing_trace_ ? MotionTimingTrace::ns() : 0;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto locked = timing_trace_ ? MotionTimingTrace::ns() : 0;
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
    if (timing_trace_)
    {
      MotionTimingTrace::Event event;
      event.kind = MotionTimingTrace::Command;
      event.command_id = ++trace_command_id_;
      event.begin_ns = received;
      event.locked_ns = locked;
      event.end_ns = MotionTimingTrace::ns(last_command_);
      event.source_ros_ns = message->header.stamp.toNSec();
      event.result = tool_frame_ ? 1 : 0;  // 0=base, 1=tool
      for (int i = 0; i < 3; ++i) event.desired[i] = requested_(i);
      timing_trace_->append(event);
    }
  }

  bool enable(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
  {
    const auto stop_epoch = stop_epoch_.load();
    if (!request.data)
      ++stop_epoch_;
    std::string controller_error;
    const bool controller_ready = !request.data || controllerRunning(controller_error);
    std::lock_guard<std::mutex> lock(mutex_);
    response.success = true;
    if (!request.data)
    {
      enabled_ = false;
      invalidateControlTarget();
      requested_.setZero();
      stopHome();
      response.message = "已请求平滑停止；控制柜已有点仍需执行";
      return true;
    }
    if (!controller_ready)
    {
      if (enabled_)
        fault("ros_control_controller_not_running");
      response.success = false;
      response.message = controller_error;
      return true;
    }
    if (stop_epoch != stop_epoch_.load())
    {
      response.success = false;
      response.message = "查询控制器期间已收到停止请求，请重新使能";
      return true;
    }
    if (enabled_)
    {
      response.message = "已使能";
      return true;
    }
    if (home_goal_ || shutting_down_ || !initialized_ || !fault_.empty() || !settled() ||
        elapsed(Clock::now(), last_feedback_) > feedback_timeout_ ||
        !sdkFeedbackHealthy() || !safeStop(position_, filters_))
    {
      response.success = false;
      response.message = "未就绪、尚未停稳、位姿不安全或故障未复位：" + fault_;
      return true;
    }
    if (sdk_ && !sdk_->armMotion())
    {
      response.success = false;
      response.message = "SDK诊断尚未就绪；等待连续有效空MAC诊断后重新使能";
      return true;
    }
    enabled_ = true;
    invalidateControlTarget();
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
                       sdkFeedbackHealthy() && safeStop(position_, filters_);
    for (std::size_t i = 0; i < kDof; ++i)
      response.success = response.success && std::abs(position_[i] - feedback_[i]) < following_error_;
    if (response.success)
      fault_.clear();
    response.message = response.success ? "故障已复位，仍保持禁用" : "复位失败：检查反馈、碰撞和停止状态";
    return true;
  }

  void feedback(const JointPoint& point, Time sampled = Clock::now(), double transport_age = 0.0)
  {
    const auto received = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!visual_servo_internal::finitePoint(point))
    {
      fault("invalid_feedback");
      return;
    }
    const double dt = elapsed(sampled, last_feedback_);
    feedback_speed_ = 0.0;
    if (have_feedback_ && dt > 1e-6)
      for (std::size_t i = 0; i < kDof; ++i)
        feedback_speed_ = std::max(feedback_speed_, std::abs(point[i] - feedback_[i]) / dt);
    feedback_ = point;
    if (have_feedback_)
      max_feedback_gap_ = std::max(max_feedback_gap_, elapsed(received, last_feedback_received_));
    ++feedback_samples_;
    last_feedback_received_ = received;
    feedback_transport_age_ = transport_age;
    feedback_lock_wait_ = elapsed(Clock::now(), received);
    have_feedback_ = true;
    last_feedback_ = sampled;
  }

  void readSdk(const ros::WallTimerEvent&)
  {
    JointPoint point;
    if (!sdk_->readState(point))
    {
      const auto backend_failure = sdk_->failureReason();
      std::lock_guard<std::mutex> lock(mutex_);
      fault(backend_failure.empty() ? "sdk_feedback_failed" : "sdk_backend_failed: " + backend_failure);
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
    Time sampled = Clock::now();
    double age = 0.0;
    if (backend_ == "ros_control")
    {
      // This topic must contain original hardware samples, not a relay that
      // republishes cached positions with a new stamp. Delayed samples must
      // not refresh the physical-feedback watchdog on receipt.
      age = (ros::Time::now() - message->header.stamp).toSec();
      if (message->header.stamp.isZero() || age > feedback_timeout_ || age < -0.05 ||
          message->name.size() != message->position.size())
      {
        ROS_ERROR_THROTTLE(1.0, "[cartesian_jog/feedback] 原始反馈无效或过期：消息年龄 %.2f ms，seq=%u",
                           age * 1000.0, message->header.seq);
        std::lock_guard<std::mutex> lock(mutex_);
        fault("invalid_or_stale_feedback");
        return;
      }
      sampled -= std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(std::max(0.0, age)));
    }
    JointPoint point;
    for (std::size_t i = 0; i < kDof; ++i)
    {
      const auto found = std::find(message->name.begin(), message->name.end(), names_[i]);
      const auto index = static_cast<std::size_t>(found - message->name.begin());
      if (found == message->name.end() || index >= message->position.size())
        return;
      point[i] = message->position[index];
    }
    feedback(point, sampled, age);
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

  // Caller holds mutex_. Guards operate on wall/steady time; the interpolator
  // always advances by the fixed point period, never by RPC or timer jitter.
  bool prepareGeneration(Time started)
  {
    if (!have_feedback_ || fatal_)
      return false;
    if (!initialized_)
    {
      // SDK 传入上一个实际下发点；不要用稍晚读取的反馈来重置轨迹起点。
      // The SDK caller seeds position_ separately from its last actual point.
      if (backend_ != "sdk")
        position_ = feedback_;
      initialized_ = true;
      last_generated_ = started;
    }
    if (elapsed(started, last_feedback_) > feedback_timeout_)
    {
      if (fault_.empty())
        ROS_ERROR("[cartesian_jog/feedback] 采样年龄 %.2f ms，接收间隔最大 %.2f ms，"
                  "上帧传输/回调排队 %.2f ms，上帧锁等待 %.2f ms，已收 %llu 帧",
                  elapsed(started, last_feedback_) * 1000.0, max_feedback_gap_ * 1000.0,
                  feedback_transport_age_ * 1000.0, feedback_lock_wait_ * 1000.0,
                  static_cast<unsigned long long>(feedback_samples_));
      fault("feedback_timeout");
    }
    // 尚未发起过运动的禁用等待阶段没有轨迹流，不能将空闲查询间隔
    // 当成运动断流。使能/home 或首个非零关节增量之后仍严格检查。
    // SDK calls on demand while MAC already contains a buffered trajectory.
    // Its own diagnosis/send watchdogs detect stalled output; a full cabinet
    // legitimately needs no new generated point for longer than a timer period.
    if (backend_ != "sdk" && elapsed(started, last_generated_) > 0.08 &&
        (motion_started_ || enabled_ || home_goal_))
      fault("stream_schedule_gap");
    last_generated_ = started;
    if (enabled_ && elapsed(started, last_command_) > command_timeout_)
      fault("command_timeout");
    for (std::size_t i = 0; i < kDof; ++i)
      if (std::abs(position_[i] - feedback_[i]) > following_error_)
        fault("following_error");

    return true;
  }

  bool interpolatePoint(JointPoint& output, const JointVelocity& desired, PointTiming* timing = nullptr)
  {
    if (timing)
      for (std::size_t i = 0; i < kDof; ++i)
        timing->desired[i] = desired(i);
    auto next_filters = filters_;
    JointPoint next = position_;
    for (std::size_t i = 0; i < kDof; ++i)
      next[i] += next_filters[i].step(desired(i)) * kPeriod;
    if (!safeStop(next, next_filters))
    {
      fault("collision_or_stopping_limit");
      next_filters = filters_;
      next = position_;
      if (timing)
        timing->desired.fill(0.0);  // Record the braking input actually used below.
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
    motion_started_ = motion_started_ || next != position_;
    position_ = next;
    filters_ = std::move(next_filters);
    output = position_;
    return true;
  }

  bool generate(JointPoint& output, PointTiming* timing = nullptr)
  {
    const Time started = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    const Time locked = Clock::now();
    if (!initialized_ && backend_ == "sdk")
      position_ = output;
    if (!prepareGeneration(started))
      return false;
    const auto desired = pointTarget(started);
    if (!interpolatePoint(output, desired, timing))
      return false;
    if (timing)
    {
      timing->begin_ns = MotionTimingTrace::ns(started);
      timing->locked_ns = MotionTimingTrace::ns(locked);
      timing->end_ns = MotionTimingTrace::ns();
      timing->target_id = trace_target_id_;
      timing->command_id = home_goal_ ? 0 : trace_command_id_;
      for (std::size_t i = 0; i < kDof; ++i)
      {
        timing->velocity[i] = filters_[i].velocity();
      }
    }
    if (elapsed(Clock::now(), started) > 0.02)
    {
      ROS_ERROR("[cartesian_jog] 供点超时：总耗时 %.2f ms，锁等待 %.2f ms，计算 %.2f ms",
                elapsed(Clock::now(), started) * 1000.0,
                elapsed(locked, started) * 1000.0, elapsed(Clock::now(), locked) * 1000.0);
      fault("control_computation_overrun");
      // 已经超出供点预算时不再发送本次新点，禁止停顿后恢复成一次突发补点。
      fatal_ = true;
      return false;
    }
    return true;
  }

  void controlLoop()
  {
    while (control_running_.load() && ros::ok())
    {
      const auto started = Clock::now();
      try
      {
        updateControlTarget(started);
      }
      catch (const std::exception& error)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ROS_ERROR("[cartesian_jog] 控制线程异常：%s", error.what());
        fault("control_exception");
        fatal_ = true;
      }
      // Schedule from this actual cycle start. A delayed cycle is never
      // followed by a burst of catch-up trajectories.
      std::unique_lock<std::mutex> lock(control_wait_mutex_);
      control_wakeup_.wait_until(lock, started + std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(control_period_)),
          [this] { return !control_running_.load(); });
    }
  }

  void updateControlTarget(Time started)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto locked = timing_trace_ ? MotionTimingTrace::ns() : 0;
    if (!initialized_ || !have_feedback_ || fatal_)
      return;
    if (elapsed(started, last_feedback_) > feedback_timeout_)
      fault("feedback_timeout");
    if (enabled_ && elapsed(started, last_command_) > command_timeout_)
      fault("command_timeout");
    JointVelocity target = JointVelocity::Zero();
    if (!shutting_down_ && fault_.empty())
      target = home_goal_ ? homeVelocity() : desiredVelocity();
    if (elapsed(Clock::now(), started) >= control_period_)
    {
      fault("control_computation_overrun");
      fatal_ = true;
      return;
    }
    control_target_ = target;
    last_control_update_ = started;
    control_target_ready_ = true;
    if (timing_trace_)
    {
      MotionTimingTrace::Event event;
      event.kind = MotionTimingTrace::Target;
      event.target_id = ++trace_target_sequence_;
      trace_target_id_ = event.target_id;
      event.command_id = home_goal_ ? 0 : trace_command_id_;
      event.begin_ns = MotionTimingTrace::ns(started);
      event.locked_ns = locked;
      event.end_ns = MotionTimingTrace::ns();
      event.result = home_goal_ ? 1 : 0;
      for (std::size_t i = 0; i < kDof; ++i) event.desired[i] = target(i);
      timing_trace_->append(event);
    }
  }

  void invalidateControlTarget()
  {
    control_target_.setZero();
    control_target_ready_ = false;
    trace_target_id_ = 0;
    last_control_update_ = Clock::now();
  }

  JointVelocity pointTarget(Time now)
  {
    if (backend_ == "ros_control")  // Legacy optional path, not the real default.
      return home_goal_ ? homeVelocity() : desiredVelocity();
    if (shutting_down_ || !fault_.empty() ||
        (home_goal_ && home_stopping_) ||
        (!home_goal_ && (!enabled_ || !neutral_seen_ || requested_.norm() < 1e-12)))
      return JointVelocity::Zero();
    if (elapsed(now, last_control_update_) > 2.0 * control_period_)
    {
      fault("control_target_timeout");
      return JointVelocity::Zero();
    }
    return control_target_ready_ ? control_target_ : JointVelocity::Zero();
  }

  void gazeboOutput(const ros::TimerEvent&)
  {
    JointPoint point{};
    if (!generate(point))
      return;
    if (backend_ == "ros_control")
    {
      trajectory_msgs::JointTrajectory message;
      message.header.stamp = ros::Time(0);
      message.joint_names = names_;
      trajectory_msgs::JointTrajectoryPoint waypoint;
      waypoint.positions.assign(point.begin(), point.end());
      waypoint.time_from_start = ros::Duration(2.0 * kPeriod);
      message.points.push_back(std::move(waypoint));
      trajectory_pub_.publish(message);
      return;
    }
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
    else if (sdk_ && !sdk_->motionReady())
      message.data = "WAITING_FOR_SDK";
    else if (!initialized_)
      message.data = "WAITING_FOR_FEEDBACK";
    else if (!enabled_)
      message.data = settled() ? "DISABLED" : "STOPPING";
    else
      message.data = settled() && requested_.norm() < 1e-12 ? "ENABLED_IDLE" : "JOGGING";
    state_pub_.publish(message);
  }

  ros::CallbackQueue feedback_queue_;
  ros::NodeHandle nh_, private_nh_, io_, feedback_nh_;
  ros::Publisher state_pub_, joint_pub_, trajectory_pub_;
  ros::Subscriber command_sub_, feedback_sub_;
  ros::ServiceServer enable_service_, reset_service_;
  ros::ServiceClient controller_list_client_;
  ros::Timer output_timer_;
  ros::WallTimer feedback_timer_, status_timer_;
  std::atomic<bool> control_running_{false};
  std::thread control_thread_;
  std::mutex control_wait_mutex_;
  std::condition_variable control_wakeup_;
  double control_period_{0.02};
  unsigned control_steps_{4};
  JointVelocity control_target_{JointVelocity::Zero()};
  Time last_control_update_{Clock::now()};
  bool control_target_ready_{false};
  std::vector<ros::Publisher> joint_commands_;
  std::string backend_, base_, tool_, group_, joint_topic_, scene_joint_topic_, controller_command_topic_;
  std::string controller_manager_service_, controller_name_;
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
  std::shared_ptr<MotionTimingTrace> timing_trace_;
  std::uint64_t trace_command_id_{0}, trace_target_id_{0}, trace_target_sequence_{0};
  std::vector<jog::VelocitySmoother> filters_;
  std::mutex mutex_;
  // A late controller-status response must not undo a newer stop request.
  std::atomic<std::uint64_t> stop_epoch_{0};
  bool enabled_{false}, neutral_seen_{false}, initialized_{false}, have_feedback_{false};
  bool tool_frame_{false}, capture_orientation_{true}, fatal_{false};
  bool shutting_down_{false}, motion_started_{false};
  Eigen::Vector3d requested_{Eigen::Vector3d::Zero()};
  KDL::Rotation hold_rotation_;
  Time last_feedback_{}, last_command_{}, last_generated_{};
  Time last_feedback_received_{};
  double max_feedback_gap_{0.0}, feedback_transport_age_{0.0}, feedback_lock_wait_{0.0};
  std::uint64_t feedback_samples_{0};
  ros::Time last_stamp_;
  std::string fault_;
  std::unique_ptr<HomeServer> home_server_;
  std::unique_ptr<HomeHandle> home_goal_;
  JointPoint home_position_{};
  double home_velocity_, home_timeout_, feedback_speed_{0.0};
  bool home_stopping_{false}, home_cancelled_{false};
  Time home_started_{};
  std::unique_ptr<ros::AsyncSpinner> feedback_spinner_;
  // 放在最后：构造抛异常时也先销毁 SDK 输出线程，再释放它引用的其他成员。
  std::unique_ptr<visual_servo_internal::DirectSdkBackend> sdk_;
};
}  // namespace aubo_ros_control

int main(int argc, char** argv)
{
  // 保留 spinner 直到平滑停车完成；SIGINT 默认 ros::shutdown 会直接结束 SDK 喂点。
  ros::init(argc, argv, "aubo_cartesian_jog", ros::init_options::NoSigintHandler);
  // 仅设置字符编码；保留ROS数值解析使用的C locale。
  std::setlocale(LC_CTYPE, "");
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
    // 给名义柜内前瞻留出排空余量；MAC 无硬件时间戳，不能保证实际排空或异常时平滑退出。
    if (ros::ok() && jog.healthy())
      ros::WallDuration(0.40).sleep();
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
