#include <gtest/gtest.h>
#include <aubo_ros_control/MoveHomeAction.h>
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/ListControllers.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

class RosControlJogIntegration : public testing::Test
{
protected:
  using Joints = std::array<double, 6>;
  struct Pending
  {
    ros::WallTime received, due;
    Joints target;
  };

  void SetUp() override
  {
    std::vector<double> initial;
    if (nh_.getParam("/jog_ros_control_test/initial_positions", initial))
    {
      ASSERT_EQ(initial.size(), q_.size());
      std::copy(initial.begin(), initial.end(), q_.begin());
    }
    raw_ = nh_.advertise<sensor_msgs::JointState>("/jog_ros_control/raw_joint_states", 1);
    bool scene_relay = false;
    nh_.param("/jog_ros_control_test/scene_relay", scene_relay, false);
    cached_ = nh_.advertise<sensor_msgs::JointState>(
        scene_relay ? "/jog_ros_control/cached_joint_states" : "/joint_states", 1);
    nh_.param("/jog_ros_control_test/sdk_points", sdk_points_, false);
    command_ = nh_.advertise<geometry_msgs::TwistStamped>("/cartesian_jog/command", 1);
    controller_service_ = nh_.advertiseService("/aubo_i5/controller_manager/list_controllers",
        &RosControlJogIntegration::listControllers, this);
    output_ = nh_.subscribe("/jog_ros_control/command", 100,
                           &RosControlJogIntegration::trajectory, this,
                           ros::TransportHints().tcpNoDelay());
    if (sdk_points_)
      sdk_output_ = nh_.subscribe("/jog_sdk/point_batches", 50,
          &RosControlJogIntegration::sdkBatch, this, ros::TransportHints().tcpNoDelay());
    status_ = nh_.subscribe<std_msgs::String>("/cartesian_jog/state", 1,
        [this](const std_msgs::String::ConstPtr& message) {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = message->data;
        });
    enable_ = nh_.serviceClient<std_srvs::SetBool>("/cartesian_jog/set_enabled");
    reset_ = nh_.serviceClient<std_srvs::Trigger>("/cartesian_jog/reset");
    plant_timer_ = nh_.createWallTimer(ros::WallDuration(0.002),
        &RosControlJogIntegration::advancePlant, this);
    feedback_timer_ = nh_.createWallTimer(ros::WallDuration(0.02),
        &RosControlJogIntegration::publishInputs, this);
    ASSERT_TRUE(enable_.waitForExistence(ros::Duration(15)));
    ASSERT_TRUE(wait([this] { return state() == "DISABLED"; }));
  }

  void TearDown() override
  {
    releaseControllerQuery();
    set(false);
    feedback_timer_.stop();
    plant_timer_.stop();
  }

  void trajectory(const trajectory_msgs::JointTrajectory::ConstPtr& message)
  {
    const auto received = ros::WallTime::now();
    std::lock_guard<std::mutex> lock(mutex_);
    ++commands_received_;
    if (message->joint_names != names_ || message->points.empty())
    {
      invalid_output_ = "Missing, reordered or incomplete six-joint command";
      return;
    }
    double previous = 0.0;
    for (const auto& point : message->points)
    {
      const double horizon = point.time_from_start.toSec();
      if (point.positions.size() != q_.size() || !std::isfinite(horizon) ||
          horizon <= previous || horizon > (sdk_points_ ? 0.25 : 0.1) ||
          (!point.velocities.empty() && point.velocities.size() != q_.size()) ||
          (!point.accelerations.empty() && point.accelerations.size() != q_.size()))
      {
        invalid_output_ = "Invalid trajectory dimensions or execution horizon";
        return;
      }
      const auto finite = [](double value) { return std::isfinite(value); };
      if (!std::all_of(point.positions.begin(), point.positions.end(), finite) ||
          !std::all_of(point.velocities.begin(), point.velocities.end(), finite) ||
          !std::all_of(point.accelerations.begin(), point.accelerations.end(), finite))
      {
        invalid_output_ = "Nonfinite trajectory target";
        return;
      }
      Joints target;
      std::copy(point.positions.begin(), point.positions.end(), target.begin());
      if (sdk_points_)
      {
        Joints velocity{}, acceleration{};
        for (std::size_t joint = 0; joint < target.size(); ++joint)
        {
          velocity[joint] = (target[joint] - last_setpoint_[joint]) / 0.005;
          acceleration[joint] = (velocity[joint] - last_setpoint_velocity_[joint]) / 0.005;
          const double jerk = (acceleration[joint] - last_setpoint_acceleration_[joint]) / 0.005;
          if ((setpoint_count_ >= 1 && std::abs(velocity[joint]) > (joint < 3 ? 0.25 : 0.35) + 1e-7) ||
              (setpoint_count_ >= 2 && std::abs(acceleration[joint]) > (joint < 3 ? 0.60 : 0.80) + 1e-5) ||
              (setpoint_count_ >= 3 && std::abs(jerk) > (joint < 3 ? 8.0 : 12.0) + 1e-3))
            invalid_output_ = "Direct 5ms points violated velocity/acceleration/jerk bounds";
        }
        last_setpoint_ = target;
        last_setpoint_velocity_ = velocity;
        last_setpoint_acceleration_ = acceleration;
        ++setpoint_count_;
      }
      if (forbid_movement_)
        for (std::size_t i = 0; i < target.size(); ++i)
          if (std::abs(target[i] - forbidden_origin_[i]) > 1e-9)
          {
            ++unexpected_motion_commands_;
            break;
          }
      // The controller's scheduled target is retained in a separate FIFO. It
      // cannot become measured feedback until the extra 100 ms has elapsed.
      const double start_delay = message->header.stamp.isZero() ? 0.0 :
          std::max(0.0, (message->header.stamp - ros::Time::now()).toSec());
      if (sdk_points_)
      {
        // The cabinet consumes accepted points in order; a refill overlaps
        // the initial buffer in wall time, but cannot overtake earlier points.
        sdk_execution_tail_ = std::max(sdk_execution_tail_, received) + ros::WallDuration(0.005);
        pending_.push_back({received, sdk_execution_tail_ + ros::WallDuration(0.1), target});
      }
      else pending_.push_back({received, received + ros::WallDuration(start_delay + horizon + 0.1), target});
      previous = horizon;
    }
  }

  void sdkBatch(const std_msgs::Float64MultiArray::ConstPtr& batch)
  {
    const auto count = batch->data.size() / 6;
    if (batch->data.size() % 6 != 0 || count == 0 || count > 50)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      invalid_output_ = "Unexpected direct SDK batch dimensions";
      return;
    }
    auto command = boost::make_shared<trajectory_msgs::JointTrajectory>();
    command->joint_names = names_;
    for (std::size_t index = 0; index < count; ++index)
    {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions.assign(batch->data.begin() + index * 6, batch->data.begin() + (index + 1) * 6);
      point.time_from_start = ros::Duration((index + 1) * 0.005);
      command->points.push_back(std::move(point));
    }
    trajectory(command);  // Test-side decoding only; no JTC receives this object.
  }

  bool listControllers(controller_manager_msgs::ListControllers::Request&,
                       controller_manager_msgs::ListControllers::Response& response)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (pause_next_controller_query_)
    {
      pause_next_controller_query_ = false;
      controller_query_waiting_ = true;
      // Feedback and the delayed plant remain live while this RPC is held.
      controller_query_release_.wait(lock, [this] { return release_controller_query_; });
      controller_query_waiting_ = false;
    }
    controller_manager_msgs::ControllerState controller;
    controller.name = "aubo_i5_controller";
    controller.type = "position_controllers/JointTrajectoryController";
    controller.state = controller_running_ ? "running" : "stopped";
    response.controller.push_back(controller);
    return true;
  }

  void releaseControllerQuery()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      release_controller_query_ = true;
    }
    controller_query_release_.notify_all();
  }

  void advancePlant(const ros::WallTimerEvent&)
  {
    const auto now = ros::WallTime::now();
    std::lock_guard<std::mutex> lock(mutex_);
    while (!pending_.empty() && pending_.front().due <= now)
    {
      const auto point = pending_.front();
      pending_.pop_front();
      if (!freeze_)
      {
        q_ = point.target;
        min_application_delay_ = std::min(min_application_delay_, (now - point.received).toSec());
        ++points_applied_;
      }
    }
  }

  void publishInputs(const ros::WallTimerEvent&)
  {
    sensor_msgs::JointState feedback;
    geometry_msgs::TwistStamped command;
    ros::Time raw_stamp;
    bool send_command, send_raw;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      feedback.name = names_;
      feedback.position.assign(q_.begin(), q_.end());
      feedback.header.stamp = ros::Time::now();
      if (!freeze_raw_stamp_ || raw_stamp_.isZero())
        raw_stamp_ = feedback.header.stamp;
      raw_stamp = raw_stamp_;
      command.header.stamp = ros::Time::now();
      command.header.frame_id = "base_link";
      if (axis_ == 0) command.twist.linear.x = speed_;
      else if (axis_ == 1) command.twist.linear.y = speed_;
      else command.twist.linear.z = speed_;
      send_command = send_command_;
      send_raw = send_raw_;
      ++cached_samples_;
    }
    // Keep this republisher alive even during a raw-feedback outage. The
    // adapter must not mistake fresh stamps on cached /joint_states for IO.
    cached_.publish(feedback);
    feedback.header.stamp = raw_stamp;
    if (send_raw) raw_.publish(feedback);
    if (send_command) command_.publish(command);
  }

  bool wait(const std::function<bool()>& predicate, double seconds = 4.0)
  {
    const auto deadline = ros::WallTime::now() + ros::WallDuration(seconds);
    while (ros::ok() && ros::WallTime::now() < deadline)
    {
      if (predicate()) return true;
      ros::WallDuration(0.01).sleep();
    }
    return false;
  }

  bool set(bool value)
  {
    std_srvs::SetBool request;
    request.request.data = value;
    return enable_.call(request) && request.response.success;
  }

  bool reset()
  {
    std_srvs::Trigger request;
    return reset_.call(request) && request.response.success;
  }

  std::string state()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  Joints position()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return q_;
  }

  void speed(double value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_ = value;
  }

  void ready()
  {
    speed(0.0);
    ASSERT_TRUE(set(true));
    ASSERT_TRUE(wait([this] { return state() == "ENABLED_IDLE"; }));
    // A status publication may precede the first zero heartbeat.
    ros::WallDuration(0.06).sleep();
  }

  void fault(const std::string& reason)
  {
    ASSERT_TRUE(wait([this, &reason] { return state().find("FAULT: " + reason) == 0; })) << state();
    ASSERT_TRUE(wait([this] { return state().find("STOPPED") != std::string::npos; }));
    EXPECT_FALSE(set(true));
  }

  void assertHeld()
  {
    // Account for already accepted commands and the plant's physical delay.
    ros::WallDuration(sdk_points_ ? 0.45 : 0.25).sleep();
    const auto stopped = position();
    ros::WallDuration(0.2).sleep();
    const auto later = position();
    for (std::size_t i = 0; i < q_.size(); ++i)
      EXPECT_NEAR(later[i], stopped[i], 1e-6) << "joint " << i;
  }

  ros::NodeHandle nh_;
  ros::Publisher raw_, cached_, command_;
  ros::Subscriber output_, status_, sdk_output_;
  ros::ServiceClient enable_, reset_;
  ros::ServiceServer controller_service_;
  ros::WallTimer feedback_timer_, plant_timer_;
  std::mutex mutex_;
  std::condition_variable controller_query_release_;
  const std::vector<std::string> names_{"shoulder_joint", "upperArm_joint", "foreArm_joint",
                                         "wrist1_joint", "wrist2_joint", "wrist3_joint"};
  Joints q_{{0, 0, 0, 0, 0, 1}};
  Joints forbidden_origin_{{0, 0, 0, 0, 0, 1}};
  std::deque<Pending> pending_;
  std::string state_, invalid_output_;
  double speed_{0.0}, min_application_delay_{std::numeric_limits<double>::infinity()};
  std::size_t commands_received_{0}, points_applied_{0}, cached_samples_{0};
  std::size_t unexpected_motion_commands_{0};
  ros::Time raw_stamp_;
  bool send_command_{true}, send_raw_{true}, freeze_{false}, freeze_raw_stamp_{false};
  bool controller_running_{true}, forbid_movement_{false};
  bool pause_next_controller_query_{false}, controller_query_waiting_{false};
  bool release_controller_query_{false};
  int axis_{0};
  bool sdk_points_{false};
  ros::WallTime sdk_execution_tail_;
  Joints last_setpoint_{}, last_setpoint_velocity_{}, last_setpoint_acceleration_{};
  std::size_t setpoint_count_{0};
};

TEST_F(RosControlJogIntegration, RealModelXYZ)
{
  bool real_model = false;
  nh_.param("/jog_ros_control_test/real_model", real_model, false);
  if (!real_model) return;  // Separate launch selects the real-model test only.
  const auto scene_sample = ros::topic::waitForMessage<sensor_msgs::JointState>(
      "/joint_states", nh_, ros::Duration(2.0));
  ASSERT_TRUE(scene_sample);
  for (const auto* name : {"joint1", "joint2"})
  {
    const auto found = std::find(scene_sample->name.begin(), scene_sample->name.end(), name);
    ASSERT_NE(found, scene_sample->name.end());
    const auto index = static_cast<std::size_t>(found - scene_sample->name.begin());
    ASSERT_LT(index, scene_sample->position.size());
    EXPECT_DOUBLE_EQ(scene_sample->position[index], 0.0);
  }
  std::string description;
  ASSERT_TRUE(nh_.getParam("/robot_description", description));
  KDL::Tree tree;
  KDL::Chain chain;
  ASSERT_TRUE(kdl_parser::treeFromString(description, tree));
  ASSERT_TRUE(tree.getChain("base_link", "tcp_link", chain));
  ASSERT_EQ(chain.getNrOfJoints(), 6u);
  KDL::ChainFkSolverPos_recursive fk(chain);
  auto tcp = [&] {
    const auto measured = position();
    KDL::JntArray joints(6);
    for (unsigned i = 0; i < 6; ++i) joints(i) = measured[i];
    KDL::Frame frame;
    EXPECT_GE(fk.JntToCart(joints, frame), 0);
    return frame.p;
  };
  ready();
  double linear_speed = 0.005;
  nh_.param("/jog_ros_control_test/linear_speed", linear_speed, 0.005);
  ASSERT_TRUE(std::isfinite(linear_speed));
  ASSERT_GT(linear_speed, 0.0);
  ASSERT_LE(linear_speed, 0.01);
  for (int axis = 0; axis < 3; ++axis)
    for (const double sign : {1.0, -1.0})
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        axis_ = axis;
      }
      const auto before = tcp();
      speed(sign * linear_speed);
      ros::WallDuration(1.5).sleep();
      speed(0.0);
      ASSERT_TRUE(wait([this] { return state() == "ENABLED_IDLE"; })) << state();
      ros::WallDuration(0.2).sleep();
      const auto moved = tcp() - before;
      EXPECT_GT(sign * moved(axis), linear_speed * 0.6) << "axis " << axis << " sign " << sign;
      EXPECT_LT(sign * moved(axis), linear_speed * 2.4);
      for (int other = 0; other < 3; ++other)
        if (other != axis) EXPECT_LT(std::abs(moved(other)), 0.001);
    }
  ASSERT_TRUE(set(false));
  ASSERT_TRUE(wait([this] { return state() == "DISABLED"; }));
  assertHeld();
  // Cached /joint_states stays live, but real raw samples now stop.
  ready();
  { std::lock_guard<std::mutex> lock(mutex_); send_raw_ = false; }
  fault(sdk_points_ ? "sdk_feedback_failed" : "feedback_timeout");
  EXPECT_FALSE(reset());
  { std::lock_guard<std::mutex> lock(mutex_); send_raw_ = true; }
  ASSERT_TRUE(wait([this] { return reset(); }));
  if (sdk_points_)
  {
    using HomeClient = actionlib::SimpleActionClient<aubo_ros_control::MoveHomeAction>;
    HomeClient home("/cartesian_jog/move_home", true);
    ASSERT_TRUE(home.waitForServer(ros::Duration(3)));
    const auto before = position();
    home.sendGoal(aubo_ros_control::MoveHomeGoal());
    ASSERT_TRUE(wait([this, &before] { return position()[0] > before[0] + 0.002; }));
    home.cancelGoal();
    ASSERT_TRUE(home.waitForResult(ros::Duration(5)));
    EXPECT_EQ(home.getState(), actionlib::SimpleClientGoalState::PREEMPTED);
    ASSERT_TRUE(wait([this] { return state() == "DISABLED"; }));
    home.sendGoal(aubo_ros_control::MoveHomeGoal());
    ASSERT_TRUE(home.waitForResult(ros::Duration(10)));
    EXPECT_EQ(home.getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
    ASSERT_TRUE(wait([this] { return state() == "DISABLED"; }));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(invalid_output_.empty()) << invalid_output_;
    EXPECT_GT(setpoint_count_, 1000u);
  }
}

TEST_F(RosControlJogIntegration, DelayedPlantHomeAndFaultLifecycle)
{
  using HomeClient = actionlib::SimpleActionClient<aubo_ros_control::MoveHomeAction>;
  HomeClient home("/cartesian_jog/move_home", true);
  ASSERT_TRUE(home.waitForServer(ros::Duration(5)));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    controller_running_ = false;
    forbid_movement_ = true;
    forbidden_origin_ = q_;
  }
  EXPECT_FALSE(set(true));
  home.sendGoal(aubo_ros_control::MoveHomeGoal());
  ASSERT_TRUE(home.waitForResult(ros::Duration(3)));
  EXPECT_EQ(home.getState(), actionlib::SimpleClientGoalState::REJECTED);
  assertHeld();
  EXPECT_EQ(state(), "DISABLED");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(unexpected_motion_commands_, 0u);
    forbid_movement_ = false;
    controller_running_ = true;
  }

  // A delayed readiness lookup must not resurrect an enable request after a
  // newer disable has completed. Use a separate client so disable is not
  // serialized behind the stalled request by a shared service connection.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pause_next_controller_query_ = true;
    release_controller_query_ = false;
    forbid_movement_ = true;
    forbidden_origin_ = q_;
  }
  auto pending_enable = std::async(std::launch::async, [this] {
    auto client = nh_.serviceClient<std_srvs::SetBool>("/cartesian_jog/set_enabled");
    std_srvs::SetBool request;
    request.request.data = true;
    return client.call(request) && request.response.success;
  });
  const bool query_blocked = wait([this] {
    std::lock_guard<std::mutex> lock(mutex_);
    return controller_query_waiting_;
  });
  auto pending_disable = std::async(std::launch::async, [this] {
    auto client = nh_.serviceClient<std_srvs::SetBool>("/cartesian_jog/set_enabled");
    std_srvs::SetBool request;
    request.request.data = false;
    return client.call(request) && request.response.success;
  });
  const bool disabled_before_reply =
      pending_disable.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
  // Release on every path before an assertion can unwind the async future.
  releaseControllerQuery();
  EXPECT_TRUE(query_blocked);
  EXPECT_TRUE(disabled_before_reply);
  ASSERT_EQ(pending_disable.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_TRUE(pending_disable.get());
  ASSERT_EQ(pending_enable.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_FALSE(pending_enable.get());
  assertHeld();
  EXPECT_EQ(state(), "DISABLED");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(unexpected_motion_commands_, 0u);
    forbid_movement_ = false;
  }
  ASSERT_TRUE(set(false));

  speed(0.03);
  ros::WallDuration(0.25).sleep();
  EXPECT_NEAR(position()[0], 0.0, 1e-9);  // Disabled input cannot move the plant.
  ASSERT_TRUE(set(true));
  ros::WallDuration(0.05).sleep();
  EXPECT_NEAR(position()[0], 0.0, 1e-9);  // Nonzero input cannot bypass neutral.
  speed(0.0);
  ros::WallDuration(0.08).sleep();
  speed(0.03);
  ASSERT_TRUE(wait([this] { return position()[0] > 0.003; }));
  ASSERT_TRUE(set(false));
  ASSERT_TRUE(wait([this] { return state() == "DISABLED"; }));
  assertHeld();

  const auto before_home = position();
  home.sendGoal(aubo_ros_control::MoveHomeGoal());
  ASSERT_TRUE(wait([this, &before_home] { return position()[0] > before_home[0] + 0.005; }));
  EXPECT_FALSE(set(true));
  home.cancelGoal();
  ASSERT_TRUE(home.waitForResult(ros::Duration(5)));
  EXPECT_EQ(home.getState(), actionlib::SimpleClientGoalState::PREEMPTED);
  ASSERT_TRUE(home.getResult());
  EXPECT_FALSE(home.getResult()->success);
  ASSERT_TRUE(wait([this] { return state() == "DISABLED"; }));
  assertHeld();

  home.sendGoal(aubo_ros_control::MoveHomeGoal());
  ASSERT_TRUE(home.waitForResult(ros::Duration(10)));
  EXPECT_EQ(home.getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  ASSERT_TRUE(home.getResult());
  EXPECT_TRUE(home.getResult()->success);
  EXPECT_NEAR(position()[0], 0.08, 0.005);
  ASSERT_TRUE(wait([this] { return state() == "DISABLED"; }));
  assertHeld();

  ready();
  const auto before_timeout = position();
  speed(0.03);
  ASSERT_TRUE(wait([this, &before_timeout] { return position()[0] > before_timeout[0] + 0.002; }));
  { std::lock_guard<std::mutex> lock(mutex_); send_command_ = false; }
  fault("command_timeout");
  assertHeld();
  { std::lock_guard<std::mutex> lock(mutex_); speed_ = 0.0; send_command_ = true; }
  ASSERT_TRUE(wait([this] { return reset(); }));

  ready();
  const auto before_feedback = position();
  speed(0.03);
  ASSERT_TRUE(wait([this, &before_feedback] { return position()[0] > before_feedback[0] + 0.002; }));
  std::size_t cached_before;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    send_raw_ = false;
    cached_before = cached_samples_;
  }
  fault("feedback_timeout");
  EXPECT_FALSE(reset());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GT(cached_samples_, cached_before + 5);  // Cached stamps did not mask the outage.
    speed_ = 0.0;
    send_raw_ = true;
  }
  ASSERT_TRUE(wait([this] { return reset(); }));
  assertHeld();

  ready();
  const auto before_replay = position();
  speed(0.03);
  ASSERT_TRUE(wait([this, &before_replay] { return position()[0] > before_replay[0] + 0.002; }));
  { std::lock_guard<std::mutex> lock(mutex_); freeze_raw_stamp_ = true; }
  // Receiving a stream of old timestamps must not renew feedback freshness.
  // Timer order may detect the age in the callback or the generation loop.
  ASSERT_TRUE(wait([this] {
    const auto current = state();
    return current.find("FAULT: ") == 0 && current.find("feedback") != std::string::npos;
  })) << state();
  ASSERT_TRUE(wait([this] { return state().find("STOPPED") != std::string::npos; }));
  EXPECT_FALSE(set(true));
  EXPECT_FALSE(reset());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_ = 0.0;
    freeze_raw_stamp_ = false;
  }
  ASSERT_TRUE(wait([this] { return reset(); }));
  assertHeld();

  ready();
  { std::lock_guard<std::mutex> lock(mutex_); freeze_ = true; }
  speed(0.05);
  fault("following_error");  // Fresh but stuck measured joints must not pass as healthy.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_ = 0.0;
    freeze_ = false;
  }
  ASSERT_TRUE(wait([this] { return reset(); }));
  assertHeld();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(invalid_output_.empty()) << invalid_output_;
    EXPECT_GT(commands_received_, 100u);
    EXPECT_GT(points_applied_, 100u);
    EXPECT_GE(min_application_delay_, 0.1);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_jog_ros_control_integration_test");
  testing::InitGoogleTest(&argc, argv);
  ros::AsyncSpinner spinner(3);
  spinner.start();
  return RUN_ALL_TESTS();
}
