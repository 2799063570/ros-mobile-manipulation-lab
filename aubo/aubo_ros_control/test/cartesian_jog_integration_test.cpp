#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

class JogIntegration : public testing::Test
{
protected:
  void SetUp() override
  {
    joints_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    commands_ = nh_.advertise<geometry_msgs::TwistStamped>("/cartesian_jog/command", 1);
    state_sub_ = nh_.subscribe<std_msgs::String>("/cartesian_jog/state", 1,
        [this](const std_msgs::String::ConstPtr& message) {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = message->data;
        });
    for (std::size_t i = 0; i < q_.size(); ++i)
      outputs_.push_back(nh_.subscribe<std_msgs::Float64>("/jog_test/q" + std::to_string(i), 1,
          [this, i](const std_msgs::Float64::ConstPtr& message) {
            std::lock_guard<std::mutex> lock(mutex_);
            q_[i] = message->data;
          }));
    enable_ = nh_.serviceClient<std_srvs::SetBool>("/cartesian_jog/set_enabled");
    reset_ = nh_.serviceClient<std_srvs::Trigger>("/cartesian_jog/reset");
    timer_ = nh_.createWallTimer(ros::WallDuration(0.02), [this](const ros::WallTimerEvent&) {
      sensor_msgs::JointState feedback;
      geometry_msgs::TwistStamped command;
      bool send_command, send_feedback;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback.name = {"shoulder_joint", "upperArm_joint", "foreArm_joint",
                         "wrist1_joint", "wrist2_joint", "wrist3_joint"};
        feedback.position.assign(q_.begin(), q_.end());
        command.header.frame_id = frame_;
        command.twist.linear.x = speed_;
        command.twist.angular.z = angular_;
        command.header.stamp = ros::Time::now() - ros::Duration(stale_ ? 1.0 : 0.0);
        send_command = send_command_;
        send_feedback = send_feedback_;
      }
      feedback.header.stamp = ros::Time::now();
      if (send_feedback) joints_.publish(feedback);
      if (send_command) commands_.publish(command);
    });
    ASSERT_TRUE(enable_.waitForExistence(ros::Duration(15)));
    ASSERT_TRUE(wait([this]() { return state() == "DISABLED"; }));
  }

  void TearDown() override
  {
    set(false);
    timer_.stop();
  }

  bool wait(const std::function<bool()>& predicate, double seconds = 4.0)
  {
    const auto until = ros::WallTime::now() + ros::WallDuration(seconds);
    while (ros::ok() && ros::WallTime::now() < until)
    {
      if (predicate()) return true;
      ros::WallDuration(0.01).sleep();
    }
    return false;
  }

  bool set(bool enabled)
  {
    std_srvs::SetBool service;
    service.request.data = enabled;
    return enable_.call(service) && service.response.success;
  }

  bool reset()
  {
    std_srvs::Trigger service;
    return reset_.call(service) && service.response.success;
  }

  std::string state()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  std::array<double, 6> position()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return q_;
  }

  void speed(double value, const std::string& frame = "base_link")
  {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_ = value;
    frame_ = frame;
  }

  void ready()
  {
    speed(0);
    ASSERT_TRUE(set(true));
    ros::WallDuration(0.08).sleep();  // 等待使能后的零指令到达。
  }

  void waitFault(const std::string& reason)
  {
    ASSERT_TRUE(wait([this, &reason]() { return state().find("FAULT: " + reason) == 0; }));
    ASSERT_TRUE(wait([this]() { return state().find("STOPPED") != std::string::npos; }));
    ASSERT_FALSE(set(true));  // 故障不得自动恢复。
  }

  ros::NodeHandle nh_;
  ros::Publisher joints_, commands_;
  ros::Subscriber state_sub_;
  std::vector<ros::Subscriber> outputs_;
  ros::ServiceClient enable_, reset_;
  ros::WallTimer timer_;
  std::mutex mutex_;
  std::array<double, 6> q_{{0, 0, 0, 0, 0, 1.0}};
  std::string state_, frame_{"base_link"};
  double speed_{0}, angular_{0};
  bool send_command_{true}, send_feedback_{true}, stale_{false};
};

TEST_F(JogIntegration, FramesNeutralTimeoutAndFaultLatch)
{
  // 禁用状态下持续非零输入不能运动；使能也必须先经过零指令。
  speed(0.01);
  ros::WallDuration(0.1).sleep();
  EXPECT_NEAR(position()[0], 0, 1e-9);
  ASSERT_TRUE(set(true));
  ros::WallDuration(0.06).sleep();
  EXPECT_NEAR(position()[0], 0, 1e-9);
  speed(0);
  ros::WallDuration(0.06).sleep();
  speed(0.01);
  ASSERT_TRUE(wait([this]() { return position()[0] > 0.002; }));
  EXPECT_NEAR(position()[1], 0, 1e-6);
  ASSERT_TRUE(set(false));
  ASSERT_TRUE(wait([this]() { return state() == "DISABLED"; }));

  // 工具绕 Z 旋转 1 rad，工具 X+ 应同时产生基坐标系 X/Y 正向位移。
  ready();
  const auto start = position();
  speed(0.01, "tcp_link");
  ASSERT_TRUE(wait([this, &start]() { return position()[1] - start[1] > 0.002; }));
  const auto end = position();
  EXPECT_GT(end[1] - start[1], end[0] - start[0]);
  ASSERT_TRUE(set(false));
  ASSERT_TRUE(wait([this]() { return state() == "DISABLED"; }));

  ready();
  speed(0.01);
  ASSERT_TRUE(wait([this]() { return state() == "JOGGING"; }));
  { std::lock_guard<std::mutex> lock(mutex_); send_command_ = false; }
  waitFault("command_timeout");
  const auto stopped = position();
  ros::WallDuration(0.15).sleep();
  EXPECT_NEAR(position()[0], stopped[0], 1e-6);
  { std::lock_guard<std::mutex> lock(mutex_); send_command_ = true; }
  ASSERT_TRUE(reset());

  ready();
  { std::lock_guard<std::mutex> lock(mutex_); stale_ = true; }
  waitFault("invalid_or_stale_command");
  { std::lock_guard<std::mutex> lock(mutex_); stale_ = false; }
  ASSERT_TRUE(reset());

  ready();
  speed(std::numeric_limits<double>::quiet_NaN());
  waitFault("invalid_or_stale_command");
  speed(0);
  ASSERT_TRUE(reset());

  ready();
  speed(0.01, "unknown_frame");
  waitFault("invalid_or_stale_command");
  speed(0);
  ASSERT_TRUE(reset());

  ready();
  { std::lock_guard<std::mutex> lock(mutex_); angular_ = 0.1; }
  waitFault("invalid_or_stale_command");
  { std::lock_guard<std::mutex> lock(mutex_); angular_ = 0; }
  ASSERT_TRUE(reset());

  ready();
  { std::lock_guard<std::mutex> lock(mutex_); send_feedback_ = false; }
  waitFault("feedback_timeout");
  EXPECT_FALSE(reset());
  { std::lock_guard<std::mutex> lock(mutex_); send_feedback_ = true; }
  ASSERT_TRUE(wait([this]() { return reset(); }));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_jog_integration_test");
  testing::InitGoogleTest(&argc, argv);
  ros::AsyncSpinner spinner(3);
  spinner.start();
  return RUN_ALL_TESTS();
}
