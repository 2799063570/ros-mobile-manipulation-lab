#include "aubo_remote_bridge/remote_core.hpp"
#include <aubo_remote_bridge/CommandStatus.h>
#include <aubo_remote_bridge/NamedPose.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <sensor_msgs/JointState.h>
#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <functional>

namespace abr = aubo_remote_bridge;
static double wallNow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
class GazeboValidation : public ::testing::Test {
 protected:
  void SetUp() override {
    state_sub_ = nh_.subscribe<abr::CommandStatus>("/remote/motion_state", 1, [this](const abr::CommandStatus::ConstPtr& value) { state_ = *value; });
    joint_sub_ = nh_.subscribe<sensor_msgs::JointState>("/remote_validation/aubo_i5/joint_states", 1, [this](const sensor_msgs::JointState::ConstPtr& value) { joints_ = *value; joint_time_ = wallNow(); });
    goal_sub_ = nh_.subscribe<moveit_msgs::MoveGroupActionGoal>("/remote_validation/move_group/goal", 10, [this](const moveit_msgs::MoveGroupActionGoal::ConstPtr&) { ++goals_; });
    client_ = nh_.serviceClient<abr::NamedPose>("/remote/request_named_pose");
    ASSERT_TRUE(wait([this] { return client_.exists() && state_.state == REMOTE_IDLE && joint_time_ > 0; }, 60.));
  }
  static bool wait(const std::function<bool()>& predicate, double limit) {
    const double deadline = wallNow() + limit;
    while (ros::ok() && wallNow() < deadline) { ros::spinOnce(); if (predicate()) return true; ros::WallDuration(.02).sleep(); }
    return false;
  }
  bool request(const std::string& target, uint16_t id) {
    abr::NamedPose call; call.request.target = target; call.request.session = 77; call.request.request_id = id;
    return client_.call(call) && call.response.accepted;
  }
  void checkPose(const std::string& name) {
    robot_model_loader::RobotModelLoader loader("/remote_validation/robot_description");
    ASSERT_TRUE(static_cast<bool>(loader.getModel()));
    moveit::core::RobotState expected(loader.getModel()); expected.setToDefaultValues();
    ASSERT_TRUE(expected.setToDefaultValues(loader.getModel()->getJointModelGroup("aubo_i5"), name));
    ASSERT_LT(wallNow() - joint_time_, .5);
    for (const auto& name : abr::jointNames()) {
      const auto found = std::find(joints_.name.begin(), joints_.name.end(), name);
      ASSERT_NE(found, joints_.name.end());
      EXPECT_NEAR(joints_.position.at(static_cast<std::size_t>(found - joints_.name.begin())), expected.getVariablePosition(name), .03);
    }
  }
  ros::NodeHandle nh_; ros::Subscriber state_sub_, joint_sub_, goal_sub_; ros::ServiceClient client_;
  abr::CommandStatus state_; sensor_msgs::JointState joints_; double joint_time_ = -1; unsigned goals_ = 0;
};

TEST_F(GazeboValidation, HomeZeroDuplicateAndCancel) {
  // 完成的判据是Action结束加实际JointState到位，而不是服务返回accepted。
  ASSERT_TRUE(request("home", 1));
  ASSERT_TRUE(wait([this] { return state_.request_id == 1 && state_.state == REMOTE_SUCCEEDED; }, 65.));
  checkPose("home");
  const unsigned count = goals_;
  ASSERT_TRUE(request("home", 1));               // 相同编号的重试应回放结果，不再次发goal
  const double duplicate_time = wallNow();
  ASSERT_TRUE(wait([duplicate_time] { return wallNow() - duplicate_time > .3; }, 1.));
  EXPECT_EQ(goals_, count);
  ASSERT_TRUE(request("zero", 2));
  ASSERT_TRUE(wait([this] { return state_.request_id == 2 && state_.state == REMOTE_SUCCEEDED; }, 65.));
  checkPose("zero");
  ASSERT_FALSE(request("not_a_pose", 3));
  ASSERT_TRUE(request("home", 4));
  ASSERT_TRUE(wait([this] { return state_.request_id == 4 && state_.state == REMOTE_EXECUTING; }, 20.));
  ASSERT_TRUE(request("cancel", 5));
  ASSERT_TRUE(wait([this] { return state_.request_id == 4 && (state_.state == REMOTE_CANCELED || state_.state == REMOTE_SUCCEEDED); }, 5.));
}
int main(int argc, char** argv) { ros::init(argc, argv, "named_pose_gazebo_test"); ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
