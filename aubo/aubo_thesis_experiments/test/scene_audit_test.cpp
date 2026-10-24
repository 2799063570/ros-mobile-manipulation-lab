#include <gtest/gtest.h>
#include <aubo_thesis_experiments/scene_obstacles.hpp>
#include <aubo_sorting_core/trajectory_audit.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}

namespace {
moveit::core::RobotModelPtr model() {
  const auto urdf = urdf::parseURDF(R"(
    <robot name="test"><link name="base"/><link name="tip">
      <collision><origin xyz="1 0 0"/><geometry><sphere radius="0.05"/></geometry></collision>
    </link>
      <joint name="joint" type="revolute"><parent link="base"/>
        <child link="tip"/><axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint></robot>)");
  auto srdf = std::make_shared<srdf::Model>();
  srdf->initString(*urdf, R"(<robot name="test"><group name="arm">
      <joint name="joint"/></group></robot>)");
  return std::make_shared<moveit::core::RobotModel>(urdf, srdf);
}
}

TEST(SceneObstacles, RotatedBoxAndLimit) {
  planning_scene::PlanningScene scene(model());
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(1, 2, 3);
  pose.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  scene.getWorldNonConst()->addToObject("box", shapes::ShapeConstPtr(new shapes::Box(2, 4, 6)), pose);
  std::vector<double> rows;
  std::string reason;
  ASSERT_TRUE(aubo_thesis_experiments::sceneObstacles(scene, 1, rows, reason));
  ASSERT_EQ(7u, rows.size());
  EXPECT_DOUBLE_EQ(1, rows[1]);
  EXPECT_NEAR(4, rows[4], 1e-9);
  EXPECT_NEAR(2, rows[5], 1e-9);
  EXPECT_NEAR(6, rows[6], 1e-9);
  scene.getWorldNonConst()->addToObject("other", shapes::ShapeConstPtr(new shapes::Sphere(.1)),
                                      Eigen::Isometry3d::Identity());
  EXPECT_FALSE(aubo_thesis_experiments::sceneObstacles(scene, 1, rows, reason));
  EXPECT_FALSE(aubo_thesis_experiments::sceneObstacles(scene, 0, rows, reason));
}

TEST(TrajectoryAudit, ValidPathBoundsAndMissingStart) {
  planning_scene::PlanningScene scene(model());
  moveit_msgs::RobotState start;
  start.joint_state.name = {"joint"};
  start.joint_state.position = {0};
  moveit_msgs::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {"joint"};
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = {.5};
  trajectory.joint_trajectory.points.push_back(point);
  auto result = aubo_sorting_core::auditTrajectory(scene, start, trajectory, "arm", .02);
  EXPECT_TRUE(result.valid) << result.reason;
  EXPECT_GE(result.samples, 26u);
  // Both endpoints are clear; the frozen-scene audit must inspect the arc.
  Eigen::Isometry3d obstacle = Eigen::Isometry3d::Identity();
  obstacle.translation() = Eigen::Vector3d(std::cos(.5), std::sin(.5), 0);
  scene.getWorldNonConst()->addToObject("obstacle", shapes::ShapeConstPtr(new shapes::Sphere(.05)), obstacle);
  trajectory.joint_trajectory.points[0].positions[0] = .9;
  EXPECT_FALSE(aubo_sorting_core::auditTrajectory(scene, start, trajectory, "arm", .02).valid);
  scene.getWorldNonConst()->removeObject("obstacle");
  trajectory.joint_trajectory.points[0].positions[0] = 1.1;
  EXPECT_FALSE(aubo_sorting_core::auditTrajectory(scene, start, trajectory, "arm", .02).valid);
  start.joint_state.name.clear();
  start.joint_state.position.clear();
  EXPECT_FALSE(aubo_sorting_core::auditTrajectory(scene, start, trajectory, "arm", .02).valid);
}
