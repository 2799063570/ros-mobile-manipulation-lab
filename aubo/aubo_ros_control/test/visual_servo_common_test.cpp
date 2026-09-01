#include <aubo_ros_control/visual_servo_common.h>

#include <gtest/gtest.h>

#include <limits>

namespace vsi = aubo_ros_control::visual_servo_internal;

namespace
{

vsi::JointPoint pointWithValue(double value)
{
  vsi::JointPoint point{};
  point.fill(value);
  return point;
}

TEST(CommandQueue, DropsOldestPointWhenCapacityIsExceeded)
{
  vsi::CommandQueue queue(2);
  queue.push(pointWithValue(1.0));
  queue.push(pointWithValue(2.0));
  queue.push(pointWithValue(3.0));

  vsi::JointPoint output{};
  ASSERT_TRUE(queue.pop(output));
  EXPECT_DOUBLE_EQ(2.0, output[0]);
  ASSERT_TRUE(queue.pop(output));
  EXPECT_DOUBLE_EQ(3.0, output[0]);
  EXPECT_FALSE(queue.pop(output));
}

TEST(CommandQueue, EnforcesMinimumCapacityAndCanBeCleared)
{
  vsi::CommandQueue queue(0);
  queue.push(pointWithValue(1.0));
  queue.push(pointWithValue(2.0));
  EXPECT_EQ(2u, queue.size());

  queue.clear();
  EXPECT_EQ(0u, queue.size());
  vsi::JointPoint output{};
  EXPECT_FALSE(queue.pop(output));
}

TEST(VisualServoCommon, DetectsInvalidJointPoints)
{
  vsi::JointPoint point = pointWithValue(0.0);
  EXPECT_TRUE(vsi::finitePoint(point));

  point[3] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(vsi::finitePoint(point));
  point[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(vsi::finitePoint(point));
}

TEST(VisualServoCommon, ClampsValuesAtBothLimits)
{
  EXPECT_DOUBLE_EQ(-1.0, vsi::clampValue(-2.0, -1.0, 1.0));
  EXPECT_DOUBLE_EQ(0.25, vsi::clampValue(0.25, -1.0, 1.0));
  EXPECT_DOUBLE_EQ(1.0, vsi::clampValue(2.0, -1.0, 1.0));
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
