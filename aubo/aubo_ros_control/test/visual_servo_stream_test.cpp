#include <aubo_ros_control/visual_servo_stream.h>
#include <gtest/gtest.h>
#include <limits>

namespace vsi = aubo_ros_control::visual_servo_internal;
vsi::JointPoint filled(double value) { vsi::JointPoint p; p.fill(value); return p; }

TEST(VisualServoStream, ReversalAndStopKeepDiscreteDerivativeBounds)
{
  vsi::VisualServoStream stream(filled(.5), filled(.8), filled(8), filled(-10), filled(10));
  stream.initialize(filled(0));
  auto previous = stream.position();
  auto velocity = filled(0), acceleration = filled(0);
  for (int n = 0; n < 6000; ++n)
  {
    auto target = filled(n < 400 ? -.5 : n < 800 ? .5 : n < 1200 ? -.4 : 0);
    const auto next = stream.step(target);
    for (unsigned i = 0; i < vsi::kDof; ++i)
    {
      const double v = (next[i] - previous[i]) / .005;
      const double a = (v - velocity[i]) / .005;
      const double j = (a - acceleration[i]) / .005;
      EXPECT_LE(std::abs(v), .5 + 1e-8);
      EXPECT_LE(std::abs(a), .8 + 1e-8);
      EXPECT_LE(std::abs(j), 8 + 1e-6);
      velocity[i] = v; acceleration[i] = a;
    }
    previous = next;
  }
  EXPECT_TRUE(stream.settled());
  EXPECT_THROW(stream.initialize(filled(0)), std::logic_error);
}

TEST(VisualServoStream, BrakesAtJointMarginWithoutPositionClamping)
{
  vsi::VisualServoStream stream(filled(.5), filled(.8), filled(8), filled(-.2), filled(.2));
  stream.initialize(filled(0));
  bool braked = false;
  auto previous = stream.position();
  auto velocity = filled(0), acceleration = filled(0);
  for (int n = 0; n < 3000; ++n)
  {
    const auto next = stream.step(filled(.5));
    braked |= stream.braking();
    for (unsigned i = 0; i < vsi::kDof; ++i)
    {
      EXPECT_LE(next[i], .2 + 1e-10);
      const double v = (next[i] - previous[i])/.005;
      const double a = (v - velocity[i])/.005;
      EXPECT_LE(std::abs(a), .8 + 1e-8);
      EXPECT_LE(std::abs((a-acceleration[i])/.005), 8 + 1e-6);
      velocity[i] = v; acceleration[i] = a;
    }
    previous = next;
  }
  EXPECT_TRUE(braked);
}

TEST(VisualServoStream, RejectsUnsafeInitializationAndInvalidTargets)
{
  EXPECT_THROW(vsi::VisualServoStream(filled(.5), filled(.8), filled(0), filled(-1), filled(1)), std::invalid_argument);
  vsi::VisualServoStream stream(filled(.5), filled(.8), filled(8), filled(-1), filled(1));
  EXPECT_THROW(stream.step(filled(0)), std::logic_error);
  EXPECT_THROW(stream.initialize(filled(2)), std::invalid_argument);
  stream.initialize(filled(0));
  EXPECT_THROW(stream.step(filled(std::numeric_limits<double>::quiet_NaN())), std::invalid_argument);
  EXPECT_EQ(stream.position(), filled(0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
