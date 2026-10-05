#include <aubo_mobile_nav_sorting/dock_control.h>
#include <gtest/gtest.h>
#include <cmath>
using aubo_mobile_nav_sorting::dockCommand;

TEST(DockControl, CommandsStayInsideExpectedDirectionsAndBounds)
{
  EXPECT_LT(dockCommand(.4, 0, -.04, .04, .12, .06).angular, 0);
  EXPECT_GT(dockCommand(.4, .01, 0, .04, .12, .06).angular, 0);
  EXPECT_LT(dockCommand(-.3, .01, 0, .04, .12, .06).angular, 0);
  EXPECT_LT(std::abs(dockCommand(.005, 0, 0, .04, .12, .06).linear), .005);
}

TEST(DockControl, ConvergesForwardAndReverseUnderYawDisturbance)
{
  for (double direction : {1., -1.}) {
    double x=0, y=.005, yaw=.01;
    bool reached=false;
    for (int i=0; i<300; ++i) {
      double dx=direction*.4-x, dy=-y;
      if (std::hypot(dx,dy)<=.015 && std::abs(yaw)<=.025) { reached=true; break; }
      auto u=dockCommand(dx,dy,-yaw,.04,.12,.06);
      x+=u.linear*std::cos(yaw)*.05;
      y+=u.linear*std::sin(yaw)*.05;
      yaw+=(u.angular+.008)*.05; // sustained drivetrain yaw disturbance
      ASSERT_LT(std::abs(yaw), .06);
    }
    EXPECT_TRUE(reached);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
