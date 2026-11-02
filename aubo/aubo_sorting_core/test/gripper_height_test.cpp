#include <aubo_sorting_core/gripper_height.hpp>
#include <gtest/gtest.h>
#include <limits>
using aubo_sorting_core::gripperDescentHeight;

TEST(GripperHeight, UsesSixCentimetersAboveTheRealTable)
{
  EXPECT_NEAR(gripperDescentHeight(.04535,.00285,.06,.20285),.06285,1e-12);
  EXPECT_NEAR(gripperDescentHeight(.09,.00285,.06,.20285),.09,1e-12);
  EXPECT_NEAR(gripperDescentHeight(.13,.1,0.,.32),.13,1e-12);
}
TEST(GripperHeight, RejectsInvalidDepthAndIncompatibleApproachBeforeAnyMotion)
{
  EXPECT_THROW(gripperDescentHeight(.04535,.00285,.2,.20285),std::invalid_argument);
  EXPECT_THROW(gripperDescentHeight(.04535,.00285,-.01,.20285),std::invalid_argument);
  EXPECT_THROW(gripperDescentHeight(-.01,.00285,.06,.20285),std::invalid_argument);
  EXPECT_THROW(gripperDescentHeight(.04535,.00285,std::numeric_limits<double>::quiet_NaN(),.20285),std::invalid_argument);
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
