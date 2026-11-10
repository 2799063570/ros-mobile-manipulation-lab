#include <aubo_sorting_core/gripper_height.hpp>
#include <gtest/gtest.h>
#include <limits>
using aubo_sorting_core::gripperDescentHeight;
using aubo_sorting_core::stackedGraspOffset;

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
TEST(GripperHeight, CorrectionAppliesOnlyToElevatedSupportInDepthStackMode)
{
  EXPECT_DOUBLE_EQ(stackedGraspOffset(.20,.04,.1,.02,.01,true),.01);
  EXPECT_DOUBLE_EQ(stackedGraspOffset(.12,.04,.1,.02,.01,true),0.);
  EXPECT_DOUBLE_EQ(stackedGraspOffset(.20,.04,.1,.10,.01,true),0.);
  EXPECT_DOUBLE_EQ(stackedGraspOffset(.20,.04,.1,.02,.01,false),0.);
  EXPECT_DOUBLE_EQ(stackedGraspOffset(.20,.04,.1,.02,-.01,true),-.01);
  EXPECT_THROW(stackedGraspOffset(.20,.04,.1,.02,.051,true),std::invalid_argument);
  EXPECT_THROW(stackedGraspOffset(.20,.04,.1,0.,.01,true),std::invalid_argument);
  EXPECT_THROW(stackedGraspOffset(.20,.04,.1,.02,std::numeric_limits<double>::quiet_NaN(),true),std::invalid_argument);
  EXPECT_THROW(gripperDescentHeight(.30+stackedGraspOffset(.30,.04,.1,.02,.05,true),.1,.06,.32),std::invalid_argument);
}
int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
