#include <aubo_sorting_core/object_support.hpp>
#include <gtest/gtest.h>
#include <limits>
using aubo_sorting_core::validObjectSupport;

TEST(ObjectSupport, ElevatedCenterRequiresExplicitStackMode)
{
  EXPECT_TRUE(validObjectSupport(.12,.04,.1,.02,false));
  EXPECT_FALSE(validObjectSupport(.20,.04,.1,.02,false));
  EXPECT_TRUE(validObjectSupport(.20,.04,.1,.02,true));
  // Both the queue's tolerance and the grasp's tighter tolerance accept stacks.
  EXPECT_TRUE(validObjectSupport(.20,.04,.1,.04,true));
}

TEST(ObjectSupport, InvalidGeometryAndBuriedBottomRemainRejected)
{
  EXPECT_FALSE(validObjectSupport(.08,.04,.1,.02,true));
  EXPECT_FALSE(validObjectSupport(.2,0,.1,.02,true));
  EXPECT_FALSE(validObjectSupport(.2,.04,.1,-.02,true));
  EXPECT_FALSE(validObjectSupport(std::numeric_limits<double>::quiet_NaN(),.04,.1,.02,true));
  EXPECT_FALSE(validObjectSupport(.2,std::numeric_limits<double>::infinity(),.1,.02,true));
}
