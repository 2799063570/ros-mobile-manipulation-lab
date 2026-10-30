#include <gtest/gtest.h>
#include <aubo_sorting_core/placement_height.hpp>
#include <limits>

using aubo_sorting_core::placementHeight;

TEST(PlacementHeight, ClampsOnlyTheReleaseHeight)
{
  EXPECT_DOUBLE_EQ(.18, placementHeight(.06535, .18, .25285));
  EXPECT_DOUBLE_EQ(.21, placementHeight(.21, .18, .25285));
}

TEST(PlacementHeight, UnconfiguredFloorPreservesExistingHeight)
{
  EXPECT_DOUBLE_EQ(-.12, placementHeight(-.12, -std::numeric_limits<double>::infinity(), .25));
}

TEST(PlacementHeight, RejectsReleaseAtOrAboveApproach)
{
  EXPECT_THROW(placementHeight(.06535, .18, .18), std::invalid_argument);
  EXPECT_THROW(placementHeight(.30, .18, .25), std::invalid_argument);
}

TEST(PlacementHeight, RejectsNonfiniteInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(placementHeight(nan, .18, .25), std::invalid_argument);
  EXPECT_THROW(placementHeight(.06, nan, .25), std::invalid_argument);
  EXPECT_THROW(placementHeight(.06, inf, .25), std::invalid_argument);
  EXPECT_THROW(placementHeight(.06, .18, inf), std::invalid_argument);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
