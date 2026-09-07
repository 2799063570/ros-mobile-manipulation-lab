#include <gtest/gtest.h>
#include <aubo_sorting_core/height_recovery.hpp>
#include <limits>
using namespace aubo_sorting_core;

TEST(HeightRecovery, BoundsAndBudget)
{
  const auto h = liftHeightCandidates(.26, .18, .02, 5);
  ASSERT_EQ(5u, h.size());
  EXPECT_DOUBLE_EQ(.18, h.back());
  EXPECT_EQ(2u, liftHeightCandidates(.26, .18, .02, 2).size());
  EXPECT_EQ(1u, liftHeightCandidates(.26, .26, .02, 5).size());
  EXPECT_DOUBLE_EQ(.18, liftHeightCandidates(.25, .18, .03, 10).back());
}
TEST(HeightRecovery, RejectsInvalidParameters)
{
  EXPECT_THROW(liftHeightCandidates(.18, .26, .02, 5), std::invalid_argument);
  EXPECT_THROW(liftHeightCandidates(.26, .18, 0, 5), std::invalid_argument);
  EXPECT_THROW(liftHeightCandidates(.26, .18, .02, 0), std::invalid_argument);
  EXPECT_THROW(liftHeightCandidates(std::numeric_limits<double>::quiet_NaN(), .18, .02, 5),
               std::invalid_argument);
}
TEST(HeightRecovery, PlanningFailureRetriesUntilSuccess)
{
  int calls = 0;
  const auto result = runHeightRecovery(liftHeightCandidates(.26, .18, .02, 5),
      [&](double h) {
        ++calls;
        EXPECT_GE(h, .18);
        return calls == 3 ? HeightAttemptResult::Succeeded : HeightAttemptResult::PlanningFailed;
      }, [] { return false; });
  EXPECT_EQ(3, calls);
  EXPECT_EQ(HeightAttemptResult::Succeeded, result);
}
TEST(HeightRecovery, ExecutionFailureNeverRetries)
{
  int calls = 0;
  const auto result = runHeightRecovery(liftHeightCandidates(.26, .18, .02, 5),
      [&](double) { ++calls; return HeightAttemptResult::ExecutionFailed; }, [] { return false; });
  EXPECT_EQ(1, calls);
  EXPECT_EQ(HeightAttemptResult::ExecutionFailed, result);
}
TEST(HeightRecovery, CancellationPreventsNextAttempt)
{
  bool stop = false;
  int calls = 0;
  const auto result = runHeightRecovery(liftHeightCandidates(.26, .18, .02, 5),
      [&](double) { ++calls; stop = true; return HeightAttemptResult::PlanningFailed; },
      [&] { return stop; });
  EXPECT_EQ(1, calls);
  EXPECT_EQ(HeightAttemptResult::Cancelled, result);
}
TEST(HeightRecovery, ExhaustionStopsAtFloor)
{
  int calls = 0;
  const auto result = runHeightRecovery(liftHeightCandidates(.26, .18, .02, 10),
      [&](double h) { ++calls; EXPECT_GE(h, .18); return HeightAttemptResult::PlanningFailed; },
      [] { return false; });
  EXPECT_EQ(5, calls);
  EXPECT_EQ(HeightAttemptResult::PlanningFailed, result);
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
