#include <gtest/gtest.h>
#include <aubo_mobile_nav_sorting/mission_state.h>
using namespace aubo_mobile_nav_sorting;

TEST(SortingState, ParsesWireDetailAndWhitespace)
{
  EXPECT_EQ(SortingState::READY, parseSortingState("  READY \t | waiting"));
  EXPECT_EQ(SortingState::STOPPED, parseSortingState("STOPPED"));
  EXPECT_EQ(SortingState::ERROR, parseSortingState("ERROR | planner failed"));
}

TEST(SortingState, UnknownInputNeverBecomesReady)
{
  for (const auto *text : {"", "   ", "| READY", "READY_EXTRA", "ready", "NEW_STATE | READY"})
    EXPECT_EQ(SortingState::UNKNOWN, parseSortingState(text));
}

TEST(MissionState, PreservesStopProtocol)
{
  EXPECT_STREQ("STOP_UNCONFIRMED", toString(MissionState::STOP_UNCONFIRMED));
  EXPECT_STREQ("STOPPED", toString(MissionState::STOPPED));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
