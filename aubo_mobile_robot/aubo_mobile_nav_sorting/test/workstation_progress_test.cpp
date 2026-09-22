#include <aubo_mobile_nav_sorting/workstation_progress.h>
#include <gtest/gtest.h>

using aubo_mobile_nav_sorting::WorkstationProgress;
using aubo_mobile_nav_sorting::advanceWorkstationProgress;

TEST(WorkstationProgress, RecoversFromCurrentSortingStateWhenTransitionsWereMissed)
{
  const std::vector<std::string> ids{"table_1", "table_2", "table_3"};
  std::vector<WorkstationProgress> progress(3, WorkstationProgress::PENDING);
  advanceWorkstationProgress(ids, progress, "SORTING", "workstation 'table_2'");
  EXPECT_EQ(WorkstationProgress::COMPLETE, progress[0]);
  EXPECT_EQ(WorkstationProgress::ACTIVE, progress[1]);
  EXPECT_EQ(WorkstationProgress::PENDING, progress[2]);

  advanceWorkstationProgress(ids, progress, "WORKSTATION_COMPLETE", "table_2");
  EXPECT_EQ(WorkstationProgress::COMPLETE, progress[1]);
  advanceWorkstationProgress(ids, progress, "NAVIGATING", "workstation 'table_3' attempt 1/2");
  EXPECT_EQ(WorkstationProgress::ACTIVE, progress[2]);
}

TEST(WorkstationProgress, PreservesDisabledRowsAndHandlesTerminalStates)
{
  const std::vector<std::string> ids{"table_1", "table_2", "table_3"};
  std::vector<WorkstationProgress> progress{
      WorkstationProgress::PENDING, WorkstationProgress::DISABLED,
      WorkstationProgress::PENDING};
  advanceWorkstationProgress(ids, progress, "CONFIGURING_WORKSTATION", "table_3");
  EXPECT_EQ(WorkstationProgress::COMPLETE, progress[0]);
  EXPECT_EQ(WorkstationProgress::DISABLED, progress[1]);
  EXPECT_EQ(WorkstationProgress::ACTIVE, progress[2]);
  advanceWorkstationProgress(ids, progress, "FAILED", "navigation failed");
  EXPECT_EQ(WorkstationProgress::FAILED, progress[2]);
  advanceWorkstationProgress(ids, progress, "IDLE", "call /nav_sorting/start");
  EXPECT_EQ(WorkstationProgress::PENDING, progress[0]);
  EXPECT_EQ(WorkstationProgress::DISABLED, progress[1]);
  EXPECT_EQ(WorkstationProgress::PENDING, progress[2]);
}

TEST(WorkstationProgress, DistinguishesUnconfirmedStopFromConfirmedStop)
{
  const std::vector<std::string> ids{"table_1"};
  std::vector<WorkstationProgress> progress{WorkstationProgress::ACTIVE};
  advanceWorkstationProgress(ids, progress, "STOP_UNCONFIRMED", "waiting for stop acknowledgement");
  EXPECT_EQ(WorkstationProgress::STOP_UNCONFIRMED, progress[0]);
  advanceWorkstationProgress(ids, progress, "STOPPED", "stop confirmed");
  EXPECT_EQ(WorkstationProgress::STOPPED, progress[0]);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
