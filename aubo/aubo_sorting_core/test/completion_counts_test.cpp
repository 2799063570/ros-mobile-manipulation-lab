#include <aubo_sorting_core/completion_counts.hpp>
#include <aubo_sorting_core/instance_queue.hpp>
#include <gtest/gtest.h>
#include <thread>

using aubo_sorting_core::CompletionCounts;

TEST(CompletionCounts, CountsSuccessfulOperationsByCategoryInsteadOfRetainedQueueRows)
{
  CompletionCounts counts;
  counts.configure({"bottle", "can", "box", "can"});
  counts.completed("bottle");
  counts.completed("can"); counts.completed("can");
  counts.completed("box"); counts.completed("box"); counts.completed("box");
  const CompletionCounts::Counts expected{{"bottle",1},{"can",2},{"box",3}};
  EXPECT_EQ(counts.snapshot(), expected);
  // Observation epochs and queue pruning are separate from task-completion totals.
  aubo_sorting_core::InstanceQueue<int> queue;
  queue.clear();
  EXPECT_EQ(counts.snapshot(), expected);
  counts.configure({"bottle", "can", "box", "other"});
  EXPECT_EQ(counts.snapshot().back().second, 0u);
  EXPECT_EQ(counts.snapshot().front().second, 1u);
  CompletionCounts restarted;
  restarted.configure({"bottle", "can", "box"});
  EXPECT_EQ(restarted.snapshot().front().second, 0u);
}

TEST(CompletionCounts, ConcurrentReadersSeeConsistentCategoryCounts)
{
  CompletionCounts counts;
  counts.configure({"can"});
  std::thread first([&] { for(int i=0;i<1000;++i) counts.completed("can"); });
  std::thread second([&] { for(int i=0;i<1000;++i) counts.completed("can"); });
  for(int i=0;i<100;++i) EXPECT_EQ(counts.snapshot().size(), 1u);
  first.join(); second.join();
  EXPECT_EQ(counts.snapshot().front().second, 2000u);
}
