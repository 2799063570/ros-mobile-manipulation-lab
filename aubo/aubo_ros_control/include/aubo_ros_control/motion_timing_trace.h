#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <locale>
#include <stdexcept>
#include <string>
#include <vector>

namespace aubo_ros_control
{
namespace visual_servo_internal
{

// Optional instrumentation, never used for motion decisions. Allocate before
// starting threads; append reserves a unique slot without a mutex, allocation
// or file IO. NO reader/dump is allowed until ALL writers have stopped/joined.
// This is a bounded append-only capture, not a motion queue or a live ring.
class MotionTimingTrace
{
public:
  using Clock = std::chrono::steady_clock;
  enum Kind { Command, Target, Point, BatchAppend, SendBegin, SendEnd,
              QueryBegin, QueryEnd };
  struct Event
  {
    Kind kind{Point};
    std::uint64_t id{0}, batch_id{0}, target_id{0}, command_id{0}, query_id{0};
    std::int64_t begin_ns{0}, locked_ns{0}, end_ns{0};
    // ROS stamp has a separate time domain; never subtract it from steady_ns.
    std::uint64_t source_ros_ns{0};
    int count{0}, mac_scalars{-1}, result{0};
    std::array<double, 6> q{}, velocity{}, desired{};
  };

  MotionTimingTrace(const std::string& path, std::size_t capacity)
    : path_(path), events_(checkedCapacity(path, capacity))
  {
    // Open before SDK startup to reject bad paths; keep an explicit incomplete
    // marker if killed by SIGKILL/power loss. Use a NEW path for every capture.
    std::ofstream file(path_, std::ios::out | std::ios::trunc);
    if (!file) throw std::runtime_error("Cannot open timing trace: " + path_);
    file << "# capture_incomplete: CSV is dumped only after all writers stop\n";
  }

  static std::int64_t ns(Clock::time_point t = Clock::now())
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
  }

  void append(const Event& event)
  {
    const auto slot = next_.fetch_add(1, std::memory_order_relaxed);
    if (slot < events_.size()) events_[slot] = event;
  }

  std::size_t droppedAfterJoin() const
  {
    const auto n = next_.load();
    return n > events_.size() ? n - events_.size() : 0;
  }

  bool dumpAfterJoin() const
  {
    std::ofstream file(path_, std::ios::out | std::ios::trunc);
    if (!file) return false;
    file.imbue(std::locale::classic());
    file << "# motion_timing_trace_v1; clock=steady_ns; point_period_s=0.005; dropped="
         << droppedAfterJoin() << "\n";
    file << "kind,id,batch_id,target_id,command_id,query_id,begin_ns,locked_ns,end_ns,source_ros_ns,count,mac_scalars,result";
    for (const auto* prefix : {"q", "v", "desired"})
      for (int i = 0; i < 6; ++i) file << ',' << prefix << i;
    file << '\n' << std::setprecision(17);
    const char* names[] = {"command", "target", "point", "batch_append", "send_begin",
                           "send_end", "query_begin", "query_end"};
    const auto size = std::min(next_.load(), events_.size());
    for (std::size_t i = 0; i < size; ++i)
    {
      const auto& e = events_[i];
      file << names[e.kind] << ',' << e.id << ',' << e.batch_id << ',' << e.target_id
           << ',' << e.command_id << ',' << e.query_id << ',' << e.begin_ns << ','
           << e.locked_ns << ',' << e.end_ns << ',' << e.source_ros_ns << ','
           << e.count << ',' << e.mac_scalars << ',' << e.result;
      for (const auto& values : {e.q, e.velocity, e.desired})
        for (double value : values) file << ',' << value;
      file << '\n';
    }
    file.flush();
    return file.good();
  }

private:
  static std::size_t checkedCapacity(const std::string& path, std::size_t capacity)
  {
    if (path.empty() || path.front() != '/' || capacity == 0 || capacity > 1000000)
      throw std::invalid_argument("timing trace needs an absolute path and 1..1000000 events");
    return capacity;
  }
  std::string path_;
  std::vector<Event> events_;
  std::atomic<std::size_t> next_{0};
};

// Filled by the on-demand generator. In jog there is NO CommandQueue enqueue
// or dequeue: a generated point is appended to the local SDK batch instead.
struct PointTiming
{
  std::uint64_t target_id{0}, command_id{0};
  std::int64_t begin_ns{0}, locked_ns{0}, end_ns{0};
  std::array<double, 6> velocity{}, desired{};
};

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control
