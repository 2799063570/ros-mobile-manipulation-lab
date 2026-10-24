#ifndef AUBO_ROS_CONTROL_SDK_IO_DIAGNOSTICS_H
#define AUBO_ROS_CONTROL_SDK_IO_DIAGNOSTICS_H

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

namespace aubo_ros_control
{
namespace visual_servo_internal
{

// Observability only: this helper never grants flow credits or decides faults.
// One sender and one diagnosis poller may update it concurrently. Each has at
// most one active SDK call. Finish records SDK return, not physical execution.
class SdkIoDiagnostics
{
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  struct Event
  {
    bool diagnosis{false};
    std::uint64_t send_id{0};
    std::size_t count{0};  // diagnosis: raw scalars; send: six-axis points
    int result{0};
    double duration_ms{0.0};
    Time finished{};
  };

  struct Snapshot
  {
    bool send_active{false};
    std::uint64_t active_send_id{0}, started_sends{0}, returned_sends{0};
    std::size_t active_send_points{0};
    double active_send_elapsed_ms{0.0}, active_send_max_delta{0.0};
    double active_send_generation_ms{0.0};
    bool last_send_valid{false};
    std::uint64_t last_send_id{0};
    std::size_t last_send_points{0};
    int last_send_result{0};
    double last_send_duration_ms{0.0}, last_send_max_delta{0.0};
    double last_send_generation_ms{0.0};
    bool diagnosis_active{false}, last_diagnosis_valid{false};
    double diagnosis_elapsed_ms{0.0}, last_diagnosis_duration_ms{0.0};
    int last_diagnosis_result{0};
    bool mac_valid{false};
    std::size_t mac_scalars{0};
    // SDK exposes no hardware sampling timestamp. A cached snapshot can predate
    // the request; these two ages describe RPC timing only, not sample-age bounds.
    double mac_response_age_ms{0.0}, mac_request_age_ms{0.0};
    double mac_diagnosis_duration_ms{0.0};
  };

  explicit SdkIoDiagnostics(int success_code = 0) : success_code_(success_code) {}

  std::uint64_t beginSend(std::size_t points, double max_delta,
                          double generation_ms, Time now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.send_active || state_.started_sends == std::numeric_limits<std::uint64_t>::max())
      return 0;
    state_.send_active = true;
    state_.active_send_id = ++state_.started_sends;
    state_.active_send_points = points;
    state_.active_send_max_delta = max_delta;
    state_.active_send_generation_ms = generation_ms;
    send_started_ = now;
    return state_.active_send_id;
  }

  // Call immediately after the SDK returns, including after a watchdog stop.
  // An unknown/repeated ID is ignored instead of corrupting another batch.
  bool finishSend(std::uint64_t id, int result, Time now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.send_active || id != state_.active_send_id)
      return false;
    state_.last_send_valid = true;
    state_.last_send_id = id;
    state_.last_send_points = state_.active_send_points;
    state_.last_send_max_delta = state_.active_send_max_delta;
    state_.last_send_generation_ms = state_.active_send_generation_ms;
    state_.last_send_result = result;
    state_.last_send_duration_ms = elapsed(now, send_started_);
    record(Event{false, id, state_.active_send_points, result, state_.last_send_duration_ms, now});
    ++state_.returned_sends;
    state_.send_active = false;
    state_.active_send_id = 0;
    state_.active_send_points = 0;
    state_.active_send_max_delta = 0.0;
    state_.active_send_generation_ms = 0.0;
    return true;
  }

  bool beginDiagnosis(Time now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.diagnosis_active)
      return false;
    state_.diagnosis_active = true;
    diagnosis_started_ = now;
    return true;
  }

  bool finishDiagnosis(int result, std::size_t mac_scalars, Time now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.diagnosis_active)
      return false;
    state_.last_diagnosis_valid = true;
    state_.last_diagnosis_result = result;
    state_.last_diagnosis_duration_ms = elapsed(now, diagnosis_started_);
    record(Event{true, 0, mac_scalars, result, state_.last_diagnosis_duration_ms, now});
    state_.diagnosis_active = false;
    if (result == success_code_)
    {
      state_.mac_valid = true;
      state_.mac_scalars = mac_scalars;
      state_.mac_diagnosis_duration_ms = state_.last_diagnosis_duration_ms;
      mac_requested_ = diagnosis_started_;
      mac_received_ = now;
    }
    return true;
  }

  Snapshot snapshot(Time now) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Snapshot result = state_;
    if (result.send_active)
      result.active_send_elapsed_ms = elapsed(now, send_started_);
    if (result.diagnosis_active)
      result.diagnosis_elapsed_ms = elapsed(now, diagnosis_started_);
    if (result.mac_valid)
    {
      result.mac_response_age_ms = elapsed(now, mac_received_);
      result.mac_request_age_ms = elapsed(now, mac_requested_);
    }
    return result;
  }

  std::vector<Event> history() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Event> result;
    result.reserve(history_size_);
    const auto begin = (history_next_ + history_.size() - history_size_) % history_.size();
    for (std::size_t i = 0; i < history_size_; ++i)
      result.push_back(history_[(begin + i) % history_.size()]);
    return result;
  }

private:
  // Caller holds mutex_. No allocation or terminal output in the feed loop.
  void record(const Event& event)
  {
    history_[history_next_] = event;
    history_next_ = (history_next_ + 1) % history_.size();
    history_size_ = std::min(history_size_ + 1, history_.size());
  }
  static double elapsed(Time now, Time began)
  {
    // A caller may capture now before this mutex and then be preempted while
    // another thread records a newer event. Clamp that snapshot's negative age.
    return std::max(0.0, std::chrono::duration<double, std::milli>(now - began).count());
  }

  const int success_code_;
  mutable std::mutex mutex_;
  Snapshot state_;
  Time send_started_{}, diagnosis_started_{}, mac_requested_{}, mac_received_{};
  std::array<Event, 64> history_{};
  std::size_t history_next_{0}, history_size_{0};
};

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control

#endif
