#ifndef AUBO_ROS_CONTROL_SDK_JOG_TIMING_H
#define AUBO_ROS_CONTROL_SDK_JOG_TIMING_H

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace aubo_ros_control
{
namespace visual_servo_internal
{
// Query after the previous SDK call returns, then fill the measured deficit.
// MAC diagnosis has no hardware timestamp/sequence and can vary upward between
// replies. Measured space and independent production pacing both limit sends;
// diagnosis/send RPCs must never overlap.
struct SdkJogTiming
{
  explicit SdkJogTiming(int target_scalars)
  {
    if (target_scalars < 24 || target_scalars > 300 || target_scalars % 6 != 0)
      throw std::invalid_argument("SDK jog target must be 24..300 scalars in complete six-axis points");
    target_points = static_cast<std::size_t>(target_scalars / 6);
  }

  std::size_t wanted(std::size_t mac_scalars) const
  {
    const std::size_t observed = mac_scalars / 6 + (mac_scalars % 6 != 0);
    // Leave a 20ms low-water band, avoiding synchronous RPCs for a one-point
    // fluctuation and repeatedly topping up almost-full/cached observations.
    const auto minimum = std::min<std::size_t>(4, target_points / 2);
    return observed <= target_points - minimum ? target_points - observed : 0;
  }
  double lookahead() const { return target_points * 0.005; }
  // A send must return within 100 ms even with a larger cabinet queue.
  double sendBudget() const { return std::min(0.1, lookahead()); }

  std::size_t target_points{0};
};

// Independent producer rate limit. Elapsed time grants permission to PRODUCE
// points, never evidence of cabinet consumption or capacity. Every send still
// needs a nonzero measured deficit and the separate MAC capacity reservation.
class SdkPointPacer
{
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  explicit SdkPointPacer(std::size_t maximum_credit) : maximum_(maximum_credit) {}

  void start(Time now, std::size_t credit = 0)
  {
    started_ = true;
    updated_ = now;
    credit_ = std::min(maximum_, credit);
  }
  bool started() const { return started_; }
  std::size_t allowance(std::size_t deficit, Time now)
  {
    replenish(now);
    const auto points = std::min(deficit, credit_);
    // Batch at least four points unless the complete remaining deficit is smaller.
    return points >= std::min<std::size_t>(4, deficit) ? points : 0;
  }
  bool reserve(std::size_t points, Time now)
  {
    replenish(now);
    if (!started_ || points > credit_) return false;
    credit_ -= points;
    return true;
  }
  void discard(Time now) { start(now); }

private:
  void replenish(Time now)
  {
    if (!started_ || now <= updated_) return;
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - updated_).count();
    const auto points = static_cast<std::size_t>(elapsed / 5000000);
    credit_ += std::min(points, maximum_ - credit_);
    updated_ += std::chrono::nanoseconds(static_cast<std::int64_t>(points) * 5000000);
  }
  const std::size_t maximum_;
  std::size_t credit_{0};
  bool started_{false};
  Time updated_{};
};
}  // namespace visual_servo_internal
}  // namespace aubo_ros_control

#endif
