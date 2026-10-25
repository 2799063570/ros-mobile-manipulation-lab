#ifndef AUBO_ROS_CONTROL_SDK_FLOW_CONTROL_H
#define AUBO_ROS_CONTROL_SDK_FLOW_CONTROL_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>

namespace aubo_ros_control
{
namespace visual_servo_internal
{

// Credit accounting with an explicit observation contract. StrictLedger is for
// asynchronous polling: a successful send acknowledges server acceptance, and a diagnosis
// samples the server buffer at some time within its request/response interval.
// These are protocol assumptions, not a hard realtime guarantee from the SDK.
//
// A reply can omit every send pending at request start and every later send.
// Count all those points again, even if the reply actually already includes
// them. Only a new observation returns credits; elapsed time never does.
// SequentialMac instead admits from the latest measured MAC count. Queries and
// sends MUST be serialized; measured occupancy may rise above the previous local
// estimate without exceeding the capacity limit. SDK diagnosis has no sampling
// timestamp or point sequence, so this mode does not certify an instantaneous
// physical backlog bound between observations.
class SdkFlowControl
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  enum class Policy { StrictLedger, SequentialMac };
  enum class Fault
  {
    None, InvalidConfiguration, QueryProtocol, DiagnosisTimeout,
    DiagnosisFailed, Capacity, SendFailed, SendProtocol, Stopped, StartupNotEmpty
  };

  struct QueryToken
  {
    std::uint64_t sequence{0};
    std::uint64_t completed_before_request{0};
    TimePoint started{};
    explicit operator bool() const { return sequence != 0; }
  };

  explicit SdkFlowControl(std::size_t hard_cap_points = 60,
                          double diagnosis_timeout = 0.15,
                          TimePoint now = Clock::now(),
                          Policy policy = Policy::StrictLedger,
                          bool require_motion_arm = false)
    : hard_cap_(hard_cap_points), upper_(hard_cap_points), last_reply_(now), policy_(policy),
      awaiting_arm_(require_motion_arm)
  {
    if (hard_cap_ == 0 || !std::isfinite(diagnosis_timeout) || diagnosis_timeout <= 0.0 ||
        (require_motion_arm && policy != Policy::SequentialMac))
      fault_ = Fault::InvalidConfiguration;
    else
      timeout_ = std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(diagnosis_timeout));
  }

  QueryToken beginQuery(TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!checkFreshness(now))
      return {};
    if (policy_ == Policy::SequentialMac && reserved_ != completed_)
    {
      latch(Fault::SendProtocol);
      return {};
    }
    if (pending_ || sequence_ == std::numeric_limits<std::uint64_t>::max())
    {
      latch(Fault::QueryProtocol);
      return {};
    }
    pending_ = true;
    query_ = QueryToken{++sequence_, completed_, now};
    return query_;
  }

  bool acceptReply(const QueryToken& token, std::size_t raw_scalars, TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!checkFreshness(now))
      return false;
    if (!pending_ || token.sequence != query_.sequence ||
        token.completed_before_request != query_.completed_before_request ||
        token.started != query_.started || now < token.started)
      return latch(Fault::QueryProtocol);
    // Round partial six-joint points upwards; never grant fractional credit.
    const std::size_t measured = raw_scalars / 6 + (raw_scalars % 6 != 0);
    if (measured > hard_cap_)
      return latch(Fault::Capacity);
    if (awaiting_arm_)
    {
      // No motion may have been reserved/submitted in this phase. An unknown
      // nonempty cabinet is NOT a startup timing fluctuation we can ignore.
      if (measured != 0 || reserved_ != 0 || completed_ != 0)
        return latch(Fault::StartupNotEmpty);
      if (now - last_reply_ >= timeout_)
        startup_good_replies_ = 0;
      const bool timely = now - token.started < timeout_;
      startup_good_replies_ = timely ? std::min(3u, startup_good_replies_ + 1) : 0;
      pending_ = false;
      initialized_ = startup_good_replies_ >= 3;
      upper_ = 0;
      last_reply_ = now;
      return true;  // Observed, but NOT usable for motion unless motionReady().
    }
    if (policy_ == Policy::SequentialMac)
    {
      if (reserved_ != completed_ || token.completed_before_request != reserved_)
        return latch(Fault::SendProtocol);
      upper_ = measured;
    }
    else
    {
      if (measured > upper_)
        return latch(Fault::QueryProtocol);
      const std::uint64_t uncertain = reserved_ - token.completed_before_request;
      // Saturation is conservative under the StrictLedger protocol contract:
      // the previous upper bound already includes all uncertain reservations.
      const std::size_t candidate = uncertain > hard_cap_ - measured
          ? hard_cap_ : measured + static_cast<std::size_t>(uncertain);
      upper_ = std::min(upper_, candidate);
    }
    pending_ = false;
    initialized_ = true;
    last_reply_ = now;
    return true;
  }

  // Reserve BEFORE invoking the SDK. The short mutex protects reservations
  // against query-start capture; neither SDK call is made under this mutex.
  bool reserve(std::size_t points, TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!checkFreshness(now) || !initialized_ || awaiting_arm_)
      return false;
    if (policy_ == Policy::SequentialMac && pending_)
      return latch(Fault::SendProtocol);
    if (points > hard_cap_ - upper_ ||
        points > std::numeric_limits<std::uint64_t>::max() - reserved_)
      return latch(Fault::Capacity);
    reserved_ += points;
    upper_ += points;
    return true;
  }

  // Completion changes which sends a future query must count as uncertain;
  // it NEVER returns space to the sender. Failure must permanently latch.
  bool completeSend(std::size_t points)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_ != Fault::None)
      return false;
    if (points > reserved_ - completed_)
      return latch(Fault::SendProtocol);
    completed_ += points;
    return true;
  }

  void sendFailed()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latch(Fault::SendFailed);
  }

  void diagnosisFailed()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latch(Fault::DiagnosisFailed);
  }

  void stop(Fault reason = Fault::Stopped)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latch(reason == Fault::None ? Fault::Stopped : reason);
  }

  bool healthy(TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return checkFreshness(now);
  }

  bool initialized() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
  }

  // Startup availability is separate from connection health. Require three
  // consecutive timely empty replies before allowing an explicit enable/home.
  bool motionReady(TimePoint now) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return motionReadyLocked(now);
  }

  // One-way transition: after ANY accepted motion request, every original
  // watchdog applies, even before the first send. No automatic recovery from
  // a movement fault or re-entry into the startup phase is possible.
  bool armMotion(TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motionReadyLocked(now))
      return false;
    awaiting_arm_ = false;
    return true;
  }

  // StrictLedger: conservative protocol bound. SequentialMac: latest measured
  // occupancy plus reserved points; not a hardware timestamped backlog bound.
  std::size_t upperBound() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return upper_;
  }

  std::size_t inFlight() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<std::size_t>(reserved_ - completed_);
  }

  Fault fault() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_;
  }

private:
  bool motionReadyLocked(TimePoint now) const
  {
    return fault_ == Fault::None && initialized_ && now - last_reply_ < timeout_ &&
           (!pending_ || now - query_.started < timeout_);
  }
  bool latch(Fault value)
  {
    if (fault_ == Fault::None)
      fault_ = value;
    return false;
  }

  bool checkFreshness(TimePoint now)
  {
    if (fault_ != Fault::None)
      return false;
    // While explicitly unarmed there is no trajectory to underrun. A slow RPC
    // revokes readiness (motionReady) instead of killing the disabled node.
    // Safety/RPC failures and invalid/nonempty observations still latch faults.
    if (awaiting_arm_)
      return true;
    // Separate bounds prevent a stuck active request and a dead poller.
    // The receive-time TTL is not a claim that the hardware snapshot is this
    // recent: its true age also includes the preceding request latency.
    // A caller can capture now, be preempted before taking this mutex, and
    // observe a reply recorded by the other thread in the meantime. A negative
    // age then means a newer observation, not a clock fault or extra credit.
    if (now - last_reply_ >= timeout_ ||
        (pending_ && now - query_.started >= timeout_))
      return latch(Fault::DiagnosisTimeout);
    return true;
  }

  mutable std::mutex mutex_;
  const std::size_t hard_cap_;
  std::size_t upper_;
  std::uint64_t reserved_{0}, completed_{0}, sequence_{0};
  Clock::duration timeout_{};
  TimePoint last_reply_;
  const Policy policy_;
  QueryToken query_;
  Fault fault_{Fault::None};
  bool pending_{false}, initialized_{false};
  bool awaiting_arm_{false};
  unsigned startup_good_replies_{0};
};

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control

#endif
