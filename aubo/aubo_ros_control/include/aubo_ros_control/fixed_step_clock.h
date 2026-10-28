#pragma once

#include <chrono>
#include <cstddef>

namespace aubo_ros_control
{
// Controller time advances by exactly one cabinet sample per generated point.
// A late wakeup replays the missing samples rather than skipping along the
// trajectory and compressing a long displacement into one 5 ms cabinet slot.
class FixedStepClock
{
public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    explicit FixedStepClock(Time start) : next_(start + step()) {}

    static std::chrono::milliseconds step() { return std::chrono::milliseconds(5); }
    std::size_t pending(Time now) const
    {
        return now < next_ ? 0 : static_cast<std::size_t>((now - next_) / step()) + 1;
    }
    void advance() { next_ += step(); }
    Time deadline() const { return next_; }

private:
    Time next_;
};
}  // namespace aubo_ros_control
