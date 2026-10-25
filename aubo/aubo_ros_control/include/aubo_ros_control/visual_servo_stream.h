#pragma once

#include <aubo_ros_control/cartesian_jog_common.h>
#include <aubo_ros_control/visual_servo_common.h>

#include <stdexcept>
#include <utility>

namespace aubo_ros_control
{
namespace visual_servo_internal
{

// Caller owns synchronization. There is one integrated sequence, no position
// FIFO, feedback blending, or state reset when a new velocity target arrives.
class VisualServoStream
{
public:
  static double pointPeriod() { return 0.005; }

  VisualServoStream(const JointPoint& velocity, const JointPoint& acceleration,
                    const JointPoint& jerk, const JointPoint& lower,
                    const JointPoint& upper)
    : lower_(lower), upper_(upper), smoothers_{{
        {velocity[0], acceleration[0], jerk[0], pointPeriod()},
        {velocity[1], acceleration[1], jerk[1], pointPeriod()},
        {velocity[2], acceleration[2], jerk[2], pointPeriod()},
        {velocity[3], acceleration[3], jerk[3], pointPeriod()},
        {velocity[4], acceleration[4], jerk[4], pointPeriod()},
        {velocity[5], acceleration[5], jerk[5], pointPeriod()}}}
  {
    for (std::size_t i = 0; i < kDof; ++i)
    {
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i])
        throw std::invalid_argument("Invalid visual stream joint limits");
    }
  }

  void initialize(const JointPoint& feedback)
  {
    if (initialized_) throw std::logic_error("Cannot reset an active visual stream");
    for (std::size_t i = 0; i < kDof; ++i)
      if (!std::isfinite(feedback[i]) || feedback[i] < lower_[i] || feedback[i] > upper_[i])
        throw std::invalid_argument("Visual stream feedback outside joint limits");
    position_ = feedback;
    initialized_ = true;
  }

  JointPoint step(JointPoint target)
  {
    if (!initialized_) throw std::logic_error("Visual stream has no initial feedback");
    for (double value : target)
      if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite visual velocity target");
    auto candidate = smoothers_;
    JointPoint next{};
    auto integrate = [&] {
      for (std::size_t i = 0; i < kDof; ++i)
        next[i] = position_[i] + candidate[i].step(target[i]) * pointPeriod();
    };
    auto can_stop = [&] {
      for (std::size_t i = 0; i < kDof; ++i)
      {
        // Reserve space on both sides for the complete zero-input tail. Do not
        // clamp positions or zero velocity at a limit: that breaks derivatives.
        const double tail = candidate[i].stoppingDistance();
        if (next[i] - tail < lower_[i] - 1e-10 || next[i] + tail > upper_[i] + 1e-10)
          return false;
      }
      return true;
    };
    integrate();
    braking_ = !can_stop();
    if (braking_)
    {
      target.fill(0.0);
      candidate = smoothers_;
      integrate();
      if (!can_stop()) throw std::runtime_error("Visual stream cannot brake within joint limits");
    }
    smoothers_ = std::move(candidate);
    position_ = next;
    applied_target_ = target;
    return position_;
  }

  bool initialized() const { return initialized_; }
  bool braking() const { return braking_; }
  bool settled() const
  {
    for (const auto& smoother : smoothers_)
      if (!smoother.settled()) return false;
    return true;
  }
  const JointPoint& position() const { return position_; }
  const JointPoint& appliedTarget() const { return applied_target_; }
  JointPoint velocity() const
  {
    JointPoint result{};
    for (std::size_t i = 0; i < kDof; ++i) result[i] = smoothers_[i].velocity();
    return result;
  }

private:
  JointPoint lower_{}, upper_{}, position_{}, applied_target_{};
  std::array<jog::VelocitySmoother, kDof> smoothers_;
  bool initialized_{false}, braking_{false};
};

}  // namespace visual_servo_internal
}  // namespace aubo_ros_control
