#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aubo_ros_control
{
namespace jog
{

// 点动平滑器只依赖标准库，便于脱离 ROS 对启停、反向和离散导数做测试。
// 第一层限制速度变化率，第二层用一阶滤波限制加速度变化率。
// 固定周期 h 下：|v|<=V，|Δv/h|<=A，|Δ²v/h²|<=J。
// 第二层系数 alpha<=J*h/(2*A)，因为第一层加速度始终位于 [-A,A]。
// 这里约束的是下发序列的离散导数；控制柜插值和真机跟随仍需实测。
class VelocitySmoother
{
public:
  VelocitySmoother(double velocity, double acceleration, double jerk, double period)
    : velocity_limit_(velocity), acceleration_limit_(acceleration), period_(period)
  {
    if (!positive(velocity) || !positive(acceleration) || !positive(jerk) || !positive(period))
      throw std::invalid_argument("Invalid jog smoothing limits");
    alpha_ = std::min(1.0, jerk * period / (2.0 * acceleration));
  }

  double step(double target)
  {
    if (!std::isfinite(target))
      target = 0.0;
    target = clamp(target, -velocity_limit_, velocity_limit_);
    ramp_ += clamp(target - ramp_, -acceleration_limit_ * period_, acceleration_limit_ * period_);
    velocity_ += alpha_ * (ramp_ - velocity_);
    return velocity_;
  }

  double velocity() const { return velocity_; }
  bool settled(double tolerance = 1e-6) const
  {
    return std::abs(ramp_) < tolerance && std::abs(velocity_) < tolerance;
  }

  // 零指令下剩余角位移的保守上界，供关节限位提前减速使用。
  double stoppingDistance() const
  {
    const double ramp_time = std::abs(ramp_) / acceleration_limit_;
    return std::abs(ramp_) * (ramp_time + period_) / 2.0 +
           period_ * std::abs(velocity_) / alpha_;
  }

  static double clamp(double x, double low, double high)
  {
    return std::max(low, std::min(high, x));
  }

private:
  static bool positive(double x) { return std::isfinite(x) && x > 0.0; }
  double velocity_limit_, acceleration_limit_, period_, alpha_;
  double ramp_{0.0}, velocity_{0.0};
};

}  // namespace jog
}  // namespace aubo_ros_control
