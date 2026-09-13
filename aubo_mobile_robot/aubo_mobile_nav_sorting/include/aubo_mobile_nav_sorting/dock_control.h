#pragma once
#include <algorithm>
#include <cmath>

namespace aubo_mobile_nav_sorting
{
struct DockCommand { double linear; double angular; };

// Errors are expressed in the target heading frame. Limit the steering
// correction to stay inside the existing straight-docking yaw corridor.
inline DockCommand dockCommand(double forward, double lateral, double yaw_error,
                               double max_speed, double max_turn, double yaw_limit)
{
  const double steering_limit = std::min(0.04, 0.5 * yaw_limit);
  const double direction = forward >= 0.0 ? 1.0 : -1.0;
  const double steering = std::max(-steering_limit, std::min(steering_limit,
      std::atan2(direction * lateral, std::max(0.15, std::abs(forward)))));
  const double angular = std::max(-max_turn, std::min(max_turn, 2.0 * (yaw_error + steering)));
  const double linear = std::copysign(std::min(max_speed, 0.8 * std::abs(forward)), forward);
  return {linear, angular};
}
}
