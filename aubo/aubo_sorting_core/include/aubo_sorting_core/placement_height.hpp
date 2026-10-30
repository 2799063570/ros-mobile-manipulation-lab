#ifndef AUBO_SORTING_CORE_PLACEMENT_HEIGHT_HPP
#define AUBO_SORTING_CORE_PLACEMENT_HEIGHT_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aubo_sorting_core
{
// All arguments are absolute TCP Z in target_frame. Negative infinity disables
// the optional floor, preserving existing simulation and mobile-cell behavior.
inline double placementHeight(double nominal, double minimum, double approach)
{
  if (!std::isfinite(nominal) || std::isnan(minimum) || (std::isinf(minimum) && minimum > 0) ||
      !std::isfinite(approach))
    throw std::invalid_argument("placement heights must be finite (or no minimum)");
  const double height = std::max(nominal, minimum);
  if (approach <= height)
    throw std::invalid_argument("preplace TCP Z must exceed the release TCP Z");
  return height;
}
}  // namespace aubo_sorting_core
#endif
