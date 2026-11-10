#ifndef AUBO_SORTING_CORE_OBJECT_SUPPORT_HPP
#define AUBO_SORTING_CORE_OBJECT_SUPPORT_HPP
#include <cmath>

namespace aubo_sorting_core {
// All Z values are in target_frame. An elevated support is allowed only when
// explicitly enabled; a bottom below the table remains invalid in either mode.
inline bool validObjectSupport(double center, double height, double table,
                               double tolerance, bool allow_stacked)
{
  if (!std::isfinite(center) || !std::isfinite(height) || height <= 0 ||
      !std::isfinite(table) || !std::isfinite(tolerance) || tolerance < 0)
    return false;
  const double support_offset = center - height / 2 - table;
  return std::isfinite(support_offset) && (allow_stacked ? support_offset >= -tolerance :
                                                        std::abs(support_offset) <= tolerance);
}
}
#endif
