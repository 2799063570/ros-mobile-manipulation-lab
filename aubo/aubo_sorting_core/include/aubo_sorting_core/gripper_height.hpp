#ifndef AUBO_SORTING_CORE_GRIPPER_HEIGHT_HPP
#define AUBO_SORTING_CORE_GRIPPER_HEIGHT_HPP
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace aubo_sorting_core {
// Absolute TCP Z in target_frame; minimum_clearance is relative to the table.
inline double gripperDescentHeight(double nominal,double table,double minimum_clearance,double approach)
{
  if (!std::isfinite(nominal) || !std::isfinite(table) || !std::isfinite(minimum_clearance) ||
      !std::isfinite(approach) || minimum_clearance<0 || nominal<=table)
    throw std::invalid_argument("invalid grasp TCP/table/minimum height");
  const double height=std::max(nominal,table+minimum_clearance);
  if (!std::isfinite(height) || height>=approach)
    throw std::invalid_argument("minimum grasp TCP Z must remain below approach/lift height");
  return height;
}
}
#endif
