#ifndef AUBO_SORTING_CORE_GRIPPER_HEIGHT_HPP
#define AUBO_SORTING_CORE_GRIPPER_HEIGHT_HPP
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace aubo_sorting_core {
// Elevated support is inferred from center and own thickness, not measured
// directly. Positive correction raises the TCP; raw perception stays unchanged.
inline double stackedGraspOffset(double center,double height,double table,double minimum_support,
                                 double correction,bool enabled)
{
  if (!std::isfinite(center) || !std::isfinite(height) || height<=0 || !std::isfinite(table) ||
      !std::isfinite(minimum_support) || minimum_support<.005 || minimum_support>.30 ||
      !std::isfinite(correction) || correction<-.05 || correction>.05)
    throw std::invalid_argument("invalid stacked grasp Z correction/support threshold");
  return enabled && center-height/2-table>=minimum_support ? correction:0.;
}
// Offset from the object's center. Surface mode cancels the thickness prior
// when added back to the depth-derived center; placement keeps the same offset.
inline double graspCenterOffset(double height,double center_offset,double surface_inset,bool use_surface)
{
  if (!std::isfinite(height) || height<=0 || !std::isfinite(center_offset) ||
      !std::isfinite(surface_inset) || surface_inset<0)
    throw std::invalid_argument("invalid object thickness/grasp offset/surface inset");
  return use_surface ? height/2-surface_inset : center_offset;
}
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
