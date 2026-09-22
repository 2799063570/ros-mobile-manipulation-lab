#ifndef AUBO_SORTING_CORE_OBSERVATION_TF_HPP
#define AUBO_SORTING_CORE_OBSERVATION_TF_HPP

#include <tf/tf.h>
#include <cmath>
#include <string>

namespace aubo_sorting_core
{

inline void lookupObservationTransform(const tf::Transformer& transformer,
                                       const std::string& target_frame,
                                       const std::string& source_frame,
                                       const ros::Time& stamp,
                                       bool allow_recent_world_lag,
                                       tf::StampedTransform& transform)
{
  transform.setIdentity();
  if (target_frame == source_frame)
    return;
  try
  {
    transformer.lookupTransform(target_frame, source_frame, stamp, transform);
  }
  catch (const tf::ExtrapolationException&)
  {
    if (!allow_recent_world_lag)
      throw;
    // Gazebo's odom TF can arrive a few milliseconds after the camera frame.
    // Use its latest sample only for fixed workstation geometry, never for the
    // moving camera or gripper, and reject a missing or older sample.
    transformer.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    const double lag = (stamp - transform.stamp_).toSec();
    if (transform.stamp_.isZero() || !std::isfinite(lag) || lag < 0.0 || lag > 0.03)
      throw;
  }
}

}  // namespace aubo_sorting_core

#endif  // AUBO_SORTING_CORE_OBSERVATION_TF_HPP
