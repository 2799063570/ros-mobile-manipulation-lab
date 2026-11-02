#ifndef AUBO_SORTING_CORE_OBSERVATION_TF_HPP
#define AUBO_SORTING_CORE_OBSERVATION_TF_HPP

#include <tf/tf.h>
#include <cmath>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <thread>
#include <algorithm>

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

struct ObservationTransformRequest
{
  std::string target, source;
  bool allow_recent_world_lag;
  ObservationTransformRequest(const std::string& to,const std::string& from,bool world_lag=false)
      : target(to), source(from), allow_recent_world_lag(world_lag) {}
};

// Wait outside the task's queue mutex. Every check uses the unchanged source
// stamp: moving-camera, arm and gripper transforms never fall back to latest.
inline bool waitForObservationTransforms(const tf::Transformer& transformer,
    const std::vector<ObservationTransformRequest>& requests,const ros::Time& stamp,
    double timeout_s,const std::function<bool()>& cancelled,std::string& error)
{
  using Clock=std::chrono::steady_clock;
  error.clear();
  if (!std::isfinite(timeout_s) || timeout_s<=0 || stamp.isZero()) {
    error="invalid observation TF wait parameters";
    return false;
  }
  const auto deadline=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_s));
  while (true)
  {
    if (cancelled && cancelled()) { error.clear(); return false; }
    bool ready=true;
    for (const auto& request:requests)
    {
      try {
        tf::StampedTransform transform;
        lookupObservationTransform(transformer,request.target,request.source,stamp,
                                   request.allow_recent_world_lag,transform);
      }
      catch (const tf::TransformException& exception) {
        error=exception.what(); ready=false; break;
      }
    }
    if (ready) { error.clear(); return true; }
    const auto now=Clock::now();
    if (now>=deadline) return false;
    std::this_thread::sleep_for(std::min<Clock::duration>(deadline-now,std::chrono::milliseconds(5)));
  }
}

}  // namespace aubo_sorting_core

#endif  // AUBO_SORTING_CORE_OBSERVATION_TF_HPP
