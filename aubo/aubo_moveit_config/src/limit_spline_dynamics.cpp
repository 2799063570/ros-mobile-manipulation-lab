#include <aubo_moveit_config/limit_spline_dynamics.h>
#include <aubo_moveit_config/spline_dynamics.h>
#include <aubo_moveit_config/straight_path_timing.h>
#include <class_loader/class_loader.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/console.h>

namespace aubo_moveit_config {
void LimitSplineDynamics::initialize(const ros::NodeHandle &nh) {
  nh.param("optimize_straight_paths", optimize_straight_paths_, true);
  nh.param("stationary_start_transition_duration",
           stationary_start_transition_duration_, 0.2);
  if (!std::isfinite(stationary_start_transition_duration_) ||
      stationary_start_transition_duration_ < 0.0 ||
      stationary_start_transition_duration_ > 2.0)
    throw std::invalid_argument(
        "stationary_start_transition_duration must be 0..2 seconds");
}
std::string LimitSplineDynamics::getDescription() const {
  return "Limit executed quintic spline velocity/acceleration/jerk";
}

bool LimitSplineDynamics::adaptAndPlan(
    const PlannerFn &planner,
    const planning_scene::PlanningSceneConstPtr &scene,
    const planning_interface::MotionPlanRequest &request,
    planning_interface::MotionPlanResponse &response,
    std::vector<std::size_t> &added_path_index) const {
  if (!planner(scene, request, response))
    return false;
  if (!response.trajectory_ || response.trajectory_->getWayPointCount() < 2)
    return true;
  auto &trajectory = *response.trajectory_;
  const auto *group = trajectory.getGroup();
  if (!group)
    return fail(response, "trajectory has no joint group");
  const double velocity_scale =
      validScaling(request.max_velocity_scaling_factor);
  const double acceleration_scale =
      validScaling(request.max_acceleration_scaling_factor);
  const auto &model = group->getParentModel();
  const auto &indices = group->getVariableIndexList();
  const auto &names = group->getVariableNames();
  double time_scale = 1.0;
  std::vector<std::array<double, 3>> limits;
  try {
    for (std::size_t j = 0; j < names.size(); ++j) {
      const auto &bounds = model.getVariableBounds(names[j]);
      double jerk = 0.0;
      if (!bounds.velocity_bounded_ || !bounds.acceleration_bounded_ ||
          !ros::param::get("ruckig/" + names[j] + "/jerk_limit", jerk))
        return fail(response, "missing V/A/J limits for " + names[j]);
      const std::array<double, 3> limit{
          {bounds.max_velocity_ * velocity_scale,
           bounds.max_acceleration_ * acceleration_scale, jerk}};
      limits.push_back(limit);
      for (std::size_t i = 1; i < trajectory.getWayPointCount(); ++i) {
        const auto &first = trajectory.getWayPoint(i - 1);
        const auto &last = trajectory.getWayPoint(i);
        if (!first.hasVelocities() || !first.hasAccelerations() ||
            !last.hasVelocities() || !last.hasAccelerations())
          return fail(
              response,
              "expected position/velocity/acceleration boundary states");
        const int index = indices[j];
        const double duration = trajectory.getWayPointDurationFromPrevious(i);
        const auto polynomial = spline_dynamics::quintic(
            first.getVariablePosition(index), first.getVariableVelocity(index),
            first.getVariableAcceleration(index),
            last.getVariablePosition(index), last.getVariableVelocity(index),
            last.getVariableAcceleration(index), duration);
        time_scale =
            std::max(time_scale,
                     spline_dynamics::requiredScale(
                         spline_dynamics::peaks(polynomial, duration), limit));
      }
    }
  } catch (const std::exception &error) {
    return fail(response, error.what());
  }
  if (optimize_straight_paths_) {
    robot_trajectory::RobotTrajectory candidate(trajectory, true);
    std::vector<std::size_t> added;
    try {
      if (retimeStraightPath(candidate, limits, added) &&
          candidate.getDuration() <
              trajectory.getDuration() * time_scale * 0.99) {
        const double lead = addStartTransition(candidate);
        ROS_INFO_NAMED("aubo_moveit_config",
                       "Straight-path S-curve: %.3f s instead of %.3f s "
                       "uniform fallback; exact path, V/A/J checked; "
                       "stationary start transition %.3f s",
                       candidate.getDuration(),
                       trajectory.getDuration() * time_scale + lead, lead);
        trajectory = candidate;
        added_path_index.insert(added_path_index.end(), added.begin(),
                                added.end());
        return true;
      }
    } catch (const std::exception &error) {
      ROS_WARN_NAMED("aubo_moveit_config",
                     "Straight-path timing unavailable (%s); using verified "
                     "spline fallback",
                     error.what());
    }
  }
  if (time_scale <= 1.0 + 1e-9) {
    addStartTransition(trajectory);
    return true;
  }

  // Uniform dilation preserves every joint's interpolated geometric path.
  // It cannot preserve a moving start boundary, so reject that case instead
  // of introducing a velocity discontinuity into an ongoing motion.
  for (int index : indices)
    if (std::fabs(trajectory.getWayPoint(0).getVariableVelocity(index)) >
            1e-6 ||
        std::fabs(trajectory.getWayPoint(0).getVariableAcceleration(index)) >
            1e-6)
      return fail(response,
                  "cannot uniformly retime a nonstationary start state");

  time_scale *= 1.001; // Small numerical margin for ROS duration rounding.
  const double original_duration = trajectory.getDuration();
  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
    trajectory.setWayPointDurationFromPrevious(
        i, trajectory.getWayPointDurationFromPrevious(i) * time_scale);
    auto &state = *trajectory.getWayPointPtr(i);
    for (int index : indices) {
      state.setVariableVelocity(index,
                                state.getVariableVelocity(index) / time_scale);
      state.setVariableAcceleration(index,
                                    state.getVariableAcceleration(index) /
                                        (time_scale * time_scale));
    }
  }
  const double lead = addStartTransition(trajectory);
  ROS_INFO_NAMED("aubo_moveit_config",
                 "Quintic spline dynamics retimed %.3fx: %.3f -> %.3f s (V/A/J "
                 "limits); stationary start transition %.3f s",
                 time_scale, original_duration, trajectory.getDuration(), lead);
  return true;
}

double LimitSplineDynamics::addStartTransition(
    robot_trajectory::RobotTrajectory &trajectory) const {
  if (stationary_start_transition_duration_ == 0.0 || trajectory.empty() ||
      !trajectory.getGroup())
    return 0.0;
  const auto &first = trajectory.getWayPoint(0);
  if (!first.hasVelocities() || !first.hasAccelerations())
    return 0.0;
  for (int index : trajectory.getGroup()->getVariableIndexList())
    if (std::fabs(first.getVariableVelocity(index)) > 1e-6 ||
        std::fabs(first.getVariableAcceleration(index)) > 1e-6)
      return 0.0;
  const double existing = trajectory.getWayPointDurationFromPrevious(0);
  const double added =
      std::max(0.0, stationary_start_transition_duration_ - existing);
  // A future first waypoint lets JTC bridge from its previous hold using the
  // configured interval instead of squeezing feedback rounding error into 5ms.
  // No positions are snapped, and explicit longer start delays are preserved.
  trajectory.setWayPointDurationFromPrevious(0, existing + added);
  return added;
}

double LimitSplineDynamics::validScaling(double scaling) {
  return std::isfinite(scaling) && scaling > 0.0 && scaling <= 1.0 ? scaling
                                                                   : 1.0;
}
bool LimitSplineDynamics::fail(planning_interface::MotionPlanResponse &response,
                               const std::string &reason) {
  ROS_ERROR_NAMED("aubo_moveit_config", "Quintic spline validation failed: %s",
                  reason.c_str());
  response.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
  return false;
}
} // namespace aubo_moveit_config

CLASS_LOADER_REGISTER_CLASS(aubo_moveit_config::LimitSplineDynamics,
                            planning_request_adapter::PlanningRequestAdapter)
