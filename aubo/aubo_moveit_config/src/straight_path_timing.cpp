#include <aubo_moveit_config/spline_dynamics.h>
#include <aubo_moveit_config/straight_path_timing.h>
#include <limits>
#include <ruckig/ruckig.hpp>

namespace aubo_moveit_config {
bool retimeStraightPath(robot_trajectory::RobotTrajectory &trajectory,
                        const std::vector<std::array<double, 3>> &limits,
                        std::vector<std::size_t> &added_path_index) {
  const auto *group = trajectory.getGroup();
  if (!group || trajectory.getWayPointCount() < 2 ||
      limits.size() != group->getVariableCount())
    return false;
  const auto &indices = group->getVariableIndexList();
  if (indices.empty())
    return false;
  const std::size_t count = trajectory.getWayPointCount();
  if (std::fabs(trajectory.getWayPointDurationFromPrevious(0)) > 1e-12)
    return false;
  const auto &first = trajectory.getWayPoint(0);
  const auto &last = trajectory.getWayPoint(count - 1);
  std::vector<double> delta(indices.size());
  std::size_t pivot = 0;
  for (std::size_t j = 0; j < indices.size(); ++j) {
    const int index = indices[j];
    if (!first.hasVelocities() || !first.hasAccelerations() ||
        !last.hasVelocities() || !last.hasAccelerations() ||
        std::fabs(first.getVariableVelocity(index)) > 1e-9 ||
        std::fabs(first.getVariableAcceleration(index)) > 1e-9 ||
        std::fabs(last.getVariableVelocity(index)) > 1e-9 ||
        std::fabs(last.getVariableAcceleration(index)) > 1e-9)
      return false;
    delta[j] =
        last.getVariablePosition(index) - first.getVariablePosition(index);
    if (std::fabs(delta[j]) > std::fabs(delta[pivot]))
      pivot = j;
  }
  if (std::fabs(delta[pivot]) < 1e-8)
    return false;
  const int pivot_index = indices[pivot];
  const double pivot_start = first.getVariablePosition(pivot_index);
  std::vector<double> progress;
  for (std::size_t i = 0; i < count; ++i) {
    const auto &state = trajectory.getWayPoint(i);
    if (!state.hasVelocities() || !state.hasAccelerations())
      return false;
    for (int k : indices)
      if (!std::isfinite(state.getVariablePosition(k)) ||
          !std::isfinite(state.getVariableVelocity(k)) ||
          !std::isfinite(state.getVariableAcceleration(k)))
        return false;
    const double u =
        (state.getVariablePosition(pivot_index) - pivot_start) / delta[pivot];
    if (!std::isfinite(u) || u < -1e-9 || u > 1.0 + 1e-9 ||
        (!progress.empty() && u <= progress.back()))
      return false;
    if (i > 0 && i + 1 < count &&
        state.getVariableVelocity(pivot_index) / delta[pivot] <= 1e-9)
      return false; // Preserve intentionally stopped intermediate waypoints.
    progress.push_back(std::max(0.0, std::min(1.0, u)));
    // Other joints/grippers must not change outside this trajectory group.
    for (std::size_t k = 0; k < first.getVariableCount(); ++k)
      if (std::find(indices.begin(), indices.end(), static_cast<int>(k)) ==
              indices.end() &&
          std::fabs(state.getVariablePosition(k) -
                    first.getVariablePosition(k)) > 1e-9)
        return false;
    for (std::size_t j = 0; j < indices.size(); ++j)
      if (std::fabs(state.getVariablePosition(indices[j]) -
                    first.getVariablePosition(indices[j]) - delta[j] * u) >
          1e-9)
        return false;
    if (i == 0)
      continue;
    const auto &previous = trajectory.getWayPoint(i - 1);
    const double dt = trajectory.getWayPointDurationFromPrevious(i);
    if (!std::isfinite(dt) || dt <= 0.0)
      return false;
    auto scalar = spline_dynamics::quintic(
        previous.getVariablePosition(pivot_index),
        previous.getVariableVelocity(pivot_index),
        previous.getVariableAcceleration(pivot_index),
        state.getVariablePosition(pivot_index),
        state.getVariableVelocity(pivot_index),
        state.getVariableAcceleration(pivot_index), dt);
    scalar[0] -= pivot_start;
    for (double &coefficient : scalar)
      coefficient /= delta[pivot];
    const auto velocity = spline_dynamics::derivative(scalar);
    auto critical =
        spline_dynamics::roots(spline_dynamics::derivative(velocity));
    critical.push_back(0.0);
    critical.push_back(1.0);
    for (double t : critical)
      if (spline_dynamics::evaluate(velocity, t) < -1e-9)
        return false;
    // Bound lateral deviation of the COMPLETE old interpolant, not just knots.
    for (std::size_t j = 0; j < indices.size(); ++j) {
      const int k = indices[j];
      auto lateral = spline_dynamics::quintic(
          previous.getVariablePosition(k), previous.getVariableVelocity(k),
          previous.getVariableAcceleration(k), state.getVariablePosition(k),
          state.getVariableVelocity(k), state.getVariableAcceleration(k), dt);
      lateral[0] -= first.getVariablePosition(k);
      for (std::size_t power = 0; power < lateral.size(); ++power)
        lateral[power] -= delta[j] * scalar[power];
      if (spline_dynamics::maximumMagnitude(lateral) > 1e-9)
        return false;
    }
  }

  ruckig::InputParameter<1> input;
  input.current_position = {0.0};
  input.current_velocity = {0.0};
  input.current_acceleration = {0.0};
  input.target_position = {1.0};
  input.target_velocity = {0.0};
  input.target_acceleration = {0.0};
  input.max_velocity = input.max_acceleration =
      input.max_jerk = {std::numeric_limits<double>::max()};
  for (std::size_t j = 0; j < indices.size(); ++j) {
    for (double limit : limits[j])
      if (!std::isfinite(limit) || limit <= 0.0)
        return false;
    if (std::fabs(delta[j]) < 1e-12)
      continue;
    // A 0.1% margin also accommodates nanosecond timestamp serialization.
    input.max_velocity[0] = std::min(
        input.max_velocity[0], limits[j][0] * 0.999 / std::fabs(delta[j]));
    input.max_acceleration[0] = std::min(
        input.max_acceleration[0], limits[j][1] * 0.999 / std::fabs(delta[j]));
    input.max_jerk[0] =
        std::min(input.max_jerk[0], limits[j][2] * 0.999 / std::fabs(delta[j]));
  }
  ruckig::Ruckig<1> generator;
  ruckig::Trajectory<1> profile;
  const auto result = generator.calculate(input, profile);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
    return false;
  const double duration = profile.get_duration();
  if (!std::isfinite(duration) || duration <= 0.0)
    return false;

  struct Sample {
    double time;
    bool original;
    std::size_t original_index;
  };
  std::vector<Sample> samples;
  for (std::size_t i = 0; i < count; ++i) {
    double lower = 0.0, upper = duration;
    for (int iteration = 0; iteration < 60; ++iteration) {
      const double mid = (lower + upper) * 0.5;
      std::array<double, 1> p, v, a;
      profile.at_time(mid, p, v, a);
      if (p[0] < progress[i])
        lower = mid;
      else
        upper = mid;
    }
    const double t =
        i == 0 ? 0.0 : (i == count - 1 ? duration : (lower + upper) * 0.5);
    samples.push_back({t, true, i});
  }
  const auto profiles = profile.get_profiles();
  for (double t : profiles[0][0].t_sum)
    if (t > 1e-8 && t < duration - 1e-8) {
      const bool exists = std::any_of(
          samples.begin(), samples.end(), [t](const Sample &sample) {
            return std::fabs(sample.time - t) < 1e-8;
          });
      if (!exists)
        samples.push_back({t, false, 0});
    }
  std::sort(samples.begin(), samples.end(),
            [](const Sample &a, const Sample &b) { return a.time < b.time; });
  auto candidate = std::make_shared<robot_trajectory::RobotTrajectory>(
      trajectory.getRobotModel(), group->getName());
  std::vector<std::size_t> added;
  double previous_time = 0.0;
  for (const auto &sample : samples) {
    const double rounded_time = std::round(sample.time * 1e9) * 1e-9;
    const double dt = rounded_time - previous_time;
    if (!candidate->empty() && dt < 1e-5)
      return false;
    std::array<double, 1> p, v, a;
    profile.at_time(sample.time, p, v, a);
    moveit::core::RobotState state(first);
    for (std::size_t j = 0; j < indices.size(); ++j) {
      const int index = indices[j];
      state.setVariablePosition(
          index, sample.original
                     ? trajectory.getWayPoint(sample.original_index)
                           .getVariablePosition(index)
                     : first.getVariablePosition(index) + delta[j] * p[0]);
      state.setVariableVelocity(index, delta[j] * v[0]);
      state.setVariableAcceleration(index, delta[j] * a[0]);
    }
    if (candidate->empty() || sample.original_index == count - 1)
      for (int index : indices) {
        state.setVariableVelocity(index, 0.0);
        state.setVariableAcceleration(index, 0.0);
      }
    if (!sample.original)
      added.push_back(candidate->getWayPointCount());
    candidate->addSuffixWayPoint(state, dt);
    previous_time = rounded_time;
  }
  // This also catches short-segment numerical/serialization errors and an
  // accidental missing jerk boundary. Fall back without changing the input.
  for (std::size_t i = 1; i < candidate->getWayPointCount(); ++i)
    for (std::size_t j = 0; j < indices.size(); ++j) {
      const int k = indices[j];
      const auto &a = candidate->getWayPoint(i - 1);
      const auto &b = candidate->getWayPoint(i);
      const double dt = candidate->getWayPointDurationFromPrevious(i);
      const auto polynomial = spline_dynamics::quintic(
          a.getVariablePosition(k), a.getVariableVelocity(k),
          a.getVariableAcceleration(k), b.getVariablePosition(k),
          b.getVariableVelocity(k), b.getVariableAcceleration(k), dt);
      if (spline_dynamics::requiredScale(spline_dynamics::peaks(polynomial, dt),
                                         limits[j]) > 1.0 + 1e-9)
        return false;
    }
  trajectory = *candidate;
  added_path_index.insert(added_path_index.end(), added.begin(), added.end());
  return true;
}
} // namespace aubo_moveit_config
