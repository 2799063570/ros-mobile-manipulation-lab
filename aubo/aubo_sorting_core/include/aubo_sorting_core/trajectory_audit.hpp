#ifndef AUBO_SORTING_CORE_TRAJECTORY_AUDIT_HPP
#define AUBO_SORTING_CORE_TRAJECTORY_AUDIT_HPP
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace aubo_sorting_core {
struct TrajectoryAudit {
  bool valid{false};
  std::string reason;
  unsigned int samples{0};
  int segment{-1};
};

// Check the whole robot against the frozen scene and its ACM, including self
// collisions and attached bodies. This is discrete geometric validation only.
inline TrajectoryAudit auditTrajectory(const planning_scene::PlanningScene& scene,
    const moveit_msgs::RobotState& start, const moveit_msgs::RobotTrajectory& trajectory,
    const std::string& group, double step,
    const moveit_msgs::Constraints& constraints = moveit_msgs::Constraints(),
    const std::vector<moveit_msgs::Constraints>& goals = {}) {
  TrajectoryAudit result;
  const auto* jmg = scene.getRobotModel()->getJointModelGroup(group);
  if (!jmg || !std::isfinite(step) || step <= 0.0 || step > 0.1) {
    result.reason = "invalid group or interpolation step (0,0.1] rad"; return result;
  }
  const auto& jt = trajectory.joint_trajectory;
  const auto& variables = jmg->getVariableNames();
  const std::set<std::string> names(jt.joint_names.begin(), jt.joint_names.end());
  if (jt.points.empty() || names.size() != jt.joint_names.size() ||
      names != std::set<std::string>(variables.begin(), variables.end()) ||
      !trajectory.multi_dof_joint_trajectory.points.empty()) {
    result.reason = "empty path, unexpected/duplicate joints or unsupported multi-DOF path"; return result;
  }
  // A full recorded start state is mandatory: do not silently replace a
  // missing historical arm configuration with the current live state.
  if (start.joint_state.name.size() != start.joint_state.position.size()) {
    result.reason = "malformed start state"; return result;
  }
  const std::set<std::string> start_names(start.joint_state.name.begin(), start.joint_state.name.end());
  if (start_names.size() != start.joint_state.name.size()) {
    result.reason = "duplicate start-state joints"; return result;
  }
  for (const auto& name : variables) if (!start_names.count(name)) {
    result.reason = "recorded start state is missing group joints"; return result;
  }
  for (double q : start.joint_state.position) if (!std::isfinite(q)) {
    result.reason = "nonfinite start state"; return result;
  }
  moveit::core::RobotState state(scene.getCurrentState());
  // The frozen scene is authoritative for attached bodies. A joint-only
  // current-state monitor must not erase carried geometry during conversion.
  if (!moveit::core::robotStateMsgToRobotState(start, state, false)) {
    result.reason = "cannot convert recorded start state"; return result;
  }
  state.update();
  auto check = [&]() {
    ++result.samples;
    // Empty group checks all collision pairs, not just moving arm links.
    return state.satisfiesBounds() && scene.isStateValid(state, constraints, "", false);
  };
  if (!check()) { result.reason = "invalid start: bounds/collision/path constraint"; return result; }
  std::vector<double> previous;
  for (const auto& name : jt.joint_names) previous.push_back(state.getVariablePosition(name));
  for (std::size_t i = 0; i < jt.points.size(); ++i) {
    result.segment = static_cast<int>(i);
    const auto& q = jt.points[i].positions;
    if (q.size() != previous.size()) { result.reason = "wrong point dimension"; return result; }
    double squared = 0.0;
    for (std::size_t j = 0; j < q.size(); ++j) {
      if (!std::isfinite(q[j])) { result.reason = "nonfinite point"; return result; }
      squared += (q[j]-previous[j])*(q[j]-previous[j]);
    }
    const double count = std::ceil(std::sqrt(squared)/step);
    if (!std::isfinite(count) || count > 1000000.0 || result.samples + count > 1000000.0) {
      result.reason = "audit sample limit exceeded"; return result;
    }
    const std::size_t n = std::max<std::size_t>(1, static_cast<std::size_t>(count));
    for (std::size_t k = 1; k <= n; ++k) {
      for (std::size_t j = 0; j < q.size(); ++j)
        state.setVariablePosition(jt.joint_names[j], previous[j] + (q[j]-previous[j])*k/n);
      state.update();
      if (!check()) { result.reason = "bounds/collision/path constraint at interpolated sample"; return result; }
    }
    previous = q;
  }
  if (!goals.empty()) {
    bool goal_valid = false;
    for (const auto& goal : goals) goal_valid = goal_valid || scene.isStateConstrained(state, goal);
    if (!goal_valid) { result.reason = "final state violates every goal constraint"; return result; }
  }
  result.valid = true; result.reason = "valid in supplied frozen scene and ACM";
  result.segment = -1; return result;
}
}  // namespace aubo_sorting_core
#endif
