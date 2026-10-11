#include <aubo_sorting_core/sorting_task.hpp>
#include <aubo_sorting_core/ExperimentAudit.h>
#include "task_utils.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>

namespace {
thread_local std::vector<std::uint64_t> spans;
thread_local std::string attempt_id;
}
namespace aubo_sorting_core {
void SortingTask::experimentEvent(const std::string& kind, const std::string& stage,
    const std::string& description, const std::string& status, double elapsed,
    std::uint64_t span, std::uint64_t parent) {
  if (!experiment_enabled_) return;
  const auto quote = [](const std::string& s) { return "\"" + detail::jsonEscape(s) + "\""; };
  std::ostringstream out;
  out << std::setprecision(17) << "{\"schema_version\":1,\"kind\":" << quote(kind)
      << ",\"run_id\":" << quote(experiment_run_id_)
      << ",\"condition\":" << quote(experiment_condition_)
      << ",\"scene_id\":" << quote(experiment_scene_id_)
      << ",\"attempt_id\":" << quote(attempt_id)
      << ",\"instance_id\":" << active_instance_id_
      << ",\"span_id\":" << span << ",\"parent_span_id\":" << parent
      << ",\"stage\":" << quote(stage) << ",\"description\":" << quote(description)
      << ",\"status\":" << quote(status) << ",\"elapsed_s\":" << elapsed
      << ",\"ros_stamp\":" << ros::Time::now().toSec()
      << ",\"wall_stamp\":" << ros::WallTime::now().toSec() << "}";
  std_msgs::String message; message.data = out.str(); experiment_publisher_.publish(message);
}
bool SortingTask::runExperimentStage(const std::string& stage, const std::string& description,
    const std::function<bool()>& operation) {
  if (!experiment_enabled_) return operation();
  const auto id = ++experiment_sequence_;
  const auto parent = spans.empty() ? 0 : spans.back();
  const auto begin = std::chrono::steady_clock::now();
  experimentEvent("stage_begin", stage, description, "running", 0.0, id, parent);
  spans.push_back(id);
  bool success = false;
  try { success = operation(); }
  catch (...) {
    spans.pop_back();
    experimentEvent("stage_end", stage, description, "exception",
        std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count(), id, parent);
    throw;
  }
  spans.pop_back();
  experimentEvent("stage_end", stage, description, success ? "success" : "failure",
      std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count(), id, parent);
  return success;
}
bool SortingTask::pickAndPlace(const aubo_perception::DetectedObject& detected) {
  if (!experiment_enabled_) return pickAndPlaceImpl(detected);
  const std::string previous = attempt_id;
  attempt_id = experiment_run_id_ + ":" + std::to_string(++experiment_sequence_);
  const std::string category = objectCategory(detected);
  experimentEvent("attempt_begin", "pick_place", category, "running");
  try {
    const bool ok = runExperimentStage("pick_place", category, [&] { return pickAndPlaceImpl(detected); });
    // Controller return only: actual placement/classification needs independently
    // annotated ground truth and must not be inferred from action completion.
    experimentEvent("attempt_end", "pick_place", category, ok ? "controller_success" : "controller_failure");
    attempt_id = previous; return ok;
  } catch (...) {
    experimentEvent("attempt_end", "pick_place", category, "exception");
    attempt_id = previous; throw;
  }
}
bool SortingTask::planAndExecute(const std::string& description, double limit) {
  return runExperimentStage("joint_motion", description, [&] { return planAndExecuteImpl(description, limit); });
}
bool SortingTask::cartesianTo(const geometry_msgs::PoseStamped& pose, const std::string& description, bool complete) {
  return runExperimentStage("cartesian_motion", description, [&] { return cartesianToImpl(pose, description, complete); });
}
bool SortingTask::commandGripper(double position) {
  return runExperimentStage("gripper", std::to_string(position), [&] { return commandGripperImpl(position); });
}
bool SortingTask::refreshOctomap() {
  return runExperimentStage("scene_refresh", "OctoMap", [&] { return refreshOctomapImpl(); });
}
bool SortingTask::waitForObject(const std::string& category, const ros::WallTime& begin,
    aubo_perception::DetectedObject& detected) {
  return runExperimentStage("perception_wait", category, [&] { return waitForObjectImpl(category, begin, detected); });
}
void SortingTask::recordPlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
  if (!experiment_enabled_) return;
  moveit_msgs::DisplayTrajectory message;
  message.model_id = arm_->getRobotModel()->getName();
  message.trajectory_start = plan.start_state_;
  message.trajectory.push_back(plan.trajectory_);
  experiment_trajectory_publisher_.publish(message);
}
void SortingTask::recordAudit(const moveit_msgs::PlanningScene& scene, const moveit_msgs::RobotState& start,
    const moveit_msgs::RobotTrajectory& trajectory, const moveit_msgs::MotionPlanRequest& request,
    bool valid, const std::string& reason,
    unsigned int samples) {
  if (!experiment_enabled_) return;
  aubo_sorting_core::ExperimentAudit message;
  message.header.stamp = ros::Time::now();
  message.record_id = ++experiment_sequence_;
  message.run_id = experiment_run_id_; message.condition = experiment_condition_;
  message.scene_id = experiment_scene_id_; message.attempt_id = attempt_id;
  message.group_name = group_name_;
  message.scene = scene; message.start_state = start; message.trajectory = trajectory;
  message.path_constraints = request.path_constraints; message.goal_constraints = request.goal_constraints;
  message.max_joint_step = trajectory_audit_step_;
  message.valid = valid; message.reason = reason; message.samples_checked = samples;
  experiment_audit_publisher_.publish(message);
}
}  // namespace aubo_sorting_core
