#include <aubo_sorting_core/sorting_task.hpp>
#include <aubo_sorting_core/trajectory_audit.hpp>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace aubo_sorting_core {
bool SortingTask::externalPlan(const moveit_msgs::MotionPlanRequest& request,
    moveit::planning_interface::MoveGroupInterface::Plan& plan) {
  // Explicit service selection; no silent substitution by another planner.
  auto client = nh_.serviceClient<moveit_msgs::GetMotionPlan>(planning_service_);
  moveit_msgs::GetMotionPlan srv; srv.request.motion_plan_request = request;
  if (!client.call(srv) || srv.response.motion_plan_response.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    return false;
  plan.start_state_ = srv.response.motion_plan_response.trajectory_start;
  plan.trajectory_ = srv.response.motion_plan_response.trajectory;
  plan.planning_time_ = srv.response.motion_plan_response.planning_time;
  const auto current = arm_->getCurrentState(2.0);
  if (!current || plan.trajectory_.joint_trajectory.points.empty()) return false;
  // Retiming is independent from the learned planner and uses this robot's limits.
  robot_trajectory::RobotTrajectory rt(arm_->getRobotModel(), group_name_);
  rt.setRobotTrajectoryMsg(*current, plan.trajectory_);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_)) return false;
  rt.getRobotTrajectoryMsg(plan.trajectory_);
  return true;
}
bool SortingTask::revalidatePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit_msgs::MotionPlanRequest& request) {
  return runExperimentStage("collision_audit", "frozen full PlanningScene", [&]() {
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = 1023;
    auto client = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    if (!client.call(srv)) { setFailure("SCENE_AUDIT_FAILED", "cannot fetch PlanningScene"); return false; }
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    if (!scene.setPlanningSceneMsg(srv.response.scene)) {
      setFailure("SCENE_AUDIT_FAILED", "invalid scene snapshot"); return false;
    }
    // Validate the segment from the actual current configuration to the first
    // waypoint too, rather than trusting a possibly stale returned start state.
    const auto current = arm_->getCurrentState(2.0);
    if (!current) { setFailure("SCENE_AUDIT_FAILED", "current robot state unavailable"); return false; }
    moveit_msgs::RobotState start;
    moveit::core::robotStateToRobotStateMsg(*current, start);
    const auto report = auditTrajectory(scene, start, plan.trajectory_, group_name_,
        trajectory_audit_step_, request.path_constraints, request.goal_constraints);
    recordAudit(srv.response.scene, start, plan.trajectory_, request, report.valid, report.reason, report.samples);
    if (!report.valid) setFailure("SCENE_AUDIT_FAILED", report.reason);
    return report.valid;
  });
}
}  // namespace aubo_sorting_core
