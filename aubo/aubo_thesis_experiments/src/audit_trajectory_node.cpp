#include <aubo_thesis_experiments/AuditTrajectory.h>
#include <aubo_thesis_experiments/SnapshotScene.h>
#include <aubo_sorting_core/trajectory_audit.hpp>
#include <aubo_thesis_experiments/scene_obstacles.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <ros/ros.h>
#include <chrono>
#include <stdexcept>

class Auditor {
 public:
  Auditor() : loader_("robot_description") {
    ros::NodeHandle nh, pnh("~");
    std::string scene_service;
    pnh.param<std::string>("planning_scene_service", scene_service, "/get_planning_scene");
    client_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(scene_service);
    server_ = nh.advertiseService("/thesis_experiment/audit_trajectory", &Auditor::audit, this);
    snapshot_server_ = nh.advertiseService("/thesis_experiment/snapshot_scene", &Auditor::snapshot, this);
    if (!loader_.getModel()) throw std::runtime_error("robot_description unavailable");
  }
 private:
  bool snapshot(aubo_thesis_experiments::SnapshotScene::Request& req,
                aubo_thesis_experiments::SnapshotScene::Response& res) {
    try {
      moveit_msgs::GetPlanningScene srv; srv.request.components.components = 1023;
      if (!client_.call(srv)) { res.reason = "cannot obtain complete PlanningScene"; return true; }
      planning_scene::PlanningScene scene(loader_.getModel());
      if (!scene.setPlanningSceneMsg(srv.response.scene)) { res.reason = "invalid PlanningScene"; return true; }
      res.success = aubo_thesis_experiments::sceneObstacles(scene, req.max_obstacles, res.obs_flattened, res.reason);
      res.frame_id = scene.getPlanningFrame(); res.scene = srv.response.scene;
    } catch (const std::exception& error) { res.success = false; res.reason = error.what(); }
    return true;
  }
  bool audit(aubo_thesis_experiments::AuditTrajectory::Request& req,
             aubo_thesis_experiments::AuditTrajectory::Response& res) {
    const auto begin = std::chrono::steady_clock::now();
    res.valid = false; res.invalid_segment = -1;
    try {
      moveit_msgs::GetPlanningScene srv;
      srv.request.components.components = 1023; // all ROS1 PlanningSceneComponents bits
      const bool fetched = req.use_provided_scene || client_.call(srv);
      if (!fetched) res.reason = "cannot obtain complete PlanningScene snapshot";
      else {
        planning_scene::PlanningScene scene(loader_.getModel());
        const auto& message = req.use_provided_scene ? req.scene : srv.response.scene;
        if (message.is_diff) res.reason = "a complete scene is required, not a diff";
        else if (!scene.setPlanningSceneMsg(message)) res.reason = "invalid PlanningScene snapshot";
        else {
          const auto report = aubo_sorting_core::auditTrajectory(scene, req.start_state,
              req.trajectory, req.group_name, req.max_joint_step, req.path_constraints, req.goal_constraints);
          res.valid = report.valid; res.reason = report.reason;
          res.samples_checked = report.samples; res.invalid_segment = report.segment;
          res.scene_name = scene.getName();
        }
      }
    } catch (const std::exception& error) { res.reason = error.what(); }
    res.elapsed_wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    return true;
  }
  robot_model_loader::RobotModelLoader loader_;
  ros::ServiceClient client_;
  ros::ServiceServer server_;
  ros::ServiceServer snapshot_server_;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "thesis_trajectory_auditor");
  try { Auditor auditor; ros::spin(); }
  catch (const std::exception& error) { ROS_FATAL_STREAM(error.what()); return 1; }
  return 0;
}
