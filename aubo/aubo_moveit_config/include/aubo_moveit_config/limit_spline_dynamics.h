#pragma once
#include <moveit/planning_request_adapter/planning_request_adapter.h>

namespace aubo_moveit_config {
class LimitSplineDynamics
    : public planning_request_adapter::PlanningRequestAdapter {
public:
  void initialize(const ros::NodeHandle &) override;
  std::string getDescription() const override;
  bool adaptAndPlan(const PlannerFn &planner,
                    const planning_scene::PlanningSceneConstPtr &scene,
                    const planning_interface::MotionPlanRequest &request,
                    planning_interface::MotionPlanResponse &response,
                    std::vector<std::size_t> &added_path_index) const override;

private:
  bool optimize_straight_paths_{true};
  double stationary_start_transition_duration_{0.2};
  double
  addStartTransition(robot_trajectory::RobotTrajectory &trajectory) const;
  static double validScaling(double scaling);
  static bool fail(planning_interface::MotionPlanResponse &response,
                   const std::string &reason);
};
} // namespace aubo_moveit_config
