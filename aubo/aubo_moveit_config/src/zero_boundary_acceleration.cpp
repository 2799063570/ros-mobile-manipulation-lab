#include <class_loader/class_loader.hpp>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/console.h>

namespace aubo_moveit_config
{
class ZeroBoundaryAcceleration : public planning_request_adapter::PlanningRequestAdapter
{
public:
  void initialize(const ros::NodeHandle&) override
  {
  }

  std::string getDescription() const override
  {
    return "Set trajectory boundary accelerations to zero";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& request,
                    planning_interface::MotionPlanResponse& response,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    const bool planned = planner(planning_scene, request, response);
    if (!planned || !response.trajectory_ || response.trajectory_->getWayPointCount() < 2)
      return planned;

    const moveit::core::JointModelGroup* group = response.trajectory_->getGroup();
    if (!group)
      return planned;

    const std::vector<int>& variable_indices = group->getVariableIndexList();
    moveit::core::RobotStatePtr first = response.trajectory_->getWayPointPtr(0);
    moveit::core::RobotStatePtr last = response.trajectory_->getWayPointPtr(
        response.trajectory_->getWayPointCount() - 1);
    for (int variable_index : variable_indices)
    {
      first->setVariableAcceleration(variable_index, 0.0);
      last->setVariableAcceleration(variable_index, 0.0);
    }

    ROS_DEBUG_NAMED("aubo_moveit_config", "Set start and end trajectory accelerations to zero before Ruckig smoothing");
    return planned;
  }
};
}  // namespace aubo_moveit_config

CLASS_LOADER_REGISTER_CLASS(aubo_moveit_config::ZeroBoundaryAcceleration,
                            planning_request_adapter::PlanningRequestAdapter)
