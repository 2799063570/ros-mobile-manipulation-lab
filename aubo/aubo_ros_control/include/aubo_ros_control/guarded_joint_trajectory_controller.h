#pragma once

#include <aubo_ros_control/aubo_hardware_interface.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

namespace aubo_ros_control
{
// Keep upstream interpolation/tolerances, but do not mistake action ABORTED for
// a physical stop. The stock Noetic JTC leaves its trajectory running on error.
class GuardedJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController<
    trajectory_interface::QuinticSplineSegment<double>, hardware_interface::PositionJointInterface>
{
    using Base = joint_trajectory_controller::JointTrajectoryController<
        trajectory_interface::QuinticSplineSegment<double>, hardware_interface::PositionJointInterface>;
public:
    void update(const ros::Time& time, const ros::Duration& period) override
    {
        if (!hardware_ || !hardware_->isConnected()) return;
        const auto active = rt_active_goal_;
        Base::update(time, period);
        if (active && active->preallocated_result_->error_code !=
                          control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
        {
            // Main loop calls write() AFTER update(): latch first, so even the
            // point calculated in this failing update cannot enter the queue.
            hardware_->requestExecutionStop();
            ROS_ERROR("[AuboJTC] 轨迹执行失败 (%d)，已锁定供点；执行柜端停止，需重启后才能运动",
                      active->preallocated_result_->error_code);
        }
    }

protected:
    bool initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                     ros::NodeHandle& controller_nh, ClaimedResources& claims) override
    {
        hardware_ = dynamic_cast<AuboHardwareInterface*>(robot_hw);
        if (!hardware_)
        {
            ROS_ERROR("[AuboJTC] 需要独占的 AuboHardwareInterface JTC 模式");
            return false;
        }
        return Base::initRequest(robot_hw, root_nh, controller_nh, claims);
    }
private:
    AuboHardwareInterface* hardware_{nullptr};
};
} // namespace aubo_ros_control
