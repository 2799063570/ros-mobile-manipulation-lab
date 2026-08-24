#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import sys

import actionlib
import moveit_commander
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


class NavigationArmCoordinator(object):
    """Run a safe arm-pose -> base-navigation -> arm-plan sequence once."""

    def __init__(self):
        self.group_name = rospy.get_param("~planning_group", "aubo_i5")
        self.end_effector_link = rospy.get_param("~end_effector_link", "tcp_link")
        self.navigation_action = rospy.get_param("~navigation_action", "/move_base")
        self.navigation_frame = rospy.get_param("~navigation_frame", "map")
        self.pre_navigation_target = rospy.get_param("~pre_navigation_target", "down")
        self.arm_target_type = rospy.get_param("~arm_target_type", "named")
        self.post_navigation_target = rospy.get_param("~post_navigation_target", "up")
        self.skip_pre_navigation = bool(rospy.get_param("~skip_pre_navigation", False))
        self.server_timeout = float(rospy.get_param("~server_timeout", 30.0))
        self.startup_delay = float(rospy.get_param("~startup_delay", 2.0))
        self.navigation_timeout = float(rospy.get_param("~navigation_timeout", 180.0))
        self.arm_planning_time = float(rospy.get_param("~arm_planning_time", 10.0))
        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", 0.20))
        self.acceleration_scaling = float(rospy.get_param("~acceleration_scaling", 0.20))

        self.goal_x = float(rospy.get_param("~goal_x", 1.0))
        self.goal_y = float(rospy.get_param("~goal_y", 0.0))
        self.goal_yaw = float(rospy.get_param("~goal_yaw", 0.0))

        self.arm = moveit_commander.MoveGroupCommander(self.group_name)
        self.arm.set_end_effector_link(self.end_effector_link)
        self.arm.set_planning_time(self.arm_planning_time)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.arm.set_max_acceleration_scaling_factor(self.acceleration_scaling)

        self.navigation_client = actionlib.SimpleActionClient(
            self.navigation_action, MoveBaseAction
        )

    def _move_arm_to_named_target(self, target_name):
        if not target_name:
            return True

        available_targets = self.arm.get_named_targets()
        if target_name not in available_targets:
            rospy.logerr(
                "Arm target '%s' does not exist. Available targets: %s",
                target_name,
                ", ".join(available_targets),
            )
            return False

        rospy.loginfo("Planning arm group %s to named target %s", self.group_name, target_name)
        self.arm.set_named_target(target_name)
        success = bool(self.arm.go(wait=True))
        self.arm.stop()
        self.arm.clear_pose_targets()
        if not success:
            rospy.logerr("Failed to move arm to named target %s", target_name)
        return success

    def _move_arm_to_pose_target(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = rospy.get_param("~arm_pose_frame", "base_link")
        pose.pose.position.x = float(rospy.get_param("~arm_x", 0.45))
        pose.pose.position.y = float(rospy.get_param("~arm_y", 0.0))
        pose.pose.position.z = float(rospy.get_param("~arm_z", 0.45))
        roll = float(rospy.get_param("~arm_roll", 0.0))
        pitch = float(rospy.get_param("~arm_pitch", math.pi))
        yaw = float(rospy.get_param("~arm_yaw", 0.0))
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        rospy.loginfo(
            "Planning %s to pose in %s: [%.3f, %.3f, %.3f]",
            self.end_effector_link,
            pose.header.frame_id,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        self.arm.set_pose_target(pose, self.end_effector_link)
        success = bool(self.arm.go(wait=True))
        self.arm.stop()
        self.arm.clear_pose_targets()
        if not success:
            rospy.logerr("Failed to plan or execute arm pose target")
        return success

    def _navigate(self):
        rospy.loginfo("Waiting for navigation action %s", self.navigation_action)
        if not self.navigation_client.wait_for_server(rospy.Duration(self.server_timeout)):
            rospy.logerr("Navigation action server was not available")
            return False

        quaternion = quaternion_from_euler(0.0, 0.0, self.goal_yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.navigation_frame
        goal.target_pose.pose.position.x = self.goal_x
        goal.target_pose.pose.position.y = self.goal_y
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo(
            "Navigating in %s to x=%.3f y=%.3f yaw=%.3f",
            self.navigation_frame,
            self.goal_x,
            self.goal_y,
            self.goal_yaw,
        )
        self.navigation_client.send_goal(goal)
        finished = self.navigation_client.wait_for_result(
            rospy.Duration(self.navigation_timeout)
        )
        if not finished:
            self.navigation_client.cancel_goal()
            rospy.logerr("Navigation timed out after %.1f seconds", self.navigation_timeout)
            return False

        state = self.navigation_client.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr("Navigation failed with action state %d", state)
            return False

        rospy.loginfo("Navigation goal reached")
        return True

    def run(self):
        if self.startup_delay > 0.0:
            rospy.loginfo("Waiting %.1f seconds for controllers and localization", self.startup_delay)
            rospy.sleep(self.startup_delay)

        if not self.skip_pre_navigation:
            if not self._move_arm_to_named_target(self.pre_navigation_target):
                return False

        if not self._navigate():
            return False

        if self.arm_target_type == "named":
            return self._move_arm_to_named_target(self.post_navigation_target)
        if self.arm_target_type == "pose":
            return self._move_arm_to_pose_target()

        rospy.logerr("Unsupported arm_target_type '%s'; use 'named' or 'pose'", self.arm_target_type)
        return False


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("nav_arm_coordinator")
    exit_code = 1
    try:
        coordinator = NavigationArmCoordinator()
        exit_code = 0 if coordinator.run() else 1
    except Exception as error:
        rospy.logerr("Navigation/arm task failed: %s", str(error))
    finally:
        moveit_commander.roscpp_shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
