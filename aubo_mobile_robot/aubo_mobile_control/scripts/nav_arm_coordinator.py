#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import sys
import threading
import time

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
        self.group_name = rospy.get_param("~planning_group", "aubo_i5") # 规划组
        self.end_effector_link = rospy.get_param("~end_effector_link", "tcp_link") # 末端执行坐标系
        self.navigation_action = rospy.get_param("~navigation_action", "/move_base") # 导航动作服务器
        self.navigation_frame = rospy.get_param("~navigation_frame", "map") 
        self.goal_source = rospy.get_param("~goal_source", "launch").strip().lower() # 去掉空格转成小写
        self.rviz_goal_topic = rospy.get_param(
            "~rviz_goal_topic", "/nav_arm_coordinator/goal"
        )
        self.goal_wait_timeout = float(rospy.get_param("~goal_wait_timeout", 0.0))
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

        self.goal_x = float(rospy.get_param("~goal_x", 1.0))        # 让机器人移动的目标位置
        self.goal_y = float(rospy.get_param("~goal_y", 0.0))
        self.goal_yaw = float(rospy.get_param("~goal_yaw", 0.0))

        if self.goal_source not in ("launch", "rviz"):
            raise ValueError(
                "Unsupported goal_source '%s'; use 'launch' or 'rviz'"
                % self.goal_source
            )

        self.rviz_goal = None
        self.rviz_goal_event = threading.Event()
        self.rviz_goal_subscriber = None
        if self.goal_source == "rviz":
            self.rviz_goal_subscriber = rospy.Subscriber(
                self.rviz_goal_topic,
                PoseStamped,
                self._rviz_goal_callback,
                queue_size=1,
            )  # 如果数据源是rviz 设置对应的话题订阅

        self.arm = moveit_commander.MoveGroupCommander(self.group_name) # 对moveit_group接口 进行初始化
        self.arm.set_end_effector_link(self.end_effector_link)
        self.arm.set_planning_time(self.arm_planning_time)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.arm.set_max_acceleration_scaling_factor(self.acceleration_scaling)

        self.navigation_client = actionlib.SimpleActionClient(
            self.navigation_action, MoveBaseAction
        ) # 导航move_base动作客户端

    def _rviz_goal_callback(self, target):
        self.rviz_goal = target
        self.rviz_goal_event.set()

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

    def _get_navigation_target(self):
        if self.goal_source == "rviz":
            rospy.loginfo(
                "Waiting for an RViz 2D Nav Goal on %s", self.rviz_goal_topic
            )
            deadline = None
            if self.goal_wait_timeout > 0.0:
                deadline = time.time() + self.goal_wait_timeout

            while not self.rviz_goal_event.wait(0.2):
                if rospy.is_shutdown():
                    return None
                if deadline is not None and time.time() >= deadline:
                    rospy.logerr(
                        "Timed out waiting %.1f seconds for an RViz navigation goal",
                        self.goal_wait_timeout,
                    )
                    return None

            target = self.rviz_goal
            if not target.header.frame_id:
                rospy.logerr("RViz navigation goal has an empty frame_id")
                return None

            # This coordinator intentionally executes one navigation/arm task.
            # Stop accepting later RViz clicks so they cannot alter the task.
            self.rviz_goal_subscriber.unregister()

            rospy.loginfo(
                "Received RViz navigation goal in %s: x=%.3f y=%.3f",
                target.header.frame_id,
                target.pose.position.x,
                target.pose.position.y,
            )
            return target

        quaternion = quaternion_from_euler(0.0, 0.0, self.goal_yaw)
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = self.navigation_frame
        target.pose.position.x = self.goal_x
        target.pose.position.y = self.goal_y
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]

        rospy.loginfo(
            "Navigating in %s to x=%.3f y=%.3f yaw=%.3f",
            self.navigation_frame,
            self.goal_x,
            self.goal_y,
            self.goal_yaw,
        )

        return target

    def _navigate(self):
        target = self._get_navigation_target()
        if target is None:
            return False

        rospy.loginfo("Waiting for navigation action %s", self.navigation_action)
        if not self.navigation_client.wait_for_server(rospy.Duration(self.server_timeout)):
            rospy.logerr("Navigation action server was not available")
            return False

        goal = MoveBaseGoal()
        goal.target_pose = target
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
            state_name = {
                GoalStatus.PENDING: "PENDING",
                GoalStatus.ACTIVE: "ACTIVE",
                GoalStatus.PREEMPTED: "PREEMPTED",
                GoalStatus.SUCCEEDED: "SUCCEEDED",
                GoalStatus.ABORTED: "ABORTED",
                GoalStatus.REJECTED: "REJECTED",
                GoalStatus.PREEMPTING: "PREEMPTING",
                GoalStatus.RECALLING: "RECALLING",
                GoalStatus.RECALLED: "RECALLED",
                GoalStatus.LOST: "LOST",
            }.get(state, "UNKNOWN")
            rospy.logerr(
                "Navigation did not succeed: %s (action state %d)",
                state_name,
                state,
            )
            return False

        rospy.loginfo("Navigation goal reached")
        return True

    def run(self):
        if self.startup_delay > 0.0:
            rospy.loginfo("Waiting %.1f seconds for controllers and localization", self.startup_delay)
            rospy.sleep(self.startup_delay)

        if not self.skip_pre_navigation:  # 如果不跳过导航前的动作
            if not self._move_arm_to_named_target(self.pre_navigation_target): # 移动机械臂到指定位置
                return False

        if not self._navigate():        # 导航到指定位置
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
