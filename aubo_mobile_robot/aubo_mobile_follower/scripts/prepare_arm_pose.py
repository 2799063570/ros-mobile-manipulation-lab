#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Move the arm to a named follower camera pose before enabling the base."""

from __future__ import print_function

import sys

import moveit_commander
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class ArmPosePreparer(object):
    def __init__(self):
        self.group_name = rospy.get_param("~planning_group", "aubo_i5")
        self.target = rospy.get_param("~target", "transport")
        self.ready_topic = rospy.get_param(
            "~ready_topic", "/aubo_mobile_follower/arm_ready"
        )
        self.state_topic = rospy.get_param(
            "~state_topic", "/aubo_mobile_follower/state"
        )
        self.wait_for_camera = bool(rospy.get_param("~wait_for_camera", False))
        self.camera_topic = rospy.get_param(
            "~camera_topic", "/hand_camera/image_raw"
        )
        self.camera_timeout = float(rospy.get_param("~camera_timeout", 5.0))
        self.planning_time = float(rospy.get_param("~planning_time", 10.0))

        self.ready_publisher = rospy.Publisher(
            self.ready_topic, Bool, queue_size=1, latch=True
        )
        self.state_publisher = rospy.Publisher(
            self.state_topic, String, queue_size=1, latch=True
        )
        self.ready_publisher.publish(Bool(data=False))
        self.state_publisher.publish(String(data="preparing_arm"))

        self.arm = moveit_commander.MoveGroupCommander(self.group_name)
        self.arm.set_planning_time(self.planning_time)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(
            float(rospy.get_param("~velocity_scaling", 0.15))
        )
        self.arm.set_max_acceleration_scaling_factor(
            float(rospy.get_param("~acceleration_scaling", 0.15))
        )

    @staticmethod
    def _normalise_plan(result):
        if isinstance(result, tuple):
            if len(result) < 2:
                return False, None
            success, trajectory = bool(result[0]), result[1]
        else:
            trajectory = result
            success = trajectory is not None
        points = []
        if trajectory is not None and hasattr(trajectory, "joint_trajectory"):
            points = trajectory.joint_trajectory.points
        return success and bool(points), trajectory

    def prepare(self):
        targets = self.arm.get_named_targets()
        if self.target not in targets:
            rospy.logerr(
                "Follower arm pose '%s' is unavailable. Available: %s",
                self.target,
                ", ".join(targets),
            )
            self.state_publisher.publish(String(data="arm_target_missing"))
            return False

        self.arm.set_start_state_to_current_state()
        self.arm.set_named_target(self.target)
        planned, trajectory = self._normalise_plan(self.arm.plan())
        if not planned:
            rospy.logerr(
                "Follower arm pose '%s' is not reachable; follower remains disabled",
                self.target,
            )
            self.arm.clear_pose_targets()
            self.state_publisher.publish(String(data="arm_plan_failed"))
            return False

        rospy.loginfo("Executing collision-checked follower arm pose '%s'", self.target)
        executed = bool(self.arm.execute(trajectory, wait=True))
        self.arm.stop()
        self.arm.clear_pose_targets()
        if not executed:
            self.state_publisher.publish(String(data="arm_execution_failed"))
            return False

        if self.wait_for_camera:
            try:
                image = rospy.wait_for_message(
                    self.camera_topic, Image, timeout=self.camera_timeout
                )
            except rospy.ROSException:
                rospy.logerr("No image received from %s", self.camera_topic)
                self.state_publisher.publish(String(data="camera_not_ready"))
                return False
            if image.width <= 0 or image.height <= 0:
                self.state_publisher.publish(String(data="camera_image_empty"))
                return False

        self.ready_publisher.publish(Bool(data=True))
        self.state_publisher.publish(String(data="ready"))
        rospy.loginfo("Follower enabled; arm pose '%s' is ready", self.target)
        return True


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("prepare_follower_arm_pose")
    try:
        preparer = ArmPosePreparer()
        preparer.prepare()
        rospy.spin()
    except Exception as error:
        rospy.logerr("Cannot prepare follower arm pose: %s", str(error))
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
