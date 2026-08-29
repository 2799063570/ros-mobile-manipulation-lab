#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Low-rate eye-to-hand PBVS using MoveIt pose replanning.

The fixed RGB-D frontend publishes a target PoseStamped.  This node transforms
that target into the arm base, compares it with the current TCP, and advances
by bounded Cartesian-position steps.  It is deliberately a separate process
from the high-rate eye-in-hand SDK controller; never launch both controllers.
"""

from __future__ import print_function

import math
import threading
import time

import moveit_commander
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler


class EyeToHandMoveItServo(object):
    def __init__(self):
        self.group_name = rospy.get_param("~planning_group", "aubo_i5")
        self.end_effector_link = rospy.get_param("~end_effector_link", "tcp_link")
        self.reference_frame = rospy.get_param("~reference_frame", "base_link")
        self.target_topic = rospy.get_param(
            "~target_topic", "/visual_servo/target_pose"
        )
        self.state_topic = rospy.get_param("~state_topic", "/visual_servo/state")
        self.base_lock_topic = rospy.get_param(
            "~base_lock_topic", "/sorting/base_locked"
        )
        self.plan_only = bool(rospy.get_param("~plan_only", True))
        self.start_enabled = bool(rospy.get_param("~start_enabled", False))
        self.target_timeout = float(rospy.get_param("~target_timeout", 0.35))
        self.loop_rate = float(rospy.get_param("~loop_rate", 2.0))
        self.maximum_step = float(rospy.get_param("~maximum_step", 0.04))
        self.position_tolerance = float(
            rospy.get_param("~position_tolerance", 0.01)
        )
        self.target_offset = np.asarray(
            rospy.get_param("~target_offset", [0.0, 0.0, 0.18]), dtype=float
        )
        self.grasp_rpy = rospy.get_param("~grasp_rpy", [math.pi, 0.0, 0.0])
        if self.target_offset.shape != (3,):
            raise rospy.ROSInitException("target_offset must contain three values")

        self.tf_listener = tf.TransformListener()
        self.arm = moveit_commander.MoveGroupCommander(self.group_name)
        self.arm.set_end_effector_link(self.end_effector_link)
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.set_planning_time(float(rospy.get_param("~planning_time", 5.0)))
        self.arm.set_num_planning_attempts(5)
        self.arm.set_max_velocity_scaling_factor(
            float(rospy.get_param("~velocity_scaling", 0.08))
        )
        self.arm.set_max_acceleration_scaling_factor(
            float(rospy.get_param("~acceleration_scaling", 0.08))
        )

        self._lock = threading.RLock()
        self._target = None
        self._target_received = 0.0
        self._enabled = self.start_enabled
        self._running = True

        self.state_publisher = rospy.Publisher(
            self.state_topic, String, queue_size=1, latch=True
        )
        self.base_lock_publisher = rospy.Publisher(
            self.base_lock_topic, Bool, queue_size=1, latch=True
        )
        self.target_subscriber = rospy.Subscriber(
            self.target_topic, PoseStamped, self._target_callback, queue_size=1
        )
        self.enable_service = rospy.Service(
            "/visual_servo/set_enabled", SetBool, self._set_enabled
        )
        self.reset_service = rospy.Service(
            "/visual_servo/reset", Trigger, self._reset
        )
        rospy.on_shutdown(self.shutdown)
        self._publish_state("WAITING" if self._enabled else "DISABLED")
        self.base_lock_publisher.publish(Bool(data=self._enabled))

    def _publish_state(self, state, detail=""):
        message = state if not detail else state + "|" + detail
        self.state_publisher.publish(String(data=message))

    def _target_callback(self, message):
        if not message.header.frame_id:
            return
        try:
            self.tf_listener.waitForTransform(
                self.reference_frame,
                message.header.frame_id,
                message.header.stamp,
                rospy.Duration(0.10),
            )
            transformed = self.tf_listener.transformPose(self.reference_frame, message)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as error:
            rospy.logwarn_throttle(1.0, "Eye-to-hand target TF failed: %s", str(error))
            return
        with self._lock:
            self._target = transformed
            self._target_received = time.time()

    def _set_enabled(self, request):
        with self._lock:
            self._enabled = bool(request.data)
            if self._enabled:
                self._target = None
                self._target_received = 0.0
        self.arm.stop()
        self.arm.clear_pose_targets()
        self.base_lock_publisher.publish(Bool(data=self._enabled))
        self._publish_state("WAITING" if self._enabled else "DISABLED")
        return SetBoolResponse(True, "eye-to-hand servo state changed")

    def _reset(self, _request):
        with self._lock:
            self._target = None
            self._target_received = 0.0
        self.arm.stop()
        self.arm.clear_pose_targets()
        self._publish_state("WAITING" if self._enabled else "DISABLED")
        return TriggerResponse(True, "eye-to-hand target cleared")

    def _goal(self, target):
        current = self.arm.get_current_pose(self.end_effector_link)
        current_position = np.asarray(
            [current.pose.position.x, current.pose.position.y, current.pose.position.z]
        )
        desired = np.asarray(
            [target.pose.position.x, target.pose.position.y, target.pose.position.z]
        ) + self.target_offset
        error = desired - current_position
        distance = float(np.linalg.norm(error))
        if distance <= self.position_tolerance:
            return None, distance
        if distance > self.maximum_step:
            desired = current_position + error * (self.maximum_step / distance)

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.reference_frame
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = desired
        quaternion = quaternion_from_euler(*[float(value) for value in self.grasp_rpy])
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        return goal, distance

    def spin(self):
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown() and self._running:
            with self._lock:
                enabled = self._enabled
                target = self._target
                target_age = time.time() - self._target_received
            if not enabled:
                rate.sleep()
                continue
            if target is None or target_age > self.target_timeout:
                self.arm.stop()
                self._publish_state("WAITING", "fresh eye-to-hand target required")
                rate.sleep()
                continue

            goal, distance = self._goal(target)
            if goal is None:
                self._publish_state("ALIGNED", "position error %.4f m" % distance)
                rate.sleep()
                continue
            self.arm.set_pose_target(goal, self.end_effector_link)
            plan = self.arm.plan()
            # Melodic returns RobotTrajectory; newer bindings may return a tuple.
            if isinstance(plan, tuple):
                planned = bool(plan[0])
                trajectory = plan[1]
            else:
                trajectory = plan
                planned = bool(trajectory.joint_trajectory.points)
            if not planned:
                self.arm.clear_pose_targets()
                self._publish_state("ERROR", "MoveIt could not plan the servo step")
                rate.sleep()
                continue
            if self.plan_only:
                self._publish_state("PLAN_ONLY", "error %.4f m" % distance)
            else:
                self._publish_state("TRACKING", "error %.4f m" % distance)
                if not self.arm.execute(trajectory, wait=True):
                    self._publish_state("ERROR", "servo step execution failed")
            self.arm.stop()
            self.arm.clear_pose_targets()
            rate.sleep()

    def shutdown(self):
        self._running = False
        self.arm.stop()
        self.arm.clear_pose_targets()
        self.base_lock_publisher.publish(Bool(data=False))


def main():
    moveit_commander.roscpp_initialize([])
    rospy.init_node("eye_to_hand_moveit_servo")
    node = EyeToHandMoveItServo()
    try:
        node.spin()
    finally:
        node.shutdown()
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
