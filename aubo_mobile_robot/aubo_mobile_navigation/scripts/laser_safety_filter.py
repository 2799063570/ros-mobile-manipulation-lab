#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Last-resort lidar stop layer for mobile-base velocity commands."""

from __future__ import print_function

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class LaserSafetyFilter(object):
    """Pass velocity commands only when the corresponding swept space is clear."""

    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.input_topic = rospy.get_param("~input_cmd_vel", "/cmd_vel_raw")
        self.output_topic = rospy.get_param("~output_cmd_vel", "/cmd_vel")
        self.blocked_topic = rospy.get_param("~blocked_topic", "~blocked")
        self.motion_lock_topic = rospy.get_param(
            "~motion_lock_topic", "/sorting/base_locked"
        )

        self.robot_half_width = float(rospy.get_param("~robot_half_width", 0.30))
        self.safety_margin = float(rospy.get_param("~safety_margin", 0.12))
        self.stop_distance = float(rospy.get_param("~stop_distance", 0.35))
        self.reverse_stop_distance = float(
            rospy.get_param("~reverse_stop_distance", self.stop_distance)
        )
        self.reaction_time = float(rospy.get_param("~reaction_time", 0.25))
        self.max_deceleration = float(rospy.get_param("~max_deceleration", 0.8))
        self.rotation_clearance = float(rospy.get_param("~rotation_clearance", 0.48))
        self.min_cluster_points = int(rospy.get_param("~min_cluster_points", 3))
        self.emergency_distance = float(rospy.get_param("~emergency_distance", 0.18))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.5))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.5))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.linear_deadband = float(rospy.get_param("~linear_deadband", 0.001))
        self.angular_deadband = float(rospy.get_param("~angular_deadband", 0.001))

        if self.robot_half_width <= 0.0 or self.safety_margin < 0.0:
            raise ValueError("robot dimensions must be positive")
        if self.max_deceleration <= 0.0:
            raise ValueError("max_deceleration must be positive")
        if self.min_cluster_points < 1:
            raise ValueError("min_cluster_points must be at least one")
        if self.scan_timeout <= 0.0 or self.command_timeout <= 0.0:
            raise ValueError("scan and command timeouts must be positive")
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        self._lock = threading.Lock()
        self._scan = None
        self._scan_received = None
        self._command = Twist()
        self._command_received = None
        self._last_blocked = None
        self._motion_locked = False

        self._velocity_publisher = rospy.Publisher(
            self.output_topic, Twist, queue_size=3
        )
        self._blocked_publisher = rospy.Publisher(
            self.blocked_topic, Bool, queue_size=1, latch=True
        )
        self._scan_subscriber = rospy.Subscriber(
            self.scan_topic, LaserScan, self._scan_callback, queue_size=3
        )
        self._command_subscriber = rospy.Subscriber(
            self.input_topic, Twist, self._command_callback, queue_size=3
        )
        self._motion_lock_subscriber = rospy.Subscriber(
            self.motion_lock_topic, Bool, self._motion_lock_callback, queue_size=1
        )
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._publish_safe_command
        )
        rospy.on_shutdown(self._publish_stop)

        rospy.loginfo(
            "Lidar safety filter: %s + %s -> %s",
            self.input_topic,
            self.scan_topic,
            self.output_topic,
        )

    def _scan_callback(self, scan):
        with self._lock:
            self._scan = scan
            self._scan_received = rospy.Time.now()

    def _command_callback(self, command):
        with self._lock:
            self._command = command
            self._command_received = rospy.Time.now()

    def _motion_lock_callback(self, message):
        with self._lock:
            self._motion_locked = bool(message.data)

    @staticmethod
    def _copy_twist(source):
        command = Twist()
        command.linear.x = source.linear.x
        command.linear.y = source.linear.y
        command.linear.z = source.linear.z
        command.angular.x = source.angular.x
        command.angular.y = source.angular.y
        command.angular.z = source.angular.z
        return command

    def _stopping_distance(self, speed, reverse=False):
        base = self.reverse_stop_distance if reverse else self.stop_distance
        return base + self.reaction_time * speed + speed * speed / (
            2.0 * self.max_deceleration
        )

    def _hazard_counts(self, scan, command):
        forward_limit = self._stopping_distance(max(command.linear.x, 0.0))
        reverse_limit = self._stopping_distance(max(-command.linear.x, 0.0), True)
        corridor_half_width = self.robot_half_width + self.safety_margin
        moving_forward = command.linear.x > self.linear_deadband
        moving_reverse = command.linear.x < -self.linear_deadband
        rotating = abs(command.angular.z) > self.angular_deadband

        current_cluster = 0
        largest_cluster = 0
        emergency = False
        angle = scan.angle_min
        for measured_range in scan.ranges:
            valid = (
                not math.isnan(measured_range)
                and not math.isinf(measured_range)
                and scan.range_min <= measured_range <= scan.range_max
            )
            if valid:
                x = measured_range * math.cos(angle)
                y = measured_range * math.sin(angle)
                in_motion_path = False
                if moving_forward:
                    in_motion_path = 0.0 < x <= forward_limit and abs(y) <= corridor_half_width
                elif moving_reverse:
                    in_motion_path = -reverse_limit <= x < 0.0 and abs(y) <= corridor_half_width

                if rotating and measured_range <= self.rotation_clearance:
                    in_motion_path = True

                if in_motion_path:
                    current_cluster += 1
                    largest_cluster = max(largest_cluster, current_cluster)
                    if measured_range <= self.emergency_distance:
                        emergency = True
                else:
                    current_cluster = 0
            else:
                current_cluster = 0
            angle += scan.angle_increment
        return largest_cluster, emergency

    def _is_blocked(self, scan, command):
        moving = (
            abs(command.linear.x) > self.linear_deadband
            or abs(command.angular.z) > self.angular_deadband
        )
        if not moving:
            return False, "stationary"

        hazard_count, emergency = self._hazard_counts(scan, command)
        if emergency:
            return True, "emergency obstacle"
        if hazard_count >= self.min_cluster_points:
            return True, "obstacle cluster (%d points)" % hazard_count
        return False, "clear"

    def _publish_blocked(self, blocked):
        if blocked != self._last_blocked:
            self._blocked_publisher.publish(Bool(data=blocked))
            self._last_blocked = blocked

    def _publish_safe_command(self, _event):
        now = rospy.Time.now()
        with self._lock:
            scan = self._scan
            scan_received = self._scan_received
            command = self._copy_twist(self._command)
            command_received = self._command_received
            motion_locked = self._motion_locked

        if motion_locked:
            self._velocity_publisher.publish(Twist())
            self._publish_blocked(True)
            rospy.logwarn_throttle(
                2.0, "Mobile base locked while the manipulator is operating"
            )
            return

        if command_received is None or (now - command_received).to_sec() > self.command_timeout:
            self._velocity_publisher.publish(Twist())
            self._publish_blocked(False)
            return

        if scan is None or scan_received is None or (now - scan_received).to_sec() > self.scan_timeout:
            self._velocity_publisher.publish(Twist())
            self._publish_blocked(True)
            rospy.logwarn_throttle(2.0, "Lidar data is stale; stopping the mobile base")
            return

        blocked, reason = self._is_blocked(scan, command)
        if blocked:
            self._velocity_publisher.publish(Twist())
            self._publish_blocked(True)
            rospy.logwarn_throttle(1.0, "Lidar safety stop: %s", reason)
        else:
            self._velocity_publisher.publish(command)
            self._publish_blocked(False)

    def _publish_stop(self):
        self._velocity_publisher.publish(Twist())


def main():
    rospy.init_node("laser_safety_filter")
    LaserSafetyFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
