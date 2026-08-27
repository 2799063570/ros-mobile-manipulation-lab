#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Follow a persistent lidar point cluster at a configured distance."""

from __future__ import print_function

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class LaserFollower(object):
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel_raw")
        self.ready_topic = rospy.get_param(
            "~ready_topic", "/aubo_mobile_follower/arm_ready"
        )
        self.state_topic = rospy.get_param(
            "~state_topic", "/aubo_mobile_follower/state"
        )
        self.require_arm_ready = bool(rospy.get_param("~require_arm_ready", True))
        self.sensor_timeout = float(rospy.get_param("~sensor_timeout", 0.5))
        self.control_rate = float(rospy.get_param("~control_rate", 15.0))

        self.target_distance = float(rospy.get_param("~laser/target_distance", 1.0))
        self.min_target_distance = float(
            rospy.get_param("~laser/min_target_distance", 0.35)
        )
        self.search_angle = float(rospy.get_param("~laser/search_angle", math.pi / 3.0))
        self.min_cluster_points = int(rospy.get_param("~laser/min_cluster_points", 4))
        self.cluster_jump = float(rospy.get_param("~laser/cluster_jump", 0.12))
        self.max_tracking_jump = float(rospy.get_param("~laser/max_tracking_jump", 0.55))
        self.angle_deadband = float(rospy.get_param("~laser/angle_deadband", 0.06))
        self.distance_deadband = float(rospy.get_param("~laser/distance_deadband", 0.08))
        self.linear_kp = float(rospy.get_param("~laser/linear_kp", 0.55))
        self.angular_kp = float(rospy.get_param("~laser/angular_kp", 1.4))
        self.max_linear_speed = float(rospy.get_param("~laser/max_linear_speed", 0.30))
        self.max_reverse_speed = float(rospy.get_param("~laser/max_reverse_speed", 0.12))
        self.max_angular_speed = float(rospy.get_param("~laser/max_angular_speed", 0.65))

        self._lock = threading.Lock()
        self._arm_ready = not self.require_arm_ready
        self._target = None
        self._last_target = None
        self._last_scan_time = None

        self._command_publisher = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=2
        )
        self._state_publisher = rospy.Publisher(
            self.state_topic, String, queue_size=2
        )
        self._scan_subscriber = rospy.Subscriber(
            self.scan_topic, LaserScan, self._scan_callback, queue_size=2
        )
        self._ready_subscriber = rospy.Subscriber(
            self.ready_topic, Bool, self._ready_callback, queue_size=1
        )
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self._control
        )
        rospy.on_shutdown(self._stop)

    def _ready_callback(self, message):
        with self._lock:
            self._arm_ready = bool(message.data)

    def _finish_cluster(self, cluster, clusters):
        if len(cluster) < self.min_cluster_points:
            return
        ordered = sorted(cluster, key=lambda item: item[1])
        middle = len(ordered) // 2
        if len(ordered) % 2:
            distance = ordered[middle][1]
        else:
            distance = 0.5 * (ordered[middle - 1][1] + ordered[middle][1])
        angle = sum(item[0] for item in cluster) / float(len(cluster))
        clusters.append((angle, distance, len(cluster)))

    def _extract_clusters(self, scan):
        clusters = []
        cluster = []
        angle = scan.angle_min
        for measured_range in scan.ranges:
            valid = (
                abs(angle) <= self.search_angle
                and not math.isnan(measured_range)
                and not math.isinf(measured_range)
                and max(scan.range_min, self.min_target_distance) <= measured_range <= scan.range_max
            )
            if valid:
                if cluster and abs(measured_range - cluster[-1][1]) > self.cluster_jump:
                    self._finish_cluster(cluster, clusters)
                    cluster = []
                cluster.append((angle, measured_range))
            elif cluster:
                self._finish_cluster(cluster, clusters)
                cluster = []
            angle += scan.angle_increment
        if cluster:
            self._finish_cluster(cluster, clusters)
        return clusters

    @staticmethod
    def _target_separation(first, second):
        x1 = first[1] * math.cos(first[0])
        y1 = first[1] * math.sin(first[0])
        x2 = second[1] * math.cos(second[0])
        y2 = second[1] * math.sin(second[0])
        return math.hypot(x1 - x2, y1 - y2)

    def _select_target(self, clusters):
        if not clusters:
            return None
        nearest = min(clusters, key=lambda item: item[1])
        if self._last_target is None:
            return nearest
        tracked = min(
            clusters,
            key=lambda item: self._target_separation(item, self._last_target),
        )
        if self._target_separation(tracked, self._last_target) <= self.max_tracking_jump:
            return tracked
        return nearest

    def _scan_callback(self, scan):
        target = self._select_target(self._extract_clusters(scan))
        with self._lock:
            self._target = target
            self._last_scan_time = rospy.Time.now()
            if target is not None:
                self._last_target = target

    def _control(self, _event):
        now = rospy.Time.now()
        with self._lock:
            ready = self._arm_ready
            target = self._target
            scan_time = self._last_scan_time

        if not ready:
            self._stop()
            rospy.logwarn_throttle(2.0, "Laser follower waiting for the arm transport pose")
            return
        if scan_time is None or (now - scan_time).to_sec() > self.sensor_timeout:
            self._stop()
            rospy.logwarn_throttle(2.0, "Laser follower scan timeout")
            return
        if target is None:
            self._stop()
            self._state_publisher.publish(String(data="laser_target_lost"))
            return

        angle, distance, _count = target
        angle_error = 0.0 if abs(angle) <= self.angle_deadband else angle
        distance_error = distance - self.target_distance
        if abs(distance_error) <= self.distance_deadband:
            distance_error = 0.0

        command = Twist()
        command.angular.z = clamp(
            self.angular_kp * angle_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        command.linear.x = clamp(
            self.linear_kp * distance_error,
            -self.max_reverse_speed,
            self.max_linear_speed,
        )
        self._command_publisher.publish(command)
        self._state_publisher.publish(String(data="laser_following"))

    def _stop(self):
        self._command_publisher.publish(Twist())


def main():
    rospy.init_node("aubo_laser_follower")
    LaserFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
