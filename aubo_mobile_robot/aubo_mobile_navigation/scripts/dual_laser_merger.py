#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import threading

import rospy
import tf
from sensor_msgs.msg import LaserScan


class DualLaserMerger(object):
    """Merge two planar scans into one scan centred on a common target frame."""

    def __init__(self):
        self.front_topic = rospy.get_param("~front_topic", "/front/scan")
        self.rear_topic = rospy.get_param("~rear_topic", "/rear/scan")
        self.output_topic = rospy.get_param("~output_topic", "/scan")
        self.target_frame = rospy.get_param("~target_frame", "base_footprint")
        self.publish_rate = float(rospy.get_param("~publish_rate", 15.0))
        self.samples = int(rospy.get_param("~samples", 720))
        self.range_min = float(rospy.get_param("~range_min", 0.12))
        self.range_max = float(rospy.get_param("~range_max", 20.0))
        self.max_scan_age = rospy.Duration(float(rospy.get_param("~max_scan_age", 0.5)))
        self.require_both_scans = bool(rospy.get_param("~require_both_scans", True))

        if self.samples < 4:
            raise ValueError("~samples must be at least 4")
        if self.publish_rate <= 0.0:
            raise ValueError("~publish_rate must be positive")

        self._lock = threading.Lock()
        self._front = None
        self._rear = None
        self._last_published_stamp = None
        self._listener = tf.TransformListener()
        self._publisher = rospy.Publisher(self.output_topic, LaserScan, queue_size=2)

        self._front_subscriber = rospy.Subscriber(
            self.front_topic, LaserScan, self._front_callback, queue_size=5
        )
        self._rear_subscriber = rospy.Subscriber(
            self.rear_topic, LaserScan, self._rear_callback, queue_size=5
        )
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish)

        rospy.loginfo(
            "Merging %s and %s into %s in frame %s",
            self.front_topic,
            self.rear_topic,
            self.output_topic,
            self.target_frame,
        )

    def _front_callback(self, scan):
        with self._lock:
            self._front = (scan, rospy.Time.now())

    def _rear_callback(self, scan):
        with self._lock:
            self._rear = (scan, rospy.Time.now())

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        x, y, z, w = quaternion
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    def _transform_for_scan(self, scan):
        if scan.header.frame_id == self.target_frame:
            return (0.0, 0.0, 0.0)

        translation, quaternion = self._listener.lookupTransform(
            self.target_frame, scan.header.frame_id, rospy.Time(0)
        )
        return (translation[0], translation[1], self._yaw_from_quaternion(quaternion))

    def _add_scan(self, scan, output_ranges, angle_min, angle_increment):
        tx, ty, yaw = self._transform_for_scan(scan)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        angle = scan.angle_min
        for measured_range in scan.ranges:
            if not math.isnan(measured_range) and not math.isinf(measured_range):
                if scan.range_min <= measured_range <= scan.range_max:
                    local_x = measured_range * math.cos(angle)
                    local_y = measured_range * math.sin(angle)
                    target_x = tx + cos_yaw * local_x - sin_yaw * local_y
                    target_y = ty + sin_yaw * local_x + cos_yaw * local_y
                    target_range = math.hypot(target_x, target_y)

                    if self.range_min <= target_range <= self.range_max:
                        target_angle = math.atan2(target_y, target_x)
                        index = int(math.floor((target_angle - angle_min) / angle_increment))
                        if index == self.samples:
                            index = 0
                        if 0 <= index < self.samples and target_range < output_ranges[index]:
                            output_ranges[index] = target_range
            angle += scan.angle_increment

    def _publish(self, _event):
        now = rospy.Time.now()
        with self._lock:
            entries = [self._front, self._rear]

        valid_scans = []
        for entry in entries:
            if entry is not None and now - entry[1] <= self.max_scan_age:
                valid_scans.append(entry[0])

        if self.require_both_scans and len(valid_scans) != 2:
            rospy.logwarn_throttle(5.0, "Waiting for fresh front and rear lidar scans")
            return
        if not valid_scans:
            return

        # The timer and Gazebo lidar updates run independently.  A timer tick
        # can therefore see the same pair of source scans more than once.  Do
        # not republish that data: AMCL otherwise processes a duplicate scan
        # and broadcasts map -> odom again with the same timestamp, which
        # causes TF_REPEATED_DATA in every TF listener.
        output_stamp = max(scan.header.stamp for scan in valid_scans)
        with self._lock:
            if output_stamp == self._last_published_stamp:
                return

        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / float(self.samples)
        output_ranges = [float("inf")] * self.samples

        try:
            for scan in valid_scans:
                self._add_scan(scan, output_ranges, angle_min, angle_increment)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
            rospy.logwarn_throttle(5.0, "Cannot transform lidar scan: %s", str(error))
            return

        merged = LaserScan()
        merged.header.stamp = output_stamp
        merged.header.frame_id = self.target_frame
        merged.angle_min = angle_min
        merged.angle_max = angle_min + (self.samples - 1) * angle_increment
        merged.angle_increment = angle_increment
        merged.time_increment = 0.0
        merged.scan_time = 1.0 / self.publish_rate
        merged.range_min = self.range_min
        merged.range_max = self.range_max
        merged.ranges = output_ranges
        self._publisher.publish(merged)
        with self._lock:
            self._last_published_stamp = output_stamp


def main():
    rospy.init_node("dual_laser_merger")
    DualLaserMerger()
    rospy.spin()


if __name__ == "__main__":
    main()
