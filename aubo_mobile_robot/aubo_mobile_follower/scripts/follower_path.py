#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Publish a bounded RViz path from the mobile base odometry."""

from __future__ import print_function

import math

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class FollowerPath(object):
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.path_topic = rospy.get_param(
            "~path_topic", "/aubo_mobile_follower/path"
        )
        self.max_poses = max(10, int(rospy.get_param("~max_poses", 2000)))
        self.min_distance = max(0.0, float(rospy.get_param("~min_distance", 0.02)))
        self.path = Path()
        self._last_position = None
        self._publisher = rospy.Publisher(
            self.path_topic, Path, queue_size=1, latch=True
        )
        self._subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=20
        )

    def _odom_callback(self, message):
        position = message.pose.pose.position
        if self._last_position is not None:
            distance = math.hypot(
                position.x - self._last_position[0],
                position.y - self._last_position[1],
            )
            if distance < self.min_distance:
                return

        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.path.header = message.header
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses :]
        self._last_position = (position.x, position.y)
        self._publisher.publish(self.path)


def main():
    rospy.init_node("aubo_follower_path")
    FollowerPath()
    rospy.spin()


if __name__ == "__main__":
    main()
