#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("nav_sorting_initial_pose")
    x = float(rospy.get_param("~x", -2.5))
    y = float(rospy.get_param("~y", 0.0))
    yaw = float(rospy.get_param("~yaw", 0.0))
    frame = rospy.get_param("~frame", "map")
    publish_count = max(1, int(rospy.get_param("~publish_count", 5)))

    publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    deadline = time.time() + 20.0
    while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
        if time.time() > deadline:
            rospy.logwarn("AMCL did not subscribe to /initialpose within 20 seconds")
            break
        rospy.sleep(0.1)

    quaternion = quaternion_from_euler(0.0, 0.0, yaw)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = frame
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.x = quaternion[0]
    message.pose.pose.orientation.y = quaternion[1]
    message.pose.pose.orientation.z = quaternion[2]
    message.pose.pose.orientation.w = quaternion[3]
    message.pose.covariance[0] = 0.05 * 0.05
    message.pose.covariance[7] = 0.05 * 0.05
    message.pose.covariance[35] = math.radians(3.0) ** 2

    for _index in range(publish_count):
        message.header.stamp = rospy.Time.now()
        publisher.publish(message)
        rospy.sleep(0.2)
    rospy.loginfo("Published initial AMCL pose [%.2f, %.2f, %.2f]", x, y, yaw)


if __name__ == "__main__":
    main()
