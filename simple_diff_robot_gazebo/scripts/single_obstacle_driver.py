#!/usr/bin/env python
"""Drive a differential robot through relative waypoints around one obstacle."""

from __future__ import division

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SingleObstacleDriver(object):
    def __init__(self):
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)

        self.linear_speed = rospy.get_param("~linear_speed", 0.32)
        self.angular_speed = rospy.get_param("~angular_speed", 0.9)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.12)
        self.stop_distance = rospy.get_param("~stop_distance", 0.42)
        self.waypoints = rospy.get_param(
            "~waypoints", [[1.3, -1.2], [3.7, -1.2], [5.0, 0.0]])

        self.pose = None
        self.start_pose = None
        self.front_distance = float("inf")
        self.scan_received = False
        self.waypoint_index = 0
        self.finished = False
        rospy.on_shutdown(self.stop)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if self.start_pose is None:
            self.start_pose = self.pose
            rospy.loginfo("Obstacle demo started; following %d waypoints", len(self.waypoints))

    def scan_callback(self, msg):
        # Minimum valid range in a 50 degree cone directly in front of the robot.
        valid = []
        half_cone = math.radians(25.0)
        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if (abs(wrap_angle(angle)) <= half_cone
                    and not math.isnan(distance) and not math.isinf(distance)):
                if msg.range_min < distance < msg.range_max:
                    valid.append(distance)
        self.front_distance = min(valid) if valid else float("inf")
        self.scan_received = True

    def relative_target(self, waypoint):
        start_x, start_y, start_yaw = self.start_pose
        rel_x, rel_y = float(waypoint[0]), float(waypoint[1])
        target_x = start_x + math.cos(start_yaw) * rel_x - math.sin(start_yaw) * rel_y
        target_y = start_y + math.sin(start_yaw) * rel_x + math.cos(start_yaw) * rel_y
        return target_x, target_y

    def step(self):
        if self.pose is None or not self.scan_received or self.finished:
            return

        target_x, target_y = self.relative_target(self.waypoints[self.waypoint_index])
        dx = target_x - self.pose[0]
        dy = target_y - self.pose[1]
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.waypoint_index += 1
            self.stop()
            if self.waypoint_index >= len(self.waypoints):
                self.finished = True
                rospy.loginfo("Goal reached: the robot successfully passed the obstacle")
            else:
                rospy.loginfo("Waypoint %d/%d reached", self.waypoint_index, len(self.waypoints))
            return

        heading_error = wrap_angle(math.atan2(dy, dx) - self.pose[2])
        command = Twist()

        # The scan is an independent safety layer; the planned route should normally
        # keep the robot farther away than this threshold.
        if self.front_distance < self.stop_distance:
            command.angular.z = -self.angular_speed
            rospy.logwarn_throttle(1.0, "Safety turn: obstacle only %.2f m ahead", self.front_distance)
        else:
            command.angular.z = clamp(1.8 * heading_error, -self.angular_speed, self.angular_speed)
            if abs(heading_error) < 0.65:
                heading_scale = max(0.2, 1.0 - abs(heading_error) / 0.65)
                distance_scale = min(1.0, distance / 0.45)
                command.linear.x = self.linear_speed * heading_scale * distance_scale

        self.cmd_pub.publish(command)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("single_obstacle_driver")
    SingleObstacleDriver().run()
