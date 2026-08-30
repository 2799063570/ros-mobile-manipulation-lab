#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Follow the largest HSV colour target seen by the hand camera."""

from __future__ import print_function

import threading

import cv2
import cv_bridge
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class ColorFollower(object):
    def __init__(self):
        self.image_topic = rospy.get_param(
            "~image_topic", "/hand_camera/image_raw"
        )
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/aubo_mobile_follower/color_debug"
        )
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

        self.hsv_lower = np.array(
            rospy.get_param("~color/hsv_lower", [0, 110, 80]), dtype=np.uint8
        )
        self.hsv_upper = np.array(
            rospy.get_param("~color/hsv_upper", [10, 255, 255]), dtype=np.uint8
        )
        self.min_area_fraction = float(
            rospy.get_param("~color/min_area_fraction", 0.002)
        )
        self.target_area_fraction = float(
            rospy.get_param("~color/target_area_fraction", 0.075)
        )
        self.x_deadband = float(rospy.get_param("~color/x_deadband", 0.06))
        self.area_deadband = float(rospy.get_param("~color/area_deadband", 0.012))
        self.linear_kp = float(rospy.get_param("~color/linear_kp", 1.2))
        self.angular_kp = float(rospy.get_param("~color/angular_kp", 1.1))
        self.max_linear_speed = float(
            rospy.get_param("~color/max_linear_speed", 0.24)
        )
        self.max_reverse_speed = float(
            rospy.get_param("~color/max_reverse_speed", 0.08)
        )
        self.max_angular_speed = float(
            rospy.get_param("~color/max_angular_speed", 0.55)
        )

        self.bridge = cv_bridge.CvBridge()
        self._lock = threading.Lock()
        self._arm_ready = not self.require_arm_ready
        self._target = None
        self._last_image_time = None

        self._command_publisher = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=2
        )
        self._debug_publisher = rospy.Publisher(
            self.debug_topic, Image, queue_size=1
        )
        self._state_publisher = rospy.Publisher(
            self.state_topic, String, queue_size=2
        )
        self._image_subscriber = rospy.Subscriber(
            self.image_topic, Image, self._image_callback, queue_size=1
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

    def _image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except cv_bridge.CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "Cannot decode follower image: %s", str(error))
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]

        target = None
        if contours:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            height, width = frame.shape[:2]
            area_fraction = area / float(width * height)
            moments = cv2.moments(contour)
            if area_fraction >= self.min_area_fraction and moments["m00"] > 0.0:
                center_x = moments["m10"] / moments["m00"]
                center_y = moments["m01"] / moments["m00"]
                x_error = (center_x - width * 0.5) / (width * 0.5)
                target = (x_error, area_fraction)
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(
                    frame, (int(center_x), int(center_y)), 7, (255, 0, 255), -1
                )

        with self._lock:
            self._target = target
            self._last_image_time = rospy.Time.now()

        if self._debug_publisher.get_num_connections() > 0:
            try:
                self._debug_publisher.publish(
                    self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                )
            except cv_bridge.CvBridgeError:
                pass

    def _control(self, _event):
        now = rospy.Time.now()
        with self._lock:
            ready = self._arm_ready
            target = self._target
            image_time = self._last_image_time

        if not ready:
            self._stop()
            rospy.logwarn_throttle(2.0, "Colour follower waiting for forward camera pose")
            return
        if image_time is None or (now - image_time).to_sec() > self.sensor_timeout:
            self._stop()
            rospy.logwarn_throttle(2.0, "Colour follower image timeout")
            return
        if target is None:
            self._stop()
            self._state_publisher.publish(String(data="color_target_lost"))
            return

        x_error, area_fraction = target
        if abs(x_error) <= self.x_deadband:
            x_error = 0.0
        area_error = self.target_area_fraction - area_fraction
        if abs(area_error) <= self.area_deadband:
            area_error = 0.0

        command = Twist()
        command.angular.z = clamp(
            -self.angular_kp * x_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        command.linear.x = clamp(
            self.linear_kp * area_error,
            -self.max_reverse_speed,
            self.max_linear_speed,
        )
        self._command_publisher.publish(command)
        self._state_publisher.publish(String(data="color_following"))

    def _stop(self):
        self._command_publisher.publish(Twist())


def main():
    rospy.init_node("aubo_color_follower")
    ColorFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
