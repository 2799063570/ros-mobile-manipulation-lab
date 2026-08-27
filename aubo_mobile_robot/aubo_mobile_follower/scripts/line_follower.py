#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Follow an HSV guide line using the downward-looking hand camera."""

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


class LineFollower(object):
    def __init__(self):
        self.image_topic = rospy.get_param(
            "~image_topic", "/hand_camera/image_raw"
        )
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/aubo_mobile_follower/line_debug"
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
            rospy.get_param("~line/hsv_lower", [0, 0, 0]), dtype=np.uint8
        )
        self.hsv_upper = np.array(
            rospy.get_param("~line/hsv_upper", [180, 255, 65]), dtype=np.uint8
        )
        self.roi_top_fraction = float(
            rospy.get_param("~line/roi_top_fraction", 0.55)
        )
        self.min_mask_fraction = float(
            rospy.get_param("~line/min_mask_fraction", 0.006)
        )
        self.linear_speed = float(rospy.get_param("~line/linear_speed", 0.16))
        self.angular_kp = float(rospy.get_param("~line/angular_kp", 0.9))
        self.angular_kd = float(rospy.get_param("~line/angular_kd", 0.12))
        self.max_angular_speed = float(
            rospy.get_param("~line/max_angular_speed", 0.55)
        )

        self.bridge = cv_bridge.CvBridge()
        self._lock = threading.Lock()
        self._arm_ready = not self.require_arm_ready
        self._line_error = None
        self._error_derivative = 0.0
        self._previous_error = None
        self._previous_time = None
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
            rospy.logwarn_throttle(2.0, "Cannot decode line image: %s", str(error))
            return

        height, width = frame.shape[:2]
        roi_top = int(clamp(self.roi_top_fraction, 0.0, 0.95) * height)
        roi = frame[roi_top:height, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        moments = cv2.moments(mask)
        mask_fraction = moments["m00"] / (255.0 * mask.shape[0] * mask.shape[1])

        error = None
        if moments["m00"] > 0.0 and mask_fraction >= self.min_mask_fraction:
            center_x = moments["m10"] / moments["m00"]
            center_y = moments["m01"] / moments["m00"]
            error = (center_x - width * 0.5) / (width * 0.5)
            cv2.circle(
                frame,
                (int(center_x), roi_top + int(center_y)),
                7,
                (255, 0, 255),
                -1,
            )
        cv2.line(frame, (0, roi_top), (width - 1, roi_top), (0, 255, 255), 1)

        now = rospy.Time.now()
        derivative = 0.0
        if (
            error is not None
            and self._previous_error is not None
            and self._previous_time is not None
        ):
            delta = (now - self._previous_time).to_sec()
            if delta > 1.0e-4:
                derivative = (error - self._previous_error) / delta

        with self._lock:
            self._line_error = error
            self._error_derivative = derivative
            self._last_image_time = now
            if error is not None:
                self._previous_error = error
                self._previous_time = now
            else:
                self._previous_error = None
                self._previous_time = None

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
            error = self._line_error
            derivative = self._error_derivative
            image_time = self._last_image_time

        if not ready:
            self._stop()
            rospy.logwarn_throttle(2.0, "Line follower waiting for downward camera pose")
            return
        if image_time is None or (now - image_time).to_sec() > self.sensor_timeout:
            self._stop()
            rospy.logwarn_throttle(2.0, "Line follower image timeout")
            return
        if error is None:
            self._stop()
            self._state_publisher.publish(String(data="line_lost"))
            return

        command = Twist()
        # Slow down on sharp bends so that the extended arm remains stable.
        command.linear.x = self.linear_speed * max(0.35, 1.0 - abs(error))
        command.angular.z = clamp(
            -self.angular_kp * error - self.angular_kd * derivative,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        self._command_publisher.publish(command)
        self._state_publisher.publish(String(data="line_following"))

    def _stop(self):
        self._command_publisher.publish(Twist())


def main():
    rospy.init_node("aubo_line_follower")
    LineFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
