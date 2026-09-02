#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Follow the largest HSV colour target seen by the hand camera."""

from __future__ import print_function

import threading

import cv2
import cv_bridge
import numpy as np
import rospy
from aubo_mobile_follower.cfg import ColorFollowerConfig
from aubo_mobile_follower.image_message import bgr8_to_imgmsg
from aubo_mobile_follower.pid import FilteredPid
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

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
        self.linear_ki = float(rospy.get_param("~color/linear_ki", 0.0))
        self.linear_kd = float(rospy.get_param("~color/linear_kd", 0.0))
        self.angular_kp = float(rospy.get_param("~color/angular_kp", 1.1))
        self.angular_ki = float(rospy.get_param("~color/angular_ki", 0.0))
        self.angular_kd = float(rospy.get_param("~color/angular_kd", 0.0))
        self.derivative_filter_alpha = float(
            rospy.get_param("~color/derivative_filter_alpha", 0.25)
        )
        self.linear_integral_limit = float(
            rospy.get_param("~color/linear_integral_limit", 1.0)
        )
        self.angular_integral_limit = float(
            rospy.get_param("~color/angular_integral_limit", 1.0)
        )
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
        self._linear_pid = FilteredPid()
        self._angular_pid = FilteredPid()

        initial_configuration = {
            "h_min": int(self.hsv_lower[0]),
            "s_min": int(self.hsv_lower[1]),
            "v_min": int(self.hsv_lower[2]),
            "h_max": int(self.hsv_upper[0]),
            "s_max": int(self.hsv_upper[1]),
            "v_max": int(self.hsv_upper[2]),
            "min_area_fraction": self.min_area_fraction,
            "target_area_fraction": self.target_area_fraction,
            "x_deadband": self.x_deadband,
            "area_deadband": self.area_deadband,
            "linear_kp": self.linear_kp,
            "linear_ki": self.linear_ki,
            "linear_kd": self.linear_kd,
            "angular_kp": self.angular_kp,
            "angular_ki": self.angular_ki,
            "angular_kd": self.angular_kd,
            "derivative_filter_alpha": self.derivative_filter_alpha,
            "linear_integral_limit": self.linear_integral_limit,
            "angular_integral_limit": self.angular_integral_limit,
            "max_linear_speed": self.max_linear_speed,
            "max_reverse_speed": self.max_reverse_speed,
            "max_angular_speed": self.max_angular_speed,
        }
        self._reconfigure_server = Server(
            ColorFollowerConfig, self._reconfigure_callback
        )
        self._reconfigure_server.update_configuration(initial_configuration)

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

    def _reconfigure_callback(self, config, _level):
        with self._lock:
            self.hsv_lower = np.array(
                [config["h_min"], config["s_min"], config["v_min"]],
                dtype=np.uint8,
            )
            self.hsv_upper = np.array(
                [config["h_max"], config["s_max"], config["v_max"]],
                dtype=np.uint8,
            )
            self.min_area_fraction = config["min_area_fraction"]
            self.target_area_fraction = config["target_area_fraction"]
            self.x_deadband = config["x_deadband"]
            self.area_deadband = config["area_deadband"]
            self.linear_kp = config["linear_kp"]
            self.linear_ki = config["linear_ki"]
            self.linear_kd = config["linear_kd"]
            self.angular_kp = config["angular_kp"]
            self.angular_ki = config["angular_ki"]
            self.angular_kd = config["angular_kd"]
            self.derivative_filter_alpha = config["derivative_filter_alpha"]
            self.linear_integral_limit = config["linear_integral_limit"]
            self.angular_integral_limit = config["angular_integral_limit"]
            self._linear_pid.set_gains(
                self.linear_kp,
                self.linear_ki,
                self.linear_kd,
                self.derivative_filter_alpha,
                self.linear_integral_limit,
            )
            self._angular_pid.set_gains(
                self.angular_kp,
                self.angular_ki,
                self.angular_kd,
                self.derivative_filter_alpha,
                self.angular_integral_limit,
            )
            self.max_linear_speed = config["max_linear_speed"]
            self.max_reverse_speed = config["max_reverse_speed"]
            self.max_angular_speed = config["max_angular_speed"]
        return config

    def _image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except (cv_bridge.CvBridgeError, KeyError, TypeError, ValueError) as error:
            rospy.logwarn_throttle(2.0, "Cannot decode follower image: %s", str(error))
            return

        with self._lock:
            hsv_lower = self.hsv_lower.copy()
            hsv_upper = self.hsv_upper.copy()
            min_area_fraction = self.min_area_fraction

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
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
            if area_fraction >= min_area_fraction and moments["m00"] > 0.0:
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
                    bgr8_to_imgmsg(frame, header=message.header)
                )
            except (TypeError, ValueError) as error:
                rospy.logwarn_throttle(
                    2.0, "Cannot encode follower debug image: %s", str(error)
                )

    def _control(self, _event):
        now = rospy.Time.now()
        with self._lock:
            ready = self._arm_ready
            target = self._target
            image_time = self._last_image_time
            target_area_fraction = self.target_area_fraction
            x_deadband = self.x_deadband
            area_deadband = self.area_deadband
            max_linear_speed = self.max_linear_speed
            max_reverse_speed = self.max_reverse_speed
            max_angular_speed = self.max_angular_speed

        if not ready:
            self._stop(reset_pid=True)
            rospy.logwarn_throttle(2.0, "Colour follower waiting for forward camera pose")
            return
        if image_time is None or (now - image_time).to_sec() > self.sensor_timeout:
            self._stop(reset_pid=True)
            rospy.logwarn_throttle(2.0, "Colour follower image timeout")
            return
        if target is None:
            self._stop(reset_pid=True)
            self._state_publisher.publish(String(data="color_target_lost"))
            return

        x_error, area_fraction = target
        if abs(x_error) <= x_deadband:
            x_error = 0.0
        area_error = target_area_fraction - area_fraction
        if abs(area_error) <= area_deadband:
            area_error = 0.0

        command = Twist()
        measurement_time = image_time.to_sec()
        with self._lock:
            if x_error == 0.0:
                self._angular_pid.reset()
            command.angular.z = self._angular_pid.update(
                -x_error,
                measurement_time,
                -max_angular_speed,
                max_angular_speed,
            )
            if area_error == 0.0:
                self._linear_pid.reset()
            command.linear.x = self._linear_pid.update(
                area_error,
                measurement_time,
                -max_reverse_speed,
                max_linear_speed,
            )
        self._command_publisher.publish(command)
        self._state_publisher.publish(String(data="color_following"))

    def _stop(self, reset_pid=False):
        if reset_pid:
            with self._lock:
                self._linear_pid.reset()
                self._angular_pid.reset()
        self._command_publisher.publish(Twist())


def main():
    rospy.init_node("aubo_color_follower")
    ColorFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
