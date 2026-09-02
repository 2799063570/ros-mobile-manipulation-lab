#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Follow an HSV guide line using the downward-looking hand camera."""

from __future__ import print_function

import threading

import cv2
import cv_bridge
import numpy as np
import rospy
from aubo_mobile_follower.cfg import LineFollowerConfig
from aubo_mobile_follower.image_message import bgr8_to_imgmsg
from aubo_mobile_follower.pid import FilteredPid
from dynamic_reconfigure.server import Server
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
        self.angular_ki = float(rospy.get_param("~line/angular_ki", 0.0))
        self.angular_kd = float(rospy.get_param("~line/angular_kd", 0.12))
        self.derivative_filter_alpha = float(
            rospy.get_param("~line/derivative_filter_alpha", 0.25)
        )
        self.angular_integral_limit = float(
            rospy.get_param("~line/angular_integral_limit", 1.0)
        )
        self.max_angular_speed = float(
            rospy.get_param("~line/max_angular_speed", 0.55)
        )

        self.bridge = cv_bridge.CvBridge()
        self._lock = threading.Lock()
        self._arm_ready = not self.require_arm_ready
        self._line_error = None
        self._last_image_time = None
        self._angular_pid = FilteredPid()

        initial_configuration = {
            "h_min": int(self.hsv_lower[0]),
            "s_min": int(self.hsv_lower[1]),
            "v_min": int(self.hsv_lower[2]),
            "h_max": int(self.hsv_upper[0]),
            "s_max": int(self.hsv_upper[1]),
            "v_max": int(self.hsv_upper[2]),
            "roi_top_fraction": self.roi_top_fraction,
            "min_mask_fraction": self.min_mask_fraction,
            "linear_speed": self.linear_speed,
            "angular_kp": self.angular_kp,
            "angular_ki": self.angular_ki,
            "angular_kd": self.angular_kd,
            "derivative_filter_alpha": self.derivative_filter_alpha,
            "angular_integral_limit": self.angular_integral_limit,
            "max_angular_speed": self.max_angular_speed,
        }
        self._reconfigure_server = Server(
            LineFollowerConfig, self._reconfigure_callback
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
            self.roi_top_fraction = config["roi_top_fraction"]
            self.min_mask_fraction = config["min_mask_fraction"]
            self.linear_speed = config["linear_speed"]
            self.angular_kp = config["angular_kp"]
            self.angular_ki = config["angular_ki"]
            self.angular_kd = config["angular_kd"]
            self.derivative_filter_alpha = config["derivative_filter_alpha"]
            self.angular_integral_limit = config["angular_integral_limit"]
            self._angular_pid.set_gains(
                self.angular_kp,
                self.angular_ki,
                self.angular_kd,
                self.derivative_filter_alpha,
                self.angular_integral_limit,
            )
            self.max_angular_speed = config["max_angular_speed"]
        return config

    def _image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except (cv_bridge.CvBridgeError, KeyError, TypeError, ValueError) as error:
            rospy.logwarn_throttle(2.0, "Cannot decode line image: %s", str(error))
            return

        with self._lock:
            hsv_lower = self.hsv_lower.copy()
            hsv_upper = self.hsv_upper.copy()
            roi_top_fraction = self.roi_top_fraction
            min_mask_fraction = self.min_mask_fraction

        height, width = frame.shape[:2]
        roi_top = int(clamp(roi_top_fraction, 0.0, 0.95) * height)
        roi = frame[roi_top:height, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        moments = cv2.moments(mask)
        mask_fraction = moments["m00"] / (255.0 * mask.shape[0] * mask.shape[1])

        error = None
        if moments["m00"] > 0.0 and mask_fraction >= min_mask_fraction:
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
        with self._lock:
            self._line_error = error
            self._last_image_time = now

        if self._debug_publisher.get_num_connections() > 0:
            try:
                self._debug_publisher.publish(
                    bgr8_to_imgmsg(frame, header=message.header)
                )
            except (TypeError, ValueError) as error:
                rospy.logwarn_throttle(
                    2.0, "Cannot encode line debug image: %s", str(error)
                )

    def _control(self, _event):
        now = rospy.Time.now()
        with self._lock:
            ready = self._arm_ready
            error = self._line_error
            image_time = self._last_image_time
            linear_speed = self.linear_speed
            max_angular_speed = self.max_angular_speed

        if not ready:
            self._stop(reset_pid=True)
            rospy.logwarn_throttle(2.0, "Line follower waiting for downward camera pose")
            return
        if image_time is None or (now - image_time).to_sec() > self.sensor_timeout:
            self._stop(reset_pid=True)
            rospy.logwarn_throttle(2.0, "Line follower image timeout")
            return
        if error is None:
            self._stop(reset_pid=True)
            self._state_publisher.publish(String(data="line_lost"))
            return

        command = Twist()
        # Slow down on sharp bends so that the extended arm remains stable.
        command.linear.x = linear_speed * max(0.35, 1.0 - abs(error))
        # With the wrist-mounted camera in the ``observe`` pose, increasing
        # image x corresponds to a positive base yaw correction.
        with self._lock:
            command.angular.z = self._angular_pid.update(
                error,
                image_time.to_sec(),
                -max_angular_speed,
                max_angular_speed,
            )
        self._command_publisher.publish(command)
        self._state_publisher.publish(String(data="line_following"))

    def _stop(self, reset_pid=False):
        if reset_pid:
            with self._lock:
                self._angular_pid.reset()
        self._command_publisher.publish(Twist())


def main():
    rospy.init_node("aubo_line_follower")
    LineFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
