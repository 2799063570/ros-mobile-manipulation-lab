#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Follow the largest HSV colour target seen by the hand camera."""

from __future__ import print_function

import math
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
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String

class ColorFollower(object):
    def __init__(self):
        self.image_topic = rospy.get_param(
            "~image_topic", "/hand_camera/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/hand_camera/camera_info"
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
        # A positive target distance enables calibrated, known-width ranging.
        # Zero preserves the original image-area controller.
        self.target_distance = float(rospy.get_param("~color/target_distance", 0.0))
        self.target_width = float(rospy.get_param("~color/target_width", 0.36))
        self.distance_deadband = float(
            rospy.get_param("~color/distance_deadband", 0.05)
        )
        self.distance_resume_deadband = float(
            rospy.get_param("~color/distance_resume_deadband", 0.10)
        )
        self.min_approach_speed = float(
            rospy.get_param("~color/min_approach_speed", 0.08)
        )
        if (
            not all(math.isfinite(value) for value in (
                self.target_distance, self.target_width, self.distance_deadband,
                self.distance_resume_deadband, self.min_approach_speed
            ))
            or self.target_distance < 0.0
            or self.target_width <= 0.0
            or self.distance_deadband < 0.0
            or self.distance_resume_deadband <= self.distance_deadband
            or self.min_approach_speed < 0.0
        ):
            raise ValueError("color distance or approach speed parameter is invalid")
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
        self.search_enabled = bool(rospy.get_param("~color/search_enabled", False))
        self.search_angular_speed = float(
            rospy.get_param("~color/search_angular_speed", 0.30)
        )
        self.search_switch_period = float(
            rospy.get_param("~color/search_switch_period", 4.0)
        )
        self.search_start_delay = float(
            rospy.get_param("~color/search_start_delay", 0.5)
        )
        self.search_timeout = float(rospy.get_param("~color/search_timeout", 20.0))
        if (
            not all(math.isfinite(value) for value in (
                self.search_angular_speed, self.search_switch_period,
                self.search_start_delay, self.search_timeout
            ))
            or self.search_angular_speed <= 0.0
            or self.search_switch_period <= 0.0
            or self.search_start_delay < 0.0
            or self.search_timeout <= 0.0
        ):
            raise ValueError("color search speed or timing is invalid")

        self.bridge = cv_bridge.CvBridge()
        self._lock = threading.Lock()
        self._arm_ready = not self.require_arm_ready
        self._target = None
        self._last_image_time = None
        self._first_image_time = None
        self._last_target_time = None
        self._searching = False
        self._distance_settled = False
        self._camera_intrinsics = None
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
        self._distance_publisher = rospy.Publisher(
            rospy.get_param("~distance_topic", "/aubo_mobile_follower/color_distance"),
            Float32, queue_size=2,
        )
        self._image_subscriber = rospy.Subscriber(
            self.image_topic, Image, self._image_callback, queue_size=1
        )
        if self.target_distance > 0.0:
            self._camera_info_subscriber = rospy.Subscriber(
                self.camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1
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

    def _camera_info_callback(self, message):
        focal_length = float(message.K[0])
        if message.width > 0 and math.isfinite(focal_length) and focal_length > 0.0:
            with self._lock:
                self._camera_intrinsics = (focal_length, message.width)

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
            intrinsics = self._camera_intrinsics

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
                measured_distance = None
                if intrinsics is not None:
                    left, _top, pixel_width, _height = cv2.boundingRect(contour)
                    # A cropped target cannot give a reliable size estimate.
                    if left > 0 and left + pixel_width < width and pixel_width > 0:
                        focal_length, calibration_width = intrinsics
                        measured_distance = (
                            focal_length * width / float(calibration_width)
                            * self.target_width / float(pixel_width)
                        )
                        self._distance_publisher.publish(Float32(data=measured_distance))
                target = (x_error, area_fraction, measured_distance)
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(
                    frame, (int(center_x), int(center_y)), 7, (255, 0, 255), -1
                )

        with self._lock:
            self._target = target
            image_time = rospy.Time.now()
            self._last_image_time = image_time
            if self._first_image_time is None:
                self._first_image_time = image_time
            if target is not None:
                self._last_target_time = image_time

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
            loss_start_time = self._last_target_time or self._first_image_time
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
            loss_seconds = max(0.0, (now - loss_start_time).to_sec())
            search_seconds = loss_seconds - self.search_start_delay
            if self.search_enabled and 0.0 <= search_seconds < self.search_timeout:
                # Sweep left once, then twice as far right. A short timeout
                # must still leave enough time for the first reversal.
                switch_period = min(
                    self.search_switch_period, self.search_timeout / 3.0
                )
                phase = int((search_seconds + switch_period)
                            / (2.0 * switch_period))
                command = Twist()
                search_speed = min(self.search_angular_speed, max_angular_speed)
                command.angular.z = search_speed * (1.0 if phase % 2 == 0 else -1.0)
                with self._lock:
                    if not self._searching:
                        self._linear_pid.reset()
                        self._angular_pid.reset()
                    self._searching = True
                self._command_publisher.publish(command)
                self._state_publisher.publish(String(data="color_searching"))
                return
            self._stop(reset_pid=True)
            self._state_publisher.publish(String(data="color_target_lost"))
            return

        x_error, area_fraction, measured_distance = target
        if abs(x_error) <= x_deadband:
            x_error = 0.0
        if self.target_distance > 0.0:
            linear_error = None if measured_distance is None else (
                measured_distance - self.target_distance
            )
            with self._lock:
                if linear_error is None:
                    self._distance_settled = False
                elif self._distance_settled:
                    if abs(linear_error) <= self.distance_resume_deadband:
                        linear_error = 0.0
                    else:
                        self._distance_settled = False
                elif abs(linear_error) <= self.distance_deadband:
                    self._distance_settled = True
                    linear_error = 0.0
        else:
            linear_error = target_area_fraction - area_fraction
            if abs(linear_error) <= area_deadband:
                linear_error = 0.0

        command = Twist()
        measurement_time = image_time.to_sec()
        with self._lock:
            if self._searching:
                self._linear_pid.reset()
                self._angular_pid.reset()
                self._searching = False
            if x_error == 0.0:
                self._angular_pid.reset()
            command.angular.z = self._angular_pid.update(
                -x_error,
                measurement_time,
                -max_angular_speed,
                max_angular_speed,
            )
            # First center a visible target. This also allows yaw correction
            # when range is unavailable because the target is cropped.
            if x_error != 0.0 or linear_error is None or linear_error == 0.0:
                self._linear_pid.reset()
            else:
                linear_output = self._linear_pid.update(
                    linear_error,
                    measurement_time,
                    -max_reverse_speed,
                    max_linear_speed,
                )
                if (
                    self.target_distance > 0.0
                    and linear_error > 0.0
                    and linear_output > 0.0
                ):
                    linear_output = max(
                        linear_output,
                        min(self.min_approach_speed, max_linear_speed),
                    )
                command.linear.x = linear_output
        self._command_publisher.publish(command)
        if x_error != 0.0:
            state = "color_aligning"
        elif linear_error is None:
            state = "color_waiting_for_range"
        else:
            state = "color_following"
        self._state_publisher.publish(String(data=state))

    def _stop(self, reset_pid=False):
        if reset_pid:
            with self._lock:
                self._linear_pid.reset()
                self._angular_pid.reset()
                self._searching = False
                self._distance_settled = False
        self._command_publisher.publish(Twist())


def main():
    rospy.init_node("aubo_color_follower")
    ColorFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
