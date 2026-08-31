#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Supervise line-following velocity with semantic YOLO actions."""

from __future__ import print_function

import math
import threading

import rospy
from aubo_mobile_follower.semantic_drive_core import DetectionGate, SemanticDriveCore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion

try:
    from darknet_ros_msgs.msg import BoundingBoxes
except ImportError:
    BoundingBoxes = None


class SemanticDriveSupervisor(object):
    def __init__(self):
        self.nominal_topic = rospy.get_param(
            "~nominal_cmd_vel_topic", "/aubo_mobile_follower/line_cmd"
        )
        self.output_topic = rospy.get_param("~output_cmd_vel_topic", "/cmd_vel_raw")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.ready_topic = rospy.get_param(
            "~ready_topic", "/aubo_mobile_follower/arm_ready"
        )
        self.motion_lock_topic = rospy.get_param(
            "~motion_lock_topic", "/sorting/base_locked"
        )
        self.state_topic = rospy.get_param(
            "~state_topic", "/aubo_mobile_follower/semantic_state"
        )
        self.detection_backend = str(
            rospy.get_param("~detection_backend", "darknet")
        ).lower()
        self.boxes_topic = rospy.get_param(
            "~boxes_topic", "/darknet_ros/bounding_boxes"
        )
        self.string_topic = rospy.get_param(
            "~string_detection_topic", "/aubo_mobile_follower/semantic_detection"
        )
        self.require_arm_ready = bool(rospy.get_param("~require_arm_ready", True))
        self.nominal_timeout = float(rospy.get_param("~nominal_timeout", 0.5))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.5))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.minimum_probability = float(rospy.get_param("~minimum_probability", 0.70))
        self.minimum_box_fraction = float(rospy.get_param("~minimum_box_fraction", 0.0))
        self.detector_image_width = int(rospy.get_param("~detector_image_width", 640))
        self.detector_image_height = int(rospy.get_param("~detector_image_height", 480))
        self.action_classes = rospy.get_param(
            "~action_classes",
            {
                "stop": "stop",
                "slow_down": "slow_down",
                "resume": "resume",
                "turn_left": "turn_left",
                "turn_right": "turn_right",
            },
        )
        self.action_priorities = rospy.get_param(
            "~action_priorities",
            {"stop": 100, "turn_left": 80, "turn_right": 80,
             "slow_down": 60, "resume": 40},
        )
        confirmation_frames = int(rospy.get_param("~confirmation_frames", 4))

        if self.control_rate <= 0.0:
            raise ValueError("control_rate must be positive")
        if self.nominal_timeout <= 0.0 or self.odom_timeout <= 0.0:
            raise ValueError("nominal_timeout and odom_timeout must be positive")
        if self.minimum_box_fraction > 0.0 and (
            self.detector_image_width <= 0 or self.detector_image_height <= 0
        ):
            raise ValueError("detector image dimensions must be positive")

        core_config = {
            "stop_hold_time": rospy.get_param("~stop_hold_time", 3.0),
            "slow_scale": rospy.get_param("~slow_scale", 0.35),
            "slow_duration": rospy.get_param("~slow_duration", 4.0),
            "turn_angle": rospy.get_param("~turn_angle", math.pi / 2.0),
            "turn_kp": rospy.get_param("~turn_kp", 1.4),
            "turn_min_speed": rospy.get_param("~turn_min_speed", 0.12),
            "turn_max_speed": rospy.get_param("~turn_max_speed", 0.45),
            "turn_tolerance": rospy.get_param("~turn_tolerance", 0.05236),
            "turn_settle_time": rospy.get_param("~turn_settle_time", 0.25),
            "cooldown": rospy.get_param("~cooldown", 4.0),
        }
        self._core = SemanticDriveCore(core_config)
        self._gate = DetectionGate(confirmation_frames)
        self._lock = threading.Lock()
        self._nominal_command = Twist()
        self._nominal_received = None
        self._yaw = None
        self._odom_received = None
        self._arm_ready = not self.require_arm_ready
        self._motion_locked = False
        self._last_state = None

        self._command_publisher = rospy.Publisher(
            self.output_topic, Twist, queue_size=3
        )
        self._state_publisher = rospy.Publisher(
            self.state_topic, String, queue_size=2, latch=True
        )
        self._nominal_subscriber = rospy.Subscriber(
            self.nominal_topic, Twist, self._nominal_callback, queue_size=3
        )
        self._odom_subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=3
        )
        self._ready_subscriber = rospy.Subscriber(
            self.ready_topic, Bool, self._ready_callback, queue_size=1
        )
        self._motion_lock_subscriber = rospy.Subscriber(
            self.motion_lock_topic, Bool, self._motion_lock_callback, queue_size=1
        )
        self._configure_detection_subscriber()
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self._control
        )
        rospy.on_shutdown(self._publish_stop)
        rospy.loginfo(
            "Semantic drive supervisor: %s + %s -> %s (%s backend)",
            self.nominal_topic,
            self.odom_topic,
            self.output_topic,
            self.detection_backend,
        )

    def _configure_detection_subscriber(self):
        if self.detection_backend == "darknet":
            if BoundingBoxes is None:
                raise RuntimeError(
                    "detection_backend=darknet requires darknet_ros_msgs; "
                    "use detection_backend=string for manual validation"
                )
            self._detection_subscriber = rospy.Subscriber(
                self.boxes_topic, BoundingBoxes, self._boxes_callback, queue_size=1
            )
        elif self.detection_backend == "string":
            self._detection_subscriber = rospy.Subscriber(
                self.string_topic, String, self._string_callback, queue_size=3
            )
        else:
            raise ValueError("detection_backend must be 'darknet' or 'string'")

    @staticmethod
    def _copy_twist(source):
        command = Twist()
        command.linear.x = source.linear.x
        command.angular.z = source.angular.z
        return command

    def _nominal_callback(self, message):
        with self._lock:
            self._nominal_command = self._copy_twist(message)
            self._nominal_received = rospy.Time.now()

    def _odom_callback(self, message):
        orientation = message.pose.pose.orientation
        yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]
        with self._lock:
            self._yaw = yaw
            self._odom_received = rospy.Time.now()

    def _ready_callback(self, message):
        with self._lock:
            self._arm_ready = bool(message.data)

    def _motion_lock_callback(self, message):
        locked = bool(message.data)
        with self._lock:
            if locked and not self._motion_locked:
                self._core.cancel_motion_action()
                self._gate.reset()
            self._motion_locked = locked

    def _class_to_action(self, class_name):
        normalized = str(class_name).strip().lower()
        for action, configured_class in self.action_classes.items():
            if normalized == str(configured_class).strip().lower():
                return str(action).strip().lower()
        return None

    def _boxes_callback(self, message):
        candidates = []
        for box in message.bounding_boxes:
            probability = float(getattr(box, "probability", 0.0))
            action = self._class_to_action(
                getattr(box, "Class", getattr(box, "class_name", ""))
            )
            width = max(0, int(box.xmax) - int(box.xmin))
            height = max(0, int(box.ymax) - int(box.ymin))
            if action and probability >= self.minimum_probability:
                priority = int(self.action_priorities.get(action, 0))
                candidates.append((priority, probability, width * height, action))

        if (
            self.minimum_box_fraction > 0.0
            and self.detector_image_width > 0
            and self.detector_image_height > 0
        ):
            image_area = float(self.detector_image_width * self.detector_image_height)
            candidates = [
                item for item in candidates
                if item[2] / image_area >= self.minimum_box_fraction
            ]
        action = max(
            candidates, default=(0, 0.0, 0, None), key=lambda item: (item[0], item[1])
        )[3]
        self._accept_detection(action)

    def _string_callback(self, message):
        label = message.data.strip().lower()
        self._accept_detection(self._class_to_action(label) or label or None)

    def _accept_detection(self, action):
        now_ros = rospy.Time.now()
        now = now_ros.to_sec()
        with self._lock:
            if self._motion_locked or not self._arm_ready:
                self._gate.reset()
                return
            confirmed = self._gate.update(action)
            if not confirmed:
                return
            if confirmed in ("turn_left", "turn_right") and (
                self._odom_received is None
                or (now_ros - self._odom_received).to_sec() > self.odom_timeout
            ):
                rospy.logwarn_throttle(
                    2.0, "Ignoring semantic turn because odometry is stale"
                )
                return
            accepted = self._core.trigger(confirmed, now, self._yaw)
        if accepted:
            rospy.loginfo("Semantic drive action accepted: %s", confirmed)

    def _publish_state(self, state):
        if state != self._last_state:
            self._state_publisher.publish(String(data=state))
            self._last_state = state

    def _control(self, _event):
        now_ros = rospy.Time.now()
        now = now_ros.to_sec()
        with self._lock:
            nominal = self._copy_twist(self._nominal_command)
            nominal_received = self._nominal_received
            yaw = self._yaw
            odom_received = self._odom_received
            arm_ready = self._arm_ready
            motion_locked = self._motion_locked

            if motion_locked:
                command = Twist()
                state = "BASE_LOCKED"
            elif not arm_ready:
                command = Twist()
                state = "WAITING_FOR_ARM"
            elif nominal_received is None or (
                now_ros - nominal_received
            ).to_sec() > self.nominal_timeout:
                command = Twist()
                state = "NOMINAL_CMD_TIMEOUT"
            elif self._core.turning and (
                odom_received is None
                or (now_ros - odom_received).to_sec() > self.odom_timeout
            ):
                command = Twist()
                state = "ODOM_TIMEOUT"
            else:
                linear_x, angular_z, state = self._core.command(
                    nominal.linear.x, nominal.angular.z, now, yaw
                )
                command = Twist()
                command.linear.x = linear_x
                command.angular.z = angular_z

        self._command_publisher.publish(command)
        self._publish_state(state)

    def _publish_stop(self):
        self._command_publisher.publish(Twist())


def main():
    rospy.init_node("semantic_drive_supervisor")
    try:
        SemanticDriveSupervisor()
    except (RuntimeError, ValueError) as error:
        rospy.logfatal("Cannot start semantic drive supervisor: %s", str(error))
        return
    rospy.spin()


if __name__ == "__main__":
    main()
