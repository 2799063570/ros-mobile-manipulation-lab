#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import json
import math
import threading
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from aubo_mobile_nav_sorting.cfg import NavSortingConfig


class NavigationSortingMission(object):
    """Coordinate a safe base-navigation -> observation -> sorting mission."""

    def __init__(self):
        self.navigation_action = rospy.get_param("~navigation_action", "/move_base")
        self.navigation_frame = rospy.get_param("~navigation_frame", "map")
        goal = rospy.get_param("~sorting_goal", [2.15, 0.0, 0.0])
        if not isinstance(goal, list) or len(goal) != 3:
            raise ValueError("~sorting_goal must be [x, y, yaw]")
        self.goal_x, self.goal_y, self.goal_yaw = [float(value) for value in goal]

        self.near_field_enabled = bool(
            rospy.get_param("~near_field_enabled", False)
        )
        self.pre_dock_goal = self._pose_param(
            "~pre_dock_goal", [1.85, 0.0, 0.0]
        )
        self.candidate_x = self._float_list(
            "~near_field_candidate_x", [2.16, 2.10, 2.06]
        )
        self.candidate_y = self._float_list(
            "~near_field_candidate_y", [0.0, -0.08, 0.08]
        )
        self.candidate_yaw = self._float_list(
            "~near_field_candidate_yaw", [0.0, -0.08, 0.08]
        )
        self.near_field_max_candidates = max(
            1, int(rospy.get_param("~near_field_max_candidates", 6))
        )
        self.table_geometry = self._float_list(
            "~near_field_table", [3.0, 0.0, 0.80, 1.20]
        )
        if len(self.table_geometry) != 4:
            raise ValueError("~near_field_table must be [center_x, center_y, size_x, size_y]")
        self.base_clearance = float(
            rospy.get_param("~near_field_base_clearance", 0.40)
        )
        self.workpiece_points = rospy.get_param(
            "~near_field_workpieces",
            [[2.78, -0.12], [2.86, 0.0], [2.78, 0.12]],
        )
        if not self.workpiece_points or any(
            not isinstance(point, list) or len(point) != 2
            for point in self.workpiece_points
        ):
            raise ValueError("~near_field_workpieces must contain [x, y] points")
        self.workpiece_points = [
            [float(value) for value in point] for point in self.workpiece_points
        ]
        self.detector_workspace = self._float_list(
            "~near_field_detector_workspace", [0.40, 0.82, -0.22, 0.22]
        )
        if len(self.detector_workspace) != 4:
            raise ValueError(
                "~near_field_detector_workspace must be [min_x, max_x, min_y, max_y]"
            )
        self.camera_target = self._float_list(
            "~near_field_camera_target", [0.62, 0.0]
        )
        if len(self.camera_target) != 2:
            raise ValueError("~near_field_camera_target must be [x, y]")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.direct_dock_enabled = bool(
            rospy.get_param("~near_field_direct_dock_enabled", True)
        )
        self.direct_dock_max_distance = max(
            0.0, float(rospy.get_param("~near_field_direct_dock_max_distance", 0.50))
        )
        self.direct_dock_lateral_tolerance = max(
            0.0, float(rospy.get_param("~near_field_direct_dock_lateral_tolerance", 0.04))
        )
        self.direct_dock_yaw_tolerance = max(
            0.0, float(rospy.get_param("~near_field_direct_dock_yaw_tolerance", 0.04))
        )
        self.direct_dock_goal_tolerance = max(
            0.01, float(rospy.get_param("~near_field_direct_dock_goal_tolerance", 0.06))
        )

        self.server_timeout = float(rospy.get_param("~server_timeout", 45.0))
        self.navigation_timeout = float(rospy.get_param("~navigation_timeout", 180.0))
        self.navigation_retries = max(0, int(rospy.get_param("~navigation_retries", 1)))
        self.initialization_timeout = float(
            rospy.get_param("~sorting_initialization_timeout", 60.0)
        )
        self.operation_timeout = float(
            rospy.get_param("~sorting_operation_timeout", 300.0)
        )
        self.startup_delay = float(rospy.get_param("~startup_delay", 3.0))
        self.home_before_navigation = bool(
            rospy.get_param("~home_before_navigation", True)
        )
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.workstations = rospy.get_param("~workstations", [])
        if not isinstance(self.workstations, list):
            raise ValueError("~workstations must be a list")
        self._validate_workstations(self.workstations)

        # move_base is used between workstations. Low-speed velocity pulses are
        # also reused for the final straight approach and remain safety-filtered.
        self.base_recovery_enabled = bool(
            rospy.get_param("~base_recovery_enabled", True)
        )
        self.base_recovery_topic = rospy.get_param(
            "~base_recovery_cmd_vel_topic", "/cmd_vel_raw"
        )
        self.base_recovery_speed = abs(
            float(rospy.get_param("~base_recovery_speed", 0.04))
        )
        self.base_recovery_rate = max(
            5.0, float(rospy.get_param("~base_recovery_rate", 20.0))
        )
        self.base_recovery_settle_time = max(
            0.0, float(rospy.get_param("~base_recovery_settle_time", 0.8))
        )
        self.base_recovery_steps = rospy.get_param(
            "~base_recovery_steps",
            [[0.0, 0.06], [0.0, -0.12], [0.0, 0.06], [0.05, 0.0], [-0.10, 0.0]],
        )
        self._validate_recovery_steps(self.base_recovery_steps)

        self.sorting_state_topic = rospy.get_param(
            "~sorting_state_topic", "/sorting/state"
        )
        self.home_service_name = rospy.get_param(
            "~sorting_home_service", "/sorting/home"
        )
        self.observe_service_name = rospy.get_param(
            "~sorting_observe_service", "/sorting/move_to_observation"
        )
        self.prepare_service_name = rospy.get_param(
            "~sorting_prepare_service", "/sorting/prepare_work"
        )
        self.sort_service_name = rospy.get_param(
            "~sorting_start_service", "/sorting/start"
        )
        self.sorting_stop_service_name = rospy.get_param(
            "~sorting_stop_service", "/sorting/stop"
        )
        self.sorting_failure_topic = rospy.get_param(
            "~sorting_failure_topic", "/sorting/failure"
        )
        self.sorting_configure_service_name = rospy.get_param(
            "~sorting_configure_service", "/sorting/configure_workspace"
        )
        self.sorting_workspace_param = rospy.get_param(
            "~sorting_workspace_param", "/sorting/workspace_config"
        )

        self.navigation_client = actionlib.SimpleActionClient(
            self.navigation_action, MoveBaseAction
        )
        self.tf_listener = tf.TransformListener()
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        self.home_client = rospy.ServiceProxy(self.home_service_name, Trigger)
        self.prepare_client = rospy.ServiceProxy(self.prepare_service_name, Trigger)
        self.observe_client = rospy.ServiceProxy(self.observe_service_name, Trigger)
        self.sort_client = rospy.ServiceProxy(self.sort_service_name, Trigger)
        self.sorting_stop_client = rospy.ServiceProxy(
            self.sorting_stop_service_name, Trigger
        )
        self.configure_workspace_client = rospy.ServiceProxy(
            self.sorting_configure_service_name, Trigger
        )

        self._condition = threading.Condition()
        self._sorting_state = ""
        self._sorting_sequence = 0
        self._sorting_failure = ""
        self._busy = False
        self._stop_requested = threading.Event()

        # dynamic_reconfigure uses scalar parameters. Seed those parameters from
        # the list-based scenario file before constructing the server so launch
        # overrides and YAML remain the source of the initial values.
        self._seed_dynamic_parameters()
        self.dynamic_server = Server(
            NavSortingConfig, self._reconfigure_callback
        )

        self.state_publisher = rospy.Publisher(
            "/nav_sorting/state", String, queue_size=1, latch=True
        )
        self.workspace_publisher = rospy.Publisher(
            "/nav_sorting/current_workstation", String, queue_size=1, latch=True
        )
        self.base_recovery_publisher = rospy.Publisher(
            self.base_recovery_topic, Twist, queue_size=2
        )
        self.sorting_subscriber = rospy.Subscriber(
            self.sorting_state_topic, String, self._sorting_state_callback, queue_size=5
        )
        self.sorting_failure_subscriber = rospy.Subscriber(
            self.sorting_failure_topic,
            String,
            self._sorting_failure_callback,
            queue_size=5,
        )
        self.start_service = rospy.Service(
            "/nav_sorting/start", Trigger, self._start_callback
        )
        self.stop_service = rospy.Service(
            "/nav_sorting/stop", Trigger, self._stop_callback
        )
        self._publish_state("INITIALIZING", "waiting for navigation and sorting")

        if self.auto_start:
            rospy.Timer(rospy.Duration(max(0.1, self.startup_delay)), self._auto_start, oneshot=True)
        else:
            self._publish_state("IDLE", "call /nav_sorting/start")

    @staticmethod
    def _float_list(parameter, default):
        values = rospy.get_param(parameter, default)
        if not isinstance(values, list) or not values:
            raise ValueError("{} must be a non-empty list".format(parameter))
        return [float(value) for value in values]

    @staticmethod
    def _validate_workstations(workstations):
        identifiers = set()
        for index, workspace in enumerate(workstations):
            if not isinstance(workspace, dict):
                raise ValueError("workstations[{}] must be a mapping".format(index))
            identifier = str(workspace.get("id", "")).strip()
            if not identifier or identifier in identifiers:
                raise ValueError("each workstation needs a unique non-empty id")
            identifiers.add(identifier)
            goal = workspace.get("navigation_goal")
            if not isinstance(goal, list) or len(goal) != 3:
                raise ValueError(
                    "workstation '{}' navigation_goal must be [x, y, yaw]".format(
                        identifier
                    )
                )
            for key, size in (("table_center", 3), ("table_size", 3)):
                value = workspace.get(key)
                if not isinstance(value, list) or len(value) != size:
                    raise ValueError(
                        "workstation '{}' {} must contain {} numbers".format(
                            identifier, key, size
                        )
                    )
            objects = workspace.get("objects", [])
            if not isinstance(objects, list):
                raise ValueError(
                    "workstation '{}' objects must be a list".format(identifier)
                )
            for item in objects:
                if (
                    not isinstance(item, dict)
                    or not str(item.get("id", "")).strip()
                    or not str(item.get("color", "")).strip()
                    or not isinstance(item.get("position"), list)
                    or len(item["position"]) != 3
                ):
                    raise ValueError(
                        "workstation '{}' has an invalid known object".format(
                            identifier
                        )
                    )

    @staticmethod
    def _validate_recovery_steps(steps):
        if not isinstance(steps, list):
            raise ValueError("~base_recovery_steps must be a list")
        for step in steps:
            if not isinstance(step, list) or len(step) != 2:
                raise ValueError("each base recovery step must be [dx, dy]")
            dx, dy = float(step[0]), float(step[1])
            if abs(dx) > 1.0e-6 and abs(dy) > 1.0e-6:
                raise ValueError("base recovery steps must be horizontal or vertical")

    @classmethod
    def _pose_param(cls, parameter, default):
        values = cls._float_list(parameter, default)
        if len(values) != 3:
            raise ValueError("{} must be [x, y, yaw]".format(parameter))
        return values

    def _seed_dynamic_parameters(self):
        initial_values = {
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            "goal_yaw": self.goal_yaw,
            "pre_dock_x": self.pre_dock_goal[0],
            "pre_dock_y": self.pre_dock_goal[1],
            "pre_dock_yaw": self.pre_dock_goal[2],
        }
        for name, value in initial_values.items():
            parameter = "~" + name
            if not rospy.has_param(parameter):
                rospy.set_param(parameter, value)

    def _current_dynamic_values(self):
        return {
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            "goal_yaw": self.goal_yaw,
            "near_field_enabled": self.near_field_enabled,
            "pre_dock_x": self.pre_dock_goal[0],
            "pre_dock_y": self.pre_dock_goal[1],
            "pre_dock_yaw": self.pre_dock_goal[2],
            "near_field_base_clearance": self.base_clearance,
            "near_field_max_candidates": self.near_field_max_candidates,
            "navigation_timeout": self.navigation_timeout,
            "navigation_retries": self.navigation_retries,
            "server_timeout": self.server_timeout,
            "sorting_initialization_timeout": self.initialization_timeout,
            "sorting_operation_timeout": self.operation_timeout,
            "home_before_navigation": self.home_before_navigation,
        }

    def _reconfigure_callback(self, config, _level):
        with self._condition:
            if self._busy:
                # A mission is a single safety-critical transaction. Reject
                # mid-run changes so its target and timeout policy stay stable.
                for name, value in self._current_dynamic_values().items():
                    config[name] = value
                rospy.logwarn_throttle(
                    2.0,
                    "nav_sorting parameters cannot change while a mission is running",
                )
                return config

            self.goal_x = float(config["goal_x"])
            self.goal_y = float(config["goal_y"])
            self.goal_yaw = float(config["goal_yaw"])
            self.near_field_enabled = bool(config["near_field_enabled"])
            self.pre_dock_goal = [
                float(config["pre_dock_x"]),
                float(config["pre_dock_y"]),
                float(config["pre_dock_yaw"]),
            ]
            self.base_clearance = float(config["near_field_base_clearance"])
            self.near_field_max_candidates = int(
                config["near_field_max_candidates"]
            )
            self.navigation_timeout = float(config["navigation_timeout"])
            self.navigation_retries = int(config["navigation_retries"])
            self.server_timeout = float(config["server_timeout"])
            self.initialization_timeout = float(
                config["sorting_initialization_timeout"]
            )
            self.operation_timeout = float(config["sorting_operation_timeout"])
            self.home_before_navigation = bool(config["home_before_navigation"])
        rospy.loginfo(
            "Updated nav_sorting parameters: goal=[%.3f, %.3f, %.3f], "
            "near_field=%s, navigation_timeout=%.1f, retries=%d",
            self.goal_x,
            self.goal_y,
            self.goal_yaw,
            self.near_field_enabled,
            self.navigation_timeout,
            self.navigation_retries,
        )
        return config

    def _publish_state(self, state, detail=""):
        message = state if not detail else "{} | {}".format(state, detail)
        self.state_publisher.publish(String(data=message))
        rospy.loginfo("Navigation-sorting mission: %s", message)

    def _sorting_state_callback(self, message):
        with self._condition:
            self._sorting_state = message.data
            self._sorting_sequence += 1
            self._condition.notify_all()

    def _sorting_failure_callback(self, message):
        with self._condition:
            self._sorting_failure = message.data
            self._condition.notify_all()

    def _auto_start(self, _event):
        self._submit_mission()

    def _start_callback(self, _request):
        accepted, message = self._submit_mission()
        return TriggerResponse(success=accepted, message=message)

    def _submit_mission(self):
        with self._condition:
            if self._busy:
                return False, "a mission is already running"
            self._busy = True
            self._stop_requested.clear()
        worker = threading.Thread(target=self._run_mission)
        worker.daemon = True
        worker.start()
        return True, "mission accepted"

    def _stop_callback(self, _request):
        with self._condition:
            was_busy = self._busy
        if not was_busy:
            return TriggerResponse(success=True, message="no mission is running")
        self._stop_requested.set()
        self.navigation_client.cancel_all_goals()
        self._stop_base()
        try:
            self.sorting_stop_client()
        except rospy.ServiceException as error:
            rospy.logwarn("Could not stop sorting service: %s", str(error))
        self._publish_state("STOPPING", "cancelling active operation")
        return TriggerResponse(success=True, message="stop requested")

    def _wait_for_sorting_ready(self):
        deadline = time.time() + self.initialization_timeout
        with self._condition:
            while not rospy.is_shutdown() and time.time() < deadline:
                state = self._sorting_state.split("|", 1)[0].strip()
                if state in ("IDLE", "READY", "STOPPED"):
                    return True
                if state == "ERROR":
                    rospy.logerr("Sorting node initialization failed: %s", self._sorting_state)
                    return False
                self._condition.wait(timeout=0.2)
        rospy.logerr("Timed out waiting for sorting node readiness")
        return False

    def _call_sorting_operation(self, client, running_states, label):
        if self._stop_requested.is_set():
            return False
        with self._condition:
            start_sequence = self._sorting_sequence
        try:
            response = client()
        except rospy.ServiceException as error:
            rospy.logerr("%s service call failed: %s", label, str(error))
            return False
        if not response.success:
            rospy.logerr("%s command rejected: %s", label, response.message)
            return False

        deadline = time.time() + self.operation_timeout
        saw_operation = False
        with self._condition:
            while not rospy.is_shutdown() and time.time() < deadline:
                state = self._sorting_state.split("|", 1)[0].strip()
                if self._sorting_sequence > start_sequence:
                    if state in running_states:
                        saw_operation = True
                    elif state == "READY" and (
                        saw_operation or self._sorting_sequence > start_sequence + 1
                    ):
                        return True
                    elif state in ("ERROR", "STOPPED"):
                        rospy.logerr("%s failed: %s", label, self._sorting_state)
                        return False
                if self._stop_requested.is_set():
                    return False
                self._condition.wait(timeout=0.2)
        rospy.logerr("%s timed out after %.1f seconds", label, self.operation_timeout)
        return False

    def _planning_failed(self):
        with self._condition:
            return self._sorting_failure.startswith("PLANNING_FAILED")

    def _configure_workspace(self, workspace):
        payload = dict(workspace)
        payload.pop("navigation_goal", None)
        payload.pop("pre_dock_goal", None)
        payload.pop("enabled", None)
        rospy.set_param(self.sorting_workspace_param, payload)
        self.workspace_publisher.publish(
            String(data=json.dumps(workspace, sort_keys=True))
        )
        try:
            rospy.wait_for_service(
                self.sorting_configure_service_name, timeout=self.server_timeout
            )
            response = self.configure_workspace_client()
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr("Cannot configure workstation: %s", str(error))
            return False
        if not response.success:
            rospy.logerr("Workstation configuration rejected: %s", response.message)
            return False
        return True

    def _stop_base(self):
        self.base_recovery_publisher.publish(Twist())

    def _move_base_direct(self, step, attempt, total):
        dx, dy = float(step[0]), float(step[1])
        distance = max(abs(dx), abs(dy))
        if distance <= 1.0e-6:
            return True
        if self.base_recovery_speed <= 1.0e-6:
            rospy.logerr("base_recovery_speed must be greater than zero")
            return False

        self.navigation_client.cancel_all_goals()
        duration = distance / self.base_recovery_speed
        command = Twist()
        command.linear.x = math.copysign(self.base_recovery_speed, dx) if abs(dx) > 1.0e-6 else 0.0
        command.linear.y = math.copysign(self.base_recovery_speed, dy) if abs(dy) > 1.0e-6 else 0.0
        self._publish_state(
            "ADJUSTING_BASE",
            "cmd_vel step {}/{} dx={:.3f} dy={:.3f}".format(
                attempt, total, dx, dy
            ),
        )
        rate = rospy.Rate(self.base_recovery_rate)
        deadline = time.time() + duration
        try:
            while not rospy.is_shutdown() and time.time() < deadline:
                if self._stop_requested.is_set():
                    return False
                self.base_recovery_publisher.publish(command)
                rate.sleep()
        finally:
            self._stop_base()
        if self.base_recovery_settle_time > 0.0:
            rospy.sleep(self.base_recovery_settle_time)
        return not self._stop_requested.is_set()

    @staticmethod
    def _angle_error(target, actual):
        return math.atan2(math.sin(target - actual), math.cos(target - actual))

    def _current_base_pose(self):
        try:
            self.tf_listener.waitForTransform(
                self.navigation_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            translation, rotation = self.tf_listener.lookupTransform(
                self.navigation_frame, self.base_frame, rospy.Time(0)
            )
            yaw = euler_from_quaternion(rotation)[2]
            return [translation[0], translation[1], yaw]
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException, tf.Exception) as error:
            rospy.logwarn("Cannot read base pose for direct docking: %s", str(error))
            return None

    def _direct_dock_geometry(self, start, target):
        dx = target[0] - start[0]
        dy = target[1] - start[1]
        cosine = math.cos(start[2])
        sine = math.sin(start[2])
        longitudinal = cosine * dx + sine * dy
        lateral = -sine * dx + cosine * dy
        yaw_error = self._angle_error(target[2], start[2])
        return longitudinal, lateral, yaw_error

    def _can_direct_dock_from_pre_dock(self, candidate):
        longitudinal, lateral, yaw_error = self._direct_dock_geometry(
            self.pre_dock_goal, candidate
        )
        return (
            0.0 < longitudinal <= self.direct_dock_max_distance
            and abs(lateral) <= self.direct_dock_lateral_tolerance
            and abs(yaw_error) <= self.direct_dock_yaw_tolerance
        )

    def _drive_straight_to(self, target, label):
        start = self._current_base_pose()
        if start is None:
            return False
        longitudinal, lateral, yaw_error = self._direct_dock_geometry(start, target)
        if (
            abs(longitudinal) > self.direct_dock_max_distance
            or abs(lateral) > self.direct_dock_lateral_tolerance
            or abs(yaw_error) > self.direct_dock_yaw_tolerance
        ):
            rospy.logwarn(
                "%s is not a straight motion: forward=%.3f lateral=%.3f yaw=%.3f",
                label, longitudinal, lateral, yaw_error,
            )
            return False

        self._publish_state(
            "DIRECT_DOCKING",
            "%s %.3f m at %.3f m/s through lidar safety".format(
                label, longitudinal, self.base_recovery_speed
            ),
        )
        if not self._move_base_direct([longitudinal, 0.0], 1, 1):
            return False

        actual = self._current_base_pose()
        if actual is None:
            return False
        position_error = math.hypot(actual[0] - target[0], actual[1] - target[1])
        actual_yaw_error = abs(self._angle_error(target[2], actual[2]))
        if (
            position_error > self.direct_dock_goal_tolerance
            or actual_yaw_error > self.direct_dock_yaw_tolerance
        ):
            rospy.logwarn(
                "%s stopped short: position error=%.3f yaw error=%.3f",
                label, position_error, actual_yaw_error,
            )
            return False
        rospy.loginfo(
            "%s reached by straight cmd_vel: position error=%.3f yaw error=%.3f",
            label, position_error, actual_yaw_error,
        )
        return True

    def _prepare_and_observe_once(self):
        self._publish_state("PREPARING_ARM", "moving arm to work-ready pose")
        if not self._call_sorting_operation(
            self.prepare_client, ("PREPARING",), "arm work preparation"
        ):
            return False
        self._publish_state("VALIDATING_DOCK", "planning observation pose")
        return self._call_sorting_operation(
            self.observe_client, ("OBSERVING",), "camera observation"
        )

    def _stow_for_base_recovery(self):
        self._publish_state(
            "STOWING_ARM", "making cmd_vel recovery motion safe"
        )
        return self._call_sorting_operation(
            self.home_client, ("HOMING",), "arm stow before base recovery"
        )

    def _prepare_and_observe_with_recovery(self):
        if self._prepare_and_observe_once():
            return True
        if not self.base_recovery_enabled or not self._planning_failed():
            return False
        total = len(self.base_recovery_steps)
        for index, step in enumerate(self.base_recovery_steps):
            if not self._stow_for_base_recovery():
                return False
            if not self._move_base_direct(step, index + 1, total):
                return False
            if self._prepare_and_observe_once():
                return True
            if not self._planning_failed():
                return False
        rospy.logerr("No direct base adjustment produced a valid observation plan")
        return False

    def _sort_with_recovery(self):
        if self._call_sorting_operation(
            self.sort_client,
            ("SORTING", "DETECTING", "PICKING", "OBSERVING", "HOMING"),
            "sorting",
        ):
            return True
        if not self.base_recovery_enabled or not self._planning_failed():
            return False
        total = len(self.base_recovery_steps)
        for index, step in enumerate(self.base_recovery_steps):
            if not self._stow_for_base_recovery():
                return False
            if not self._move_base_direct(step, index + 1, total):
                return False
            if not self._prepare_and_observe_once():
                if self._planning_failed():
                    continue
                return False
            if self._call_sorting_operation(
                self.sort_client,
                ("SORTING", "DETECTING", "PICKING", "OBSERVING", "HOMING"),
                "sorting retry",
            ):
                return True
            if not self._planning_failed():
                return False
        rospy.logerr("No direct base adjustment produced a valid sorting plan")
        return False

    def _navigate_once(self, target):
        target_x, target_y, target_yaw = target
        quaternion = quaternion_from_euler(0.0, 0.0, target_yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.navigation_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        self.navigation_client.send_goal(goal)

        deadline = time.time() + self.navigation_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self._stop_requested.is_set():
                self.navigation_client.cancel_goal()
                return False
            if self.navigation_client.wait_for_result(rospy.Duration(0.2)):
                return self.navigation_client.get_state() == GoalStatus.SUCCEEDED
        self.navigation_client.cancel_goal()
        return False

    def _navigate(self, target, stage="navigation"):
        if not self.navigation_client.wait_for_server(rospy.Duration(self.server_timeout)):
            rospy.logerr("Navigation action %s is unavailable", self.navigation_action)
            return False
        for attempt in range(self.navigation_retries + 1):
            self._publish_state(
                "NAVIGATING",
                "{} attempt {}/{} to [{:.2f}, {:.2f}, {:.2f}]".format(
                    stage,
                    attempt + 1,
                    self.navigation_retries + 1,
                    target[0],
                    target[1],
                    target[2],
                ),
            )
            if self._navigate_once(target):
                return True
            if self._stop_requested.is_set():
                return False
            rospy.logwarn("Navigation attempt %d failed", attempt + 1)
            try:
                self.clear_costmaps()
            except rospy.ServiceException as error:
                rospy.logwarn("Could not clear costmaps before retry: %s", str(error))
        return False

    @staticmethod
    def _point_in_base(point, base_pose):
        dx = float(point[0]) - base_pose[0]
        dy = float(point[1]) - base_pose[1]
        cosine = math.cos(base_pose[2])
        sine = math.sin(base_pose[2])
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    def _table_clearance(self, base_pose):
        table_x, table_y, size_x, size_y = self.table_geometry
        dx = max(abs(base_pose[0] - table_x) - 0.5 * size_x, 0.0)
        dy = max(abs(base_pose[1] - table_y) - 0.5 * size_y, 0.0)
        return math.hypot(dx, dy)

    def _score_candidate(self, candidate):
        clearance = self._table_clearance(candidate)
        if clearance < self.base_clearance:
            return None

        min_x, max_x, min_y, max_y = self.detector_workspace
        camera_x, camera_y = self.camera_target
        camera_error = 0.0
        for point in self.workpiece_points:
            local_x, local_y = self._point_in_base(point, candidate)
            if not (min_x <= local_x <= max_x and min_y <= local_y <= max_y):
                return None
            camera_error += (local_x - camera_x) ** 2 + (local_y - camera_y) ** 2

        nominal_error = math.hypot(
            candidate[0] - self.goal_x, candidate[1] - self.goal_y
        )
        yaw_error = abs(candidate[2] - self.goal_yaw)
        # Camera centring dominates; the remaining terms prefer the configured
        # workstation pose and a larger table gap among similarly visible poses.
        return camera_error + 0.15 * nominal_error + 0.05 * yaw_error - 0.02 * clearance

    def _near_field_candidates(self):
        scored = []
        for x_value in self.candidate_x:
            for y_value in self.candidate_y:
                for yaw_value in self.candidate_yaw:
                    candidate = [x_value, y_value, yaw_value]
                    score = self._score_candidate(candidate)
                    if score is not None:
                        scored.append((score, candidate, self._table_clearance(candidate)))
        scored.sort(key=lambda item: item[0])
        if self.direct_dock_enabled:
            # Prefer candidates reachable by a straight, low-speed final
            # approach from pre-dock.  This avoids DWA rotation inside the
            # table's near field while retaining its score within each group.
            scored.sort(
                key=lambda item: (
                    not self._can_direct_dock_from_pre_dock(item[1]), item[0]
                )
            )
        selected = scored[:self.near_field_max_candidates]
        for index, (score, candidate, clearance) in enumerate(selected):
            rospy.loginfo(
                "Near-field candidate %d: [%.3f, %.3f, %.3f], "
                "table clearance=%.3f, score=%.5f",
                index + 1,
                candidate[0],
                candidate[1],
                candidate[2],
                clearance,
                score,
            )
        return [item[1] for item in selected]

    def _coordinate_near_field(self):
        if not self._navigate(self.pre_dock_goal, "pre-dock"):
            return False
        # Keep the arm in its transport pose while the base enters the table's
        # near field.  Prepare it only after the base has stopped at a candidate.
        arm_prepared = False
        candidates = self._near_field_candidates()
        if not candidates:
            rospy.logerr("No near-field pose satisfies base, camera and detector constraints")
            return False

        for index, candidate in enumerate(candidates):
            if self._stop_requested.is_set():
                return False
            self._publish_state(
                "COORDINATING",
                "candidate {}/{} [{:.2f}, {:.2f}, {:.2f}]".format(
                    index + 1,
                    len(candidates),
                    candidate[0],
                    candidate[1],
                    candidate[2],
                ),
            )
            used_direct_dock = (
                self.direct_dock_enabled
                and self._can_direct_dock_from_pre_dock(candidate)
            )
            if used_direct_dock:
                docked = self._drive_straight_to(
                    candidate, "fine-dock candidate {}".format(index + 1)
                )
            else:
                docked = self._navigate(candidate, "fine-dock")
            if not docked:
                rospy.logwarn("Fine-dock candidate %d motion failed", index + 1)
                if used_direct_dock:
                    self._drive_straight_to(self.pre_dock_goal, "pre-dock retreat")
                continue

            if not arm_prepared:
                self._publish_state("PREPARING_ARM", "restoring work-ready pose")
                if not self._call_sorting_operation(
                    self.prepare_client, ("PREPARING",), "arm work preparation"
                ):
                    return False
                arm_prepared = True

            self._publish_state(
                "VALIDATING_DOCK", "planning observation and checking colors"
            )
            if self._call_sorting_operation(
                self.observe_client, ("OBSERVING",), "camera observation"
            ):
                rospy.loginfo("Near-field candidate %d accepted", index + 1)
                return True

            rospy.logwarn(
                "Near-field candidate %d rejected; returning arm to transport",
                index + 1,
            )
            if not self._call_sorting_operation(
                self.home_client, ("HOMING",), "arm re-stowing"
            ):
                return False
            arm_prepared = False
            if used_direct_dock and not self._drive_straight_to(
                self.pre_dock_goal, "pre-dock retreat"
            ):
                rospy.logerr("Could not retreat safely after rejected direct dock")
                return False
        rospy.logerr("All near-field docking candidates failed validation")
        return False

    def _run_workstation_sequence(self):
        enabled = [
            workspace
            for workspace in self.workstations
            if bool(workspace.get("enabled", True))
        ]
        if not enabled:
            rospy.logerr("No enabled workstation is configured")
            return False

        for index, workspace in enumerate(enabled):
            identifier = str(workspace["id"])
            self._publish_state(
                "CONFIGURING_WORKSTATION",
                "{}/{} {}".format(index + 1, len(enabled), identifier),
            )
            if self.home_before_navigation or index > 0:
                self._publish_state(
                    "STOWING_ARM", "{} before navigation".format(identifier)
                )
                if not self._call_sorting_operation(
                    self.home_client, ("HOMING",), "arm homing"
                ):
                    return False
            if not self._configure_workspace(workspace):
                return False

            goal = [float(value) for value in workspace["navigation_goal"]]
            if not self._navigate(goal, "workstation '{}'".format(identifier)):
                return False
            self._publish_state(
                "AT_WORKSTATION", "{}; validating arm reach".format(identifier)
            )
            if not self._prepare_and_observe_with_recovery():
                return False
            self._publish_state("SORTING", "workstation '{}'".format(identifier))
            if not self._sort_with_recovery():
                return False
            self._publish_state(
                "WORKSTATION_COMPLETE", "{} ({}/{})".format(
                    identifier, index + 1, len(enabled)
                )
            )
        return True

    def _run_mission(self):
        success = False
        try:
            if not self._wait_for_sorting_ready():
                return
            if self.workstations:
                success = self._run_workstation_sequence()
                return
            if self.home_before_navigation:
                self._publish_state("STOWING_ARM", "moving arm to navigation pose")
                if not self._call_sorting_operation(
                    self.home_client, ("HOMING",), "arm homing"
                ):
                    return
            if self.near_field_enabled:
                if not self._coordinate_near_field():
                    return
            else:
                if not self._navigate(
                    [self.goal_x, self.goal_y, self.goal_yaw], "workstation"
                ):
                    return
                self._publish_state("AT_WORKSTATION", "starting camera observation")
                if not self._prepare_and_observe_with_recovery():
                    return
            self._publish_state("AT_WORKSTATION", "dock and camera view validated")
            self._publish_state("SORTING", "sorting detected objects")
            if not self._sort_with_recovery():
                return
            success = True
        except Exception as error:
            rospy.logerr("Navigation-sorting mission failed: %s", str(error))
        finally:
            self._stop_base()
            with self._condition:
                self._busy = False
            if self._stop_requested.is_set():
                self._publish_state("STOPPED", "mission cancelled")
            elif success:
                self._publish_state("SUCCEEDED", "navigation and sorting complete")
            else:
                self._publish_state("FAILED", "inspect move_base and /sorting/state")


def main():
    rospy.init_node("nav_sorting_mission")
    NavigationSortingMission()
    rospy.spin()


if __name__ == "__main__":
    main()
