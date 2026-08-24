#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler


class NavigationSortingMission(object):
    """Coordinate a safe base-navigation -> observation -> sorting mission."""

    def __init__(self):
        self.navigation_action = rospy.get_param("~navigation_action", "/move_base")
        self.navigation_frame = rospy.get_param("~navigation_frame", "map")
        goal = rospy.get_param("~sorting_goal", [2.15, 0.0, 0.0])
        if not isinstance(goal, list) or len(goal) != 3:
            raise ValueError("~sorting_goal must be [x, y, yaw]")
        self.goal_x, self.goal_y, self.goal_yaw = [float(value) for value in goal]

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

        self.sorting_state_topic = rospy.get_param(
            "~sorting_state_topic", "/sorting/state"
        )
        self.home_service_name = rospy.get_param(
            "~sorting_home_service", "/sorting/home"
        )
        self.observe_service_name = rospy.get_param(
            "~sorting_observe_service", "/sorting/move_to_observation"
        )
        self.sort_service_name = rospy.get_param(
            "~sorting_start_service", "/sorting/start"
        )
        self.sorting_stop_service_name = rospy.get_param(
            "~sorting_stop_service", "/sorting/stop"
        )

        self.navigation_client = actionlib.SimpleActionClient(
            self.navigation_action, MoveBaseAction
        )
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        self.home_client = rospy.ServiceProxy(self.home_service_name, Trigger)
        self.observe_client = rospy.ServiceProxy(self.observe_service_name, Trigger)
        self.sort_client = rospy.ServiceProxy(self.sort_service_name, Trigger)
        self.sorting_stop_client = rospy.ServiceProxy(
            self.sorting_stop_service_name, Trigger
        )

        self._condition = threading.Condition()
        self._sorting_state = ""
        self._sorting_sequence = 0
        self._busy = False
        self._stop_requested = threading.Event()

        self.state_publisher = rospy.Publisher(
            "/nav_sorting/state", String, queue_size=1, latch=True
        )
        self.sorting_subscriber = rospy.Subscriber(
            self.sorting_state_topic, String, self._sorting_state_callback, queue_size=5
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

    def _publish_state(self, state, detail=""):
        message = state if not detail else "{} | {}".format(state, detail)
        self.state_publisher.publish(String(data=message))
        rospy.loginfo("Navigation-sorting mission: %s", message)

    def _sorting_state_callback(self, message):
        with self._condition:
            self._sorting_state = message.data
            self._sorting_sequence += 1
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

    def _navigate_once(self):
        quaternion = quaternion_from_euler(0.0, 0.0, self.goal_yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.navigation_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.goal_x
        goal.target_pose.pose.position.y = self.goal_y
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

    def _navigate(self):
        if not self.navigation_client.wait_for_server(rospy.Duration(self.server_timeout)):
            rospy.logerr("Navigation action %s is unavailable", self.navigation_action)
            return False
        for attempt in range(self.navigation_retries + 1):
            self._publish_state(
                "NAVIGATING",
                "attempt {}/{} to [{:.2f}, {:.2f}, {:.2f}]".format(
                    attempt + 1,
                    self.navigation_retries + 1,
                    self.goal_x,
                    self.goal_y,
                    self.goal_yaw,
                ),
            )
            if self._navigate_once():
                return True
            if self._stop_requested.is_set():
                return False
            rospy.logwarn("Navigation attempt %d failed", attempt + 1)
            try:
                self.clear_costmaps()
            except rospy.ServiceException as error:
                rospy.logwarn("Could not clear costmaps before retry: %s", str(error))
        return False

    def _run_mission(self):
        success = False
        try:
            if not self._wait_for_sorting_ready():
                return
            if self.home_before_navigation:
                self._publish_state("STOWING_ARM", "moving arm to navigation pose")
                if not self._call_sorting_operation(
                    self.home_client, ("HOMING",), "arm homing"
                ):
                    return
            if not self._navigate():
                return
            self._publish_state("AT_WORKSTATION", "starting camera observation")
            if not self._call_sorting_operation(
                self.observe_client, ("OBSERVING",), "camera observation"
            ):
                return
            self._publish_state("SORTING", "sorting detected objects")
            if not self._call_sorting_operation(
                self.sort_client,
                ("SORTING", "DETECTING", "PICKING", "OBSERVING", "HOMING"),
                "sorting",
            ):
                return
            success = True
        except Exception as error:
            rospy.logerr("Navigation-sorting mission failed: %s", str(error))
        finally:
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
