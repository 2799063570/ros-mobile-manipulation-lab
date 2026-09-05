#!/usr/bin/env python3
"""Run without ROS: python -m unittest discover -s test -v.

Exercise the real Python mission methods with fake ROS transport, clock and TF.
These tests do not replace the Gazebo acceptance checks in README.md.
"""
import importlib.util
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


MODULES = (
    "actionlib", "rospy", "tf", "tf.transformations", "actionlib_msgs",
    "actionlib_msgs.msg", "dynamic_reconfigure", "dynamic_reconfigure.server",
    "geometry_msgs", "geometry_msgs.msg", "move_base_msgs", "move_base_msgs.msg",
    "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv",
    "aubo_mobile_nav_sorting", "aubo_mobile_nav_sorting.cfg",
)
with patch.dict(sys.modules, {name: MagicMock() for name in MODULES}):
    spec = importlib.util.spec_from_file_location(
        "mission_under_test", Path(__file__).resolve().parents[1] / "scripts/nav_sorting_mission.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)


class Stamp:
    def __init__(self, value=0):
        self.value = value

    def to_sec(self):
        return self.value

    def __sub__(self, other):
        return Stamp(self.value - other.value)

    @staticmethod
    def now():
        return Stamp(100.0)


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


def reply(success=True):
    return SimpleNamespace(success=success, message="test reply")


class MissionSafetyTests(unittest.TestCase):
    def setUp(self):
        module.rospy.reset_mock()
        module.rospy.is_shutdown.return_value = False
        module.rospy.Time = Stamp
        module.tf.LookupException = LookupError
        module.tf.ConnectivityException = ConnectionError
        module.tf.ExtrapolationException = ValueError
        module.tf.Exception = RuntimeError
        module.euler_from_quaternion = lambda _: [0.0, 0.0, 0.0]
        module.Twist = Twist
        module.TriggerResponse = lambda **values: SimpleNamespace(**values)
        m = module.NavigationSortingMission.__new__(module.NavigationSortingMission)
        self.m = m
        m._condition = threading.Condition()
        m._stop_requested = threading.Event()
        m._busy = True
        m._stop_unconfirmed = False
        m._operation_active = False
        m._base_pose_failed = False
        m._operation_start_sequence = 0
        m._operation_lock_sequence = 0
        m._base_lock_sequence = 0
        m._base_locked = False
        m._sorting_sequence = 0
        m._sorting_state = "IDLE"
        m._sorting_failure = ""
        m.initialization_timeout = 0.5
        m.operation_timeout = 0.05
        m.stop_timeout = 0.08
        m.server_timeout = 0.1
        m.sorting_stop_service_name = "/sorting/stop"
        m.sorting_stop_client = MagicMock(return_value=reply())
        m.navigation_client = MagicMock()
        m.base_recovery_publisher = MagicMock()
        m._publish_state = MagicMock()
        m.navigation_frame = "odom"
        m.base_frame = "base_footprint"
        m.tf_max_age = 0.5
        m.tf_listener = MagicMock()
        m.tf_listener.getLatestCommonTime.return_value = Stamp(99.9)
        m.tf_listener.lookupTransform.return_value = ([1, 2, 0], [0, 0, 0, 1])

    def state(self, state, unlocked=False):
        self.m._sorting_state_callback(SimpleNamespace(data=state))
        if unlocked:
            self.m._base_lock_callback(SimpleNamespace(data=False))

    def active_call(self):
        self.m._base_lock_callback(SimpleNamespace(data=True))
        self.state("SORTING")
        return reply()

    def finish_stop(self):
        self.state("STOPPED", unlocked=True)
        return reply()

    def run_single_sort(self, client):
        m = self.m
        m._wait_for_sorting_ready = lambda: True
        m.workstations = []
        m.home_before_navigation = False
        m.near_field_enabled = False
        m.goal_x = m.goal_y = m.goal_yaw = 0.0
        m._navigate = lambda *args: True
        m._prepare_and_observe_with_recovery = lambda: True
        m._sort_with_recovery = lambda: m._call_sorting_operation(client, ("SORTING",), "sort")
        m._run_mission()

    def test_stop_interrupts_initialization_without_waiting_for_timeout(self):
        self.m._sorting_state = "INITIALIZING"
        result = []
        worker = threading.Thread(target=lambda: result.append(self.m._wait_for_sorting_ready()))
        worker.start()
        self.m._stop_callback(None)
        worker.join(0.3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [False])

    def test_stop_callback_does_not_call_blocking_sorting_service(self):
        self.m.sorting_stop_client.side_effect = AssertionError("must run in mission cleanup")
        result = self.m._stop_callback(None)
        self.assertTrue(result.success)
        self.assertTrue(self.m._stop_requested.is_set())
        self.m.sorting_stop_client.assert_not_called()
        self.assert_zero_velocity()

    def test_operation_timeout_cancels_before_reporting_failure(self):
        self.m.sorting_stop_client.side_effect = self.finish_stop
        self.run_single_sort(self.active_call)
        self.m.sorting_stop_client.assert_called_once()
        states = [c.args[0] for c in self.m._publish_state.call_args_list]
        self.assertLess(states.index("STOPPING"), states.index("FAILED"))
        self.assertFalse(self.m._operation_active)
        self.assertFalse(self.m._busy)
        self.assertFalse(self.m._stop_unconfirmed)

    def test_user_stop_cancels_active_operation(self):
        def start():
            self.active_call()
            self.m._stop_callback(None)
            return reply()
        self.m.sorting_stop_client.side_effect = self.finish_stop
        self.run_single_sort(start)
        self.assertEqual(self.m._publish_state.call_args.args[0], "STOPPED")
        self.m.sorting_stop_client.assert_called_once()

    def test_stop_without_terminal_acknowledgement_blocks_restart(self):
        self.run_single_sort(self.active_call)
        self.assertTrue(self.m._stop_unconfirmed)
        self.assertEqual(self.m._publish_state.call_args.args[0], "STOP_UNCONFIRMED")
        self.assertFalse(self.m._submit_mission()[0])

    def test_old_ready_message_is_not_stop_confirmation(self):
        self.m._operation_active = True
        self.m._sorting_state = "READY"
        self.assertFalse(self.m._cancel_sorting_and_wait())
        self.assertTrue(self.m._stop_unconfirmed)

    def test_error_before_core_cleanup_is_not_stop_confirmation(self):
        self.m._operation_active = True
        self.state("ERROR")
        self.m._base_lock_callback(SimpleNamespace(data=True))
        self.assertFalse(self.m._cancel_sorting_and_wait())

    def test_success_needs_terminal_state_and_fresh_unlock(self):
        def complete():
            self.active_call()
            self.state("READY", unlocked=True)
            return reply()
        self.assertTrue(self.m._call_sorting_operation(complete, ("SORTING",), "sort"))
        self.assertFalse(self.m._operation_active)
        self.m.sorting_stop_client.assert_not_called()

    def test_late_service_response_is_cancelled_again_and_stays_interlocked(self):
        release = threading.Event()
        late_stop = threading.Event()
        def blocked_start():
            release.wait(2.0)
            return reply()
        module.rospy.ServiceProxy.return_value = lambda: late_stop.set()
        try:
            start = time.monotonic()
            self.assertFalse(self.m._call_sorting_operation(blocked_start, ("SORTING",), "sort"))
            self.assertLess(time.monotonic() - start, 0.5)
            self.assertTrue(self.m._stop_unconfirmed)
        finally:
            release.set()
        self.assertTrue(late_stop.wait(0.5))
        self.m._busy = False
        self.assertFalse(self.m._submit_mission()[0])

    def test_uncancelled_operation_cannot_launch_recovery(self):
        self.m._operation_active = True
        self.m._sorting_failure = "PLANNING_FAILED | old failure"
        other = MagicMock()
        self.assertFalse(self.m._planning_failed())
        self.assertFalse(self.m._call_sorting_operation(other, ("HOMING",), "home"))
        other.assert_not_called()

    def assert_zero_velocity(self):
        command = self.m.base_recovery_publisher.publish.call_args.args[0]
        self.assertEqual((command.linear.x, command.linear.y, command.angular.z), (0, 0, 0))

    def test_fresh_tf_is_accepted(self):
        self.assertEqual(self.m._current_base_pose(), [1, 2, 0])
        self.m.base_recovery_publisher.publish.assert_not_called()

    def test_stale_future_and_zero_tf_stop_immediately(self):
        for stamp in (98.0, 101.0, 0.0):
            with self.subTest(stamp=stamp):
                self.m.tf_listener.getLatestCommonTime.return_value = Stamp(stamp)
                self.assertIsNone(self.m._current_base_pose())
                self.assert_zero_velocity()
                self.assertTrue(self.m._base_pose_failed)

    def test_missing_tf_stops_immediately(self):
        self.m.tf_listener.getLatestCommonTime.side_effect = LookupError("TF lost")
        self.assertIsNone(self.m._current_base_pose())
        self.assert_zero_velocity()

    def test_invalid_tf_coordinates_stop_immediately(self):
        self.m.tf_listener.lookupTransform.return_value = ([math.nan, 2, 0], [0, 0, 0, 1])
        self.assertIsNone(self.m._current_base_pose())
        self.assert_zero_velocity()

    def test_heading_motion_stops_on_tf_loss_after_nonzero_command(self):
        m = self.m
        m.heading_goal_tolerance = 0.015
        m.heading_alignment_enabled = True
        m.heading_max_correction = 0.12
        m.heading_timeout = 0.5
        m.heading_stall_timeout = 0.4
        m.heading_speed = 0.12
        m.base_recovery_rate = 1000.0
        m._current_base_pose = MagicMock(side_effect=[[0, 0, 0], [0, 0, 0], None])
        self.assertFalse(m._align_heading_for_direct_dock(0.08, "test heading"))
        commands = [c.args[0] for c in m.base_recovery_publisher.publish.call_args_list]
        self.assertTrue(any(command.angular.z > 0 for command in commands))
        self.assert_zero_velocity()

    def test_blocked_stop_rpc_has_bounded_wait(self):
        release = threading.Event()
        returned = threading.Event()
        def blocked_stop():
            release.wait(2.0)
            returned.set()
            return reply()
        self.m.sorting_stop_client.side_effect = blocked_stop
        try:
            start = time.monotonic()
            self.assertFalse(self.m._cancel_sorting_and_wait())
            self.assertLess(time.monotonic() - start, 0.5)
            self.assertTrue(self.m._stop_unconfirmed)
        finally:
            release.set()
            self.assertTrue(returned.wait(0.5))

    def test_tf_fault_prevents_candidate_retreat_or_recovery_motion(self):
        self.m._base_pose_failed = True
        self.assertFalse(self.m._drive_straight_to([0, 0, 0], "retreat"))
        self.assertFalse(self.m._move_base_direct([0.05, 0], 1, 1))
        self.assertFalse(self.m._navigate_once([0, 0, 0]))
        self.m.navigation_client.send_goal.assert_not_called()
        self.assert_zero_velocity()


if __name__ == "__main__":
    unittest.main()
