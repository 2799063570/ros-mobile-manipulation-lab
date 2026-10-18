"""Known-geometry and mocked MoveIt checks for the one-shot verification tool."""
import importlib.util
import math
import sys
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('handeye_yolo_verify',
                                            ROOT/'scripts/handeye_yolo_verify.py')
verify = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(verify)


def config():
    return verify.validate_config(yaml.safe_load((ROOT/'config/handeye_yolo_verify.yaml').read_text()))


class Duration:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value

    @staticmethod
    def from_sec(value):
        return Duration(value)


class Stamp:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    def to_nsec(self):
        return round(self.seconds*1e9)

    def __sub__(self, other):
        return Stamp(self.seconds-other.seconds)


def pose(xyz=(.5, 0, .4), q=(0, 0, 0, 1)):
    return NS(position=NS(**dict(zip('xyz', xyz))), orientation=NS(**dict(zip('xyzw', q))))


def pose_stamped():
    return NS(header=NS(frame_id='', stamp=None), pose=pose())


def trajectory():
    return NS(joint_trajectory=NS(header=NS(), joint_names=['j1', 'j2'], points=[
        NS(positions=[.02*i, 0.], velocities=[0., 0.], accelerations=[0., 0.],
           time_from_start=Duration(i)) for i in range(4)]))


class VerificationTest(unittest.TestCase):
    def bare_task(self):
        task = verify.HandeyeYoloVerify.__new__(verify.HandeyeYoloVerify)
        task.config = config()
        task.calibration = dict(parameters=dict(tracking_base_frame='camera_color_optical_frame'))
        task.cancel = threading.Event()
        task.lock, task.data_lock, task.dispatch_lock = threading.Lock(), threading.Lock(), threading.Lock()
        task.arm, task.executor = mock.Mock(), mock.Mock()
        task.publish_status, task.record = mock.Mock(), mock.Mock(return_value='evidence.json')
        return task

    def test_detection_rejects_ambiguity_staleness_wrong_frames_and_duplicates(self):
        from collections import deque
        task = self.bare_task()
        task.history, task.last_detection_stamp = deque(maxlen=300), -1
        obj = NS(class_name='can', color='', pose=pose((.6, 0, .125)), confidence=.9,
                 object_height=.05, depth_valid=True)
        clock = [10.]
        fake_ros = NS(Time=NS(now=lambda: Stamp(clock[0])))
        message = NS(header=NS(stamp=Stamp(9.9), frame_id='base_link'),
                     observation_valid=True, sensor_frame='camera_color_optical_frame', objects=[obj])
        with mock.patch.object(verify, 'rospy', fake_ros, create=True):
            task.detections(message)
            task.detections(message)
            self.assertEqual(len(task.history), 1)
            message.header.stamp = Stamp(10.)
            message.objects = [obj, obj]
            task.detections(message)
            self.assertEqual(len(task.history), 0)
            for invalid in ('wrong_frame', 'wrong_sensor', 'stale', 'invalid_observation', 'low_confidence'):
                clock[0] += 2.
                message.header.stamp = Stamp(clock[0])
                message.header.frame_id = 'camera' if invalid == 'wrong_frame' else 'base_link'
                message.sensor_frame = 'other_camera' if invalid == 'wrong_sensor' else 'camera_color_optical_frame'
                message.observation_valid = invalid != 'invalid_observation'
                message.objects = [obj]
                obj.confidence = .1 if invalid == 'low_confidence' else .9
                if invalid == 'stale':
                    message.header.stamp = Stamp(clock[0]-1)
                task.detections(message)
                self.assertEqual(len(task.history), 0, invalid)

    def test_stale_or_moving_robot_cannot_dispatch_and_stop_cancels_own_goal(self):
        task = self.bare_task()
        task.active_joints, task.joints = ['j1'], {}
        task.robot = mock.Mock()
        state = NS(joint_state=NS(name=['j1'], position=[0.], header=NS()), is_diff=False)
        task.robot.get_current_state.return_value = state
        fake_ros = NS(Time=mock.Mock(return_value=None), is_shutdown=lambda: False)
        fake_ros.Time.now.return_value = Stamp(10.)
        modules = {'std_srvs': NS(), 'std_srvs.srv': NS(TriggerResponse=lambda **kw: NS(**kw)),
                   'moveit_msgs': NS(), 'moveit_msgs.msg': NS(ExecuteTrajectoryGoal=lambda **kw: NS(**kw))}
        with mock.patch.object(verify, 'rospy', fake_ros, create=True), mock.patch.dict(sys.modules, modules):
            with self.assertRaisesRegex(RuntimeError, 'feedback'):
                task.current_state()
            task.joints = {'j1': (0., 8., 0.)}
            with self.assertRaisesRegex(RuntimeError, 'feedback'):
                task.current_state()
            task.joints = {'j1': (0., 10., .1)}
            with self.assertRaisesRegex(RuntimeError, 'stationary'):
                task.current_state(stationary=True)
            task.joints = {'j1': (.1, 10., 0.)}
            with self.assertRaisesRegex(RuntimeError, 'moved'):
                task.execute(state, trajectory())
            task.executor.send_goal.assert_not_called()
            self.assertTrue(task.stop(None).success)
            self.assertTrue(task.cancel.is_set())
            task.executor.cancel_goal.assert_called_once()

    def test_plan_only_never_executes_and_move_checks_target_before_dispatch(self):
        for scenario in ('plan_only', 'target_lost', 'move_success'):
            task = self.bare_task()
            target = dict(xyz=[.6, 0., .125], surface_z=.15, height=.05)
            q = verify.verification_waypoints([.5, 0, .4], [0, 0, 0, 1], target, task.config)[-1]
            task.acquire_target = mock.Mock(return_value=target)
            task.plan = mock.Mock(return_value=(object(), trajectory(), q))
            task.recheck_target = mock.Mock(side_effect=RuntimeError('target lost')
                                           if scenario == 'target_lost' else None)
            task.execute, task.current_state = mock.Mock(), mock.Mock()
            task.fk_pose = mock.Mock(return_value=pose(q['xyz'], q['q']))
            modules = {'std_srvs': NS(), 'std_srvs.srv': NS(TriggerResponse=lambda **kw: NS(**kw))}
            with mock.patch.dict(sys.modules, modules):
                response = task.run(scenario != 'plan_only')
            if scenario == 'target_lost':
                self.assertFalse(response.success)
            else:
                self.assertTrue(response.success, response.message)
            if scenario == 'move_success':
                task.execute.assert_called_once()
            else:
                task.execute.assert_not_called()
            self.assertFalse(task.lock.locked())

    def samples(self):
        return [dict(xyz=[.6+.0001*i, .05, .125], height=.05,
                     depth_valid=True, stamp_ns=i+1) for i in range(5)]

    def test_target_height_and_stability_are_independent_of_yolo_yaw(self):
        target = verify.stable_target(self.samples(), .005, .1, .02, True)
        self.assertAlmostEqual(target['surface_z'], .15)
        self.assertAlmostEqual(target['xyz'][0], .6002)
        bad = self.samples()
        bad[-1]['xyz'][0] += .03
        with self.assertRaisesRegex(ValueError, 'unstable'):
            verify.stable_target(bad, .005, .1, .02, True)
        bad = self.samples()
        bad[-1]['depth_valid'] = False
        with self.assertRaisesRegex(ValueError, 'depth'):
            verify.stable_target(bad, .005, .1, .02, True)
        with self.assertRaises(ValueError):
            verify.stable_target([], .005, .1, .02, True)

    def test_waypoints_lift_align_transfer_then_hover_and_tcp_z_is_down(self):
        c = config()
        target = verify.stable_target(self.samples(), .005, .1, .02, True)
        waypoints = verify.verification_waypoints([.45, -.1, .2], [0, 0, 0, 1], target, c)
        self.assertAlmostEqual(waypoints[0]['xyz'][2], .35)
        self.assertEqual(waypoints[0]['q'], [0, 0, 0, 1])
        self.assertEqual(waypoints[1]['xyz'], waypoints[0]['xyz'])
        for p in waypoints[1:]:
            q = p['q']
            self.assertAlmostEqual(1-2*(q[0]**2+q[1]**2), -1)
            self.assertEqual(q, waypoints[-1]['q'])
        self.assertEqual(waypoints[2]['xyz'][:2], target['xyz'][:2])
        self.assertAlmostEqual(waypoints[-1]['xyz'][2], .25)
        target['xyz'][0] = 2
        with self.assertRaisesRegex(ValueError, 'workspace'):
            verify.verification_waypoints([.45, 0, .4], [0, 0, 0, 1], target, c)

    def test_no_fast_or_contact_configuration(self):
        for key, value in [('max_tcp_speed', .2), ('velocity_scaling', .8),
                           ('acceleration_scaling', .5), ('hover_clearance', .01),
                           ('eef_step', float('nan')), ('min_samples', 1)]:
            c = config()
            c[key] = value
            with self.assertRaises(ValueError, msg=key):
                verify.validate_config(c)

    def test_retiming_limits_between_point_quintic_peaks_and_preserves_geometry(self):
        c = config()
        positions, zero = [[0.], [.1]], [[0.], [0.]]
        factor = verify.slow_time_factor(positions, zero, zero, [0, .5],
                                         [[0, 0, 0], [.001, 0, 0]], c)
        self.assertGreater(factor, 1)
        u = np.linspace(0, 1, 1001)
        speed = .1*(30*u**2-60*u**3+30*u**4)/(.5*factor)
        acceleration = .1*(60*u-180*u**2+120*u**3)/(.5*factor)**2
        self.assertLessEqual(max(abs(speed)), c['max_joint_speed']+1e-8)
        self.assertLessEqual(max(abs(acceleration)), c['max_joint_acceleration']+1e-8)
        with self.assertRaisesRegex(ValueError, 'jump'):
            verify.slow_time_factor([[0.], [1.]], zero, zero, [0, 1], [[0, 0, 0]]*2, c)
        with self.assertRaises(ValueError):
            verify.slow_time_factor(positions, zero, zero, [0, 0], [[0, 0, 0]]*2, c)

    def test_old_and_new_cartesian_api_retimed_and_collision_checked(self):
        for old in (False, True):
            self.run_plan_case('valid', old)

    def test_partial_path_collision_wrong_orientation_and_missing_table_are_rejected(self):
        for scenario in ('partial', 'collision', 'tilted_transfer', 'missing_table'):
            self.run_plan_case(scenario)

    def run_plan_case(self, scenario, old=False):
        task = verify.HandeyeYoloVerify.__new__(verify.HandeyeYoloVerify)
        task.config, task.cancel = config(), threading.Event()
        task.active_joints, task.scene_ready = ['j1', 'j2'], scenario != 'missing_table'
        task.scene = mock.Mock()
        task.scene.get_known_object_names.return_value = ['hybrid_sorting_table']
        task.preview, task.goal_pub, task.arm = mock.Mock(), mock.Mock(), mock.Mock()
        state = NS(joint_state=NS(name=['j1', 'j2'], position=[0., 0.], header=NS()), is_diff=False)
        task.current_state = mock.Mock(return_value=state)
        target = dict(xyz=[.65, .05, .125], surface_z=.15, height=.05, stamp_ns=100)
        w = verify.verification_waypoints([.5, 0, .4], [0, 0, 0, 1], target, task.config)
        fk_poses = [pose(), pose(), pose(w[1]['xyz'], w[1]['q']),
                    pose(w[2]['xyz'], w[2]['q']), pose(w[3]['xyz'], w[3]['q'])]
        if scenario == 'tilted_transfer':
            fk_poses[3] = pose(w[2]['xyz'])
        task.fk_pose = mock.Mock(side_effect=fk_poses)
        task.validity = mock.Mock(return_value=NS(valid=scenario != 'collision'))
        path = trajectory()

        def cartesian(waypoints, step, avoid_collisions=True):
            self.assertTrue(avoid_collisions)
            self.assertEqual(len(waypoints), 4)
            return path, .9 if scenario == 'partial' else 1.

        def old_cartesian(waypoints, step, jump_threshold, avoid_collisions=True):
            self.assertEqual(jump_threshold, 0.)
            return cartesian(waypoints, step, avoid_collisions)

        task.arm.compute_cartesian_path = old_cartesian if old else cartesian
        task.arm.retime_trajectory.return_value = path
        fake_ros = NS(Time=lambda _: None, Duration=Duration, is_shutdown=lambda: False)
        fake_ros.Time = mock.Mock(side_effect=lambda _: None)
        fake_ros.Time.now.return_value = None
        modules = {'geometry_msgs': NS(), 'geometry_msgs.msg': NS(Pose=lambda: pose(), PoseStamped=pose_stamped),
                   'moveit_msgs': NS(), 'moveit_msgs.msg': NS(DisplayTrajectory=lambda **kw: NS(**kw))}
        with mock.patch.dict(sys.modules, modules), mock.patch.object(verify, 'rospy', fake_ros, create=True):
            if scenario == 'valid':
                _, result, goal = task.plan(target)
                task.arm.retime_trajectory.assert_called_once()
                self.assertEqual(task.validity.call_count, 4)
                self.assertGreater(result.joint_trajectory.points[-1].time_from_start.to_sec(), 3)
                self.assertEqual(goal, w[-1])
                task.preview.publish.assert_called_once()
            else:
                with self.assertRaises(RuntimeError, msg=scenario):
                    task.plan(target)
        task.arm.execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
