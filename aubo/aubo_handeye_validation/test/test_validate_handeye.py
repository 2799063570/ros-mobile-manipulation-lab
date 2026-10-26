#!/usr/bin/env python3
"""Synthetic ground truth checks for transform direction and error sensitivity."""
import importlib.util
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import subprocess
import shutil
import sys
import tempfile
import unittest
import uuid
from types import SimpleNamespace
from unittest import mock

import numpy as np
import yaml


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'validate_handeye.py'
SPEC = importlib.util.spec_from_file_location('validate_handeye', str(SCRIPT))
validation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validation)


@contextmanager
def temporary_directory():
    # Python 3.13's Windows mkdir(0700) can exclude a restricted sandbox token.
    root = Path(os.environ.get('HANDEYE_TEST_TMPDIR', tempfile.gettempdir())).resolve()
    directory = root / ('handeye-test-' + uuid.uuid4().hex)
    directory.mkdir(mode=0o755)
    try:
        yield str(directory)
    finally:
        if directory.resolve().parent != root:
            raise RuntimeError('unexpected test cleanup path')
        shutil.rmtree(directory)


def pose(t=(0, 0, 0), axis=(0, 0, 1), angle=0):
    axis = np.asarray(axis, dtype=float)
    q = np.r_[axis / np.linalg.norm(axis) * math.sin(angle/2), math.cos(angle/2)]
    return dict(t=list(t), q=q.tolist())


def compose(a, b):
    m = validation.matrix(a) @ validation.matrix(b)
    return dict(t=m[:3, 3].tolist(), q=validation.multiply_q(
        validation.quaternion(a['q']), validation.quaternion(b['q'])).tolist())


def inverse(a):
    q = validation.quaternion(a['q'])
    return dict(t=np.linalg.inv(validation.matrix(a))[:3, 3].tolist(),
                q=np.r_[-q[:3], q[3]].tolist())


def synthetic_session():
    calibration = pose((-.03, -.65, .91), (1, 1, .1), 3.12)
    marker = pose((.03, -.02, .15), (1, 2, 3), .4)
    groups = []
    for i in range(15):
        robot = pose((.25+.02*i, -.2+.013*i, .3+.01*i),
                     (1+i % 3, 1+(i+1) % 4, 1), .12*i)
        observed = compose(compose(inverse(calibration), robot), marker)
        observations = [dict(stamp_ns=i*1000000000+j+1, base_effector=robot,
                             camera_marker=observed) for j in range(10)]
        groups.append(dict(observations=observations))
    return dict(schema_version=1, calibration=dict(transform=calibration, frames=dict(
        robot_base_frame='base', robot_effector_frame='effector',
        tracking_base_frame='optical', tracking_marker_frame='marker')),
        reference=marker, poses=groups)


class ValidationTests(unittest.TestCase):
    def test_repository_calibration_is_a_valid_optical_frame_transform(self):
        path = SCRIPT.parents[2] / 'aubo_ros_control/config/aubo_realsense_handeye_eye_on_base.yaml'
        calibration = validation.load_calibration(str(path))
        self.assertEqual(calibration['frames']['robot_base_frame'], 'base_link')
        self.assertEqual(calibration['frames']['tracking_base_frame'], 'camera_color_optical_frame')
        self.assertTrue(np.isfinite(validation.matrix(calibration['transform'])).all())

    def run_collector(self, scenario):
        """Exercise the actual service callbacks with deterministic ROS/TF doubles."""
        session = synthetic_session()
        observed = session['poses'][0]['observations'][0]['camera_marker']
        robot = session['poses'][0]['observations'][0]['base_effector']
        clock, callbacks, services, responses = [100.0], {}, {}, []

        class Stamp:
            def __init__(self, seconds):
                self.seconds = seconds

            @staticmethod
            def now():
                return Stamp(clock[0])

            def to_nsec(self):
                return round(self.seconds*1e9)

            def to_sec(self):
                return self.seconds

            def __sub__(self, other):
                return Stamp(self.seconds-other.seconds)

            def __lt__(self, other):
                return self.seconds < other.seconds

        class TransformError(Exception):
            pass

        def message(pose, parent, child, stamp):
            return SimpleNamespace(header=SimpleNamespace(frame_id=parent, stamp=Stamp(stamp)),
                                   child_frame_id=child, transform=SimpleNamespace(
                                       translation=SimpleNamespace(**dict(zip('xyz', pose['t']))),
                                       rotation=SimpleNamespace(**dict(zip('xyzw', pose['q'])))))

        class Buffer:
            def __init__(self, **_kwargs):
                self.calls = 0

            def lookup_transform(self, target, source, stamp):
                self.calls += 1
                self_test.assertEqual((target, source), ('base', 'effector'))
                self_test.assertLessEqual(abs(clock[0]-stamp.seconds), .01)
                if scenario == 'missing_robot_tf':
                    raise TransformError('missing timestamp')
                current = copy_pose = dict(t=list(robot['t']), q=list(robot['q']))
                if scenario == 'moving':
                    copy_pose['t'][0] += .01*self.calls
                return message(current, target, source, stamp.seconds)

        def subscriber(topic, _kind, callback, **_kwargs):
            callbacks[topic] = callback
            return object()

        def service(name, _kind, callback):
            services[name] = callback
            return object()

        def sleep(seconds):
            clock[0] += seconds
            valid = message(observed, 'optical', 'marker', clock[0])
            messages = [valid, valid, message(observed, 'optical', 'marker', clock[0]-1),
                        message(observed, 'base', 'marker', clock[0])]
            if scenario == 'wrong_parent':
                messages = [message(observed, 'base', 'marker', clock[0])]
            callbacks['/tf'](SimpleNamespace(transforms=messages))

        def spin():
            responses.append(services['~capture'](None))
            responses.append(services['~report'](None))

        self_test = self
        rospy = SimpleNamespace(Time=Stamp, Duration=lambda v: v, init_node=lambda _: None,
                                Subscriber=subscriber, Service=service, spin=spin,
                                is_shutdown=lambda: False, loginfo=lambda *_: None,
                                logwarn=lambda *_: None, get_name=lambda: '/handeye_validation')
        modules = {'rospy': rospy,
                   'tf2_ros': SimpleNamespace(Buffer=Buffer, TransformListener=lambda _: object(),
                                              TransformException=TransformError),
                   'std_srvs': SimpleNamespace(),
                   'std_srvs.srv': SimpleNamespace(Trigger=object, TriggerResponse=SimpleNamespace),
                   'tf2_msgs': SimpleNamespace(),
                   'tf2_msgs.msg': SimpleNamespace(TFMessage=object)}
        with temporary_directory() as directory:
            yaml_path = os.path.join(directory, 'calibration.yaml')
            transform = session['calibration']['transform']
            values = dict(zip(('x', 'y', 'z'), transform['t']))
            values.update(zip(('qx', 'qy', 'qz', 'qw'), transform['q']))
            with open(yaml_path, 'w') as stream:
                yaml.safe_dump(dict(parameters=dict(eye_on_hand=False, **session['calibration']['frames']),
                                    transformation=values), stream)
            output = os.path.join(directory, 'capture')
            args = SimpleNamespace(calibration=yaml_path, reference=None, output=output,
                                   window=.08, min_samples=3, max_age=.5, stationary_mm=2,
                                   stationary_deg=1, tf_topic='/tf')
            with mock.patch.dict(sys.modules, modules), \
                    mock.patch.object(validation.time, 'monotonic', lambda: clock[0]), \
                    mock.patch.object(validation.time, 'sleep', sleep):
                validation.collect(args)
            with open(os.path.join(output, 'session.json')) as stream:
                saved = json.load(stream)
        return responses, saved

    def test_capture_uses_direct_fresh_observations_and_exact_robot_timestamps(self):
        responses, saved = self.run_collector('valid')
        self.assertTrue(responses[0].success, responses[0].message)
        self.assertTrue(responses[1].success)
        self.assertEqual(len(saved['poses']), 1)
        rejected = saved['poses'][0]['rejections']
        self.assertGreater(rejected['duplicate_timestamp'], 0)
        self.assertGreater(rejected['stale_or_invalid_timestamp'], 0)
        self.assertTrue(saved['attempts'][0]['accepted'])

    def test_capture_rejects_motion_missing_tf_and_indirect_marker_observations(self):
        for scenario in ('moving', 'missing_robot_tf', 'wrong_parent'):
            responses, saved = self.run_collector(scenario)
            self.assertFalse(responses[0].success, scenario)
            self.assertFalse(responses[1].success, scenario)
            self.assertEqual(saved['poses'], [])
            self.assertFalse(saved['attempts'][0]['accepted'])

    def test_ground_truth_closes_chain_and_sign_of_quaternion_is_irrelevant(self):
        session = synthetic_session()
        for group in session['poses'][::2]:
            group['observations'][0]['camera_marker']['q'] = [
                -v for v in group['observations'][0]['camera_marker']['q']]
        report = validation.analyze(session)
        self.assertLess(report['between_pose']['position_mm']['max'], 1e-9)
        self.assertLess(report['between_pose']['rotation_deg']['max'], 1e-5)
        self.assertLess(report['independent_reference_error']['position_mm']['max'], 1e-9)
        np.testing.assert_allclose(validation.matrix(report['mean_marker_in_effector']),
                                   validation.matrix(session['reference']), atol=1e-12)

    def test_wrong_extrinsic_translation_and_rotation_are_detected(self):
        for error_kind in ('translation', 'rotation'):
            session = synthetic_session()
            if error_kind == 'translation':
                session['calibration']['transform']['t'][0] += .02
            else:
                session['calibration']['transform'] = compose(
                    pose(axis=(1, 0, 0), angle=math.radians(3)),
                    session['calibration']['transform'])
            report = validation.analyze(session)
            self.assertGreater(report['between_pose']['position_mm']['rmse'], 1)
            self.assertGreater(report['independent_reference_error']['position_mm']['rmse'], 5)
            if error_kind == 'rotation':
                self.assertGreater(report['between_pose']['rotation_deg']['rmse'], .1)

    def test_single_pose_is_flagged_and_reference_can_expose_constant_bias(self):
        session = synthetic_session()
        session['poses'] = session['poses'][:1]
        session['calibration']['transform']['t'][0] += .02
        report = validation.analyze(session)
        self.assertFalse(report['absolute_accuracy_established'])
        self.assertTrue(any('Fewer' in warning for warning in report['warnings']))
        self.assertAlmostEqual(report['independent_reference_error']['position_mm']['rmse'], 20)

    def test_equal_pose_weighting_and_no_reference(self):
        session = synthetic_session()
        session.pop('reference')
        session['poses'][0]['observations'][0]['camera_marker']['t'][0] += .001
        first = validation.analyze(session)
        session['poses'][0]['observations'] *= 4
        second = validation.analyze(session)
        np.testing.assert_allclose(first['mean_marker_in_effector']['t'],
                                   second['mean_marker_in_effector']['t'])
        self.assertNotIn('independent_reference_error', second)
        self.assertTrue(any('absolute bias' in w for w in second['warnings']))

    def test_rotation_wrap_and_motion_span(self):
        a = pose(angle=math.radians(179))
        b = pose((.003, 0, 0), angle=math.radians(-179))
        mm, deg = validation.pose_span([a, b])
        self.assertAlmostEqual(mm, 3)
        self.assertAlmostEqual(deg, 2)
        center = validation.mean_pose([a, b])
        self.assertAlmostEqual(validation.distance(a, center)[1], 1)

    def test_invalid_data_and_empty_session_are_rejected(self):
        for invalid in (dict(t=[0, 0, float('nan')], q=[0, 0, 0, 1]),
                        dict(t=[0, 0, 0], q=[0, 0, 0, 0])):
            with self.assertRaises(ValueError):
                validation.matrix(invalid)
        session = synthetic_session()
        session['poses'] = []
        with self.assertRaises(ValueError):
            validation.analyze(session)

    def test_calibration_checks_and_offline_cli(self):
        session = synthetic_session()
        with temporary_directory() as directory:
            yaml_path = os.path.join(directory, 'calibration.yaml')
            values = dict(zip(('x', 'y', 'z'), session['calibration']['transform']['t']))
            values.update(zip(('qx', 'qy', 'qz', 'qw'), session['calibration']['transform']['q']))
            data = dict(parameters=dict(eye_on_hand=False, **session['calibration']['frames']),
                        transformation=values)
            with open(yaml_path, 'w') as stream:
                yaml.safe_dump(data, stream)
            self.assertEqual(validation.load_calibration(yaml_path)['frames']['tracking_base_frame'], 'optical')
            data['transformation']['z'] = '0.91neng'
            with open(yaml_path, 'w') as stream:
                yaml.safe_dump(data, stream)
            with self.assertRaisesRegex(ValueError, 'invalid transformation'):
                validation.load_calibration(yaml_path)
            source = os.path.join(directory, 'session.json')
            output = os.path.join(directory, 'report')
            with open(source, 'w') as stream:
                json.dump(session, stream)
            command = [sys.executable, str(SCRIPT), 'analyze', source, '--output', output]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('15 poses', result.stdout)
            with open(os.path.join(output, 'report.json')) as stream:
                self.assertEqual(json.load(stream)['observations'], 150)
            self.assertTrue(os.path.isfile(os.path.join(output, 'poses.csv')))
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)


if __name__ == '__main__':
    unittest.main()
