import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from aubo_vla.actions import SCHEMA, candidate, from_rotvec, multiply, pose_delta, validate_model_contract


class ActionTest(unittest.TestCase):
    def test_left_rotation_roundtrip_with_nonidentity_tcp(self):
        pose = dict(position=[0.5, 0., 0.3], orientation=from_rotvec([math.pi/2, 0., 0.]))
        action = [0.005, -0.003, 0.002, 0., 0., 0.03, 1.]
        target = candidate(pose, action, [0.3, -0.4, 0.12], [0.95, 0.4, 0.75], 0.01, 0.05)
        expected = multiply(from_rotvec(action[3:6]), pose['orientation'])
        for a, b in zip(expected, target['orientation']):
            self.assertAlmostEqual(a, b)
        for a, b in zip(action, pose_delta(pose, target, target['gripper'])):
            self.assertAlmostEqual(a, b)

    def test_quaternion_sign_does_not_create_motion(self):
        pose = dict(position=[0., 0., 0.], orientation=[0., 0., 0., 1.])
        after = dict(position=[0., 0., 0.], orientation=[0., 0., 0., -1.])
        self.assertEqual(pose_delta(pose, after, 0.), [0.]*7)

    def test_rejects_workspace_step_and_invalid_values(self):
        pose = dict(position=[0.5, 0., 0.3], orientation=[0., 0., 0., 1.])
        for action in ([0.02, 0., 0., 0., 0., 0., 0.],
                       [0., 0., 0., 0.06, 0., 0., 0.],
                       [0., 0., 0., 0., 0., 0., 1.1],
                       [float('nan')]+[0.]*6, [True]+[0.]*6, ['0']+[0.]*6):
            with self.assertRaises(ValueError):
                candidate(pose, action, [0.3, -0.4, 0.12], [0.95, 0.4, 0.75], 0.01, 0.05)
        with self.assertRaises(ValueError):
            candidate(pose, [0.]*7, [0.6, -0.4, 0.12], [0.95, 0.4, 0.75], 0.01, 0.05)

    def test_contract_requires_calibration_and_matching_horizon(self):
        result = dict(mock=False, observation_only=True, action_schema=SCHEMA,
                      revision='verified', unnorm_key='aubo_dataset', action_dt=0.5)
        validate_model_contract(result, 'verified', 'aubo_dataset', 0.5)
        for change in (dict(mock=True), dict(action_schema='unspecified'),
                       dict(revision='other'), dict(unnorm_key='bridge_orig'), dict(action_dt=1.)):
            with self.assertRaises(ValueError):
                validate_model_contract(dict(result, **change), 'verified', 'aubo_dataset', 0.5)
        with self.assertRaises(ValueError):
            validate_model_contract(result, '', '', 0.5)
