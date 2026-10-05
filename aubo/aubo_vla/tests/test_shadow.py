import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_doubles import Marker, Stamp, load_node
from aubo_vla.actions import SCHEMA


class ShadowTest(unittest.TestCase):
    def setUp(self):
        params = {'~expected_revision': 'verified', '~expected_unnorm_key': 'aubo',
                  '~workspace_min': [0.3, -0.4, 0.12], '~workspace_max': [0.95, 0.4, 0.75]}
        self.module, self.rospy, self.tf = load_node('shadow_preview', params)
        self.node = self.module.Shadow()
        self.result = dict(request_id='id1', stamp=10., frame_id='camera', mock=False,
                           observation_only=True, revision='verified', unnorm_key='aubo',
                           action_schema=SCHEMA, action_dt=0.5, action=[0.005, 0., 0., 0., 0., 0., 1.])
        Stamp.now_value = 10.2

    def send(self, **changes):
        result = dict(self.result, **changes)
        self.node.on_result(NS(data=json.dumps(dict(status='observed', result=result))))
        return json.loads(self.node.pub.publish.call_args.args[0].data)

    def test_valid_candidate_is_marked_nonexecutable(self):
        result = self.send()
        self.assertEqual(result['status'], 'preview')
        self.assertFalse(result['executable'])
        self.assertFalse(result['collision_checked'])
        self.assertAlmostEqual(result['candidate']['position'][0], 0.505)
        self.assertEqual(len(self.node.markers.publish.call_args.args[0].markers), 2)

    def test_mock_bridge_statistics_old_frame_and_steps_rejected(self):
        for change in (dict(mock=True), dict(unnorm_key='bridge_orig'), dict(stamp=1.),
                       dict(action=[0.02, 0., 0., 0., 0., 0., 0.])):
            self.assertEqual(self.send(**change)['status'], 'rejected')
            self.assertTrue(all(marker.action == Marker.DELETE
                                for marker in self.node.markers.publish.call_args.args[0].markers))

    def test_pause_and_wall_expiry_delete_markers(self):
        self.send()
        self.node.visible = (10., time.monotonic()-1.)
        self.rospy.is_shutdown.side_effect = [False, True]
        with patch.object(self.module.time, 'sleep', return_value=None):
            self.node.run()
        self.assertIsNone(self.node.visible)
        self.assertEqual(json.loads(self.node.pub.publish.call_args.args[0].data)['status'], 'expired')
        self.node.guard.simulated = True
        self.assertEqual(self.send(request_id='id2')['status'], 'rejected')

    def test_default_contract_rejects_before_tf_lookup(self):
        self.node.revision, self.node.key = '', ''
        self.assertEqual(self.send()['status'], 'rejected')
        self.tf.lookup_transform.assert_not_called()
