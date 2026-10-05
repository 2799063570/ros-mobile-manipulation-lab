import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_doubles import Stamp, load_node, transform
from test_dataset import fixture
from aubo_vla.dataset import inspect_episode


class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        _, sample, png = fixture()
        params = {'~output_root': self.temp.name, '~joint_names': ['arm'], '~gripper_joint': 'finger'}
        self.module, self.rospy, self.tf = load_node('record_episode', params, png)
        self.node = self.module.Recorder()
        self.image = NS(header=NS(stamp=Stamp(10.), frame_id='camera'), width=8, height=8)
        self.joints = NS(header=NS(stamp=Stamp(10.01)), name=['arm', 'finger'], position=[0., 0.])
        c = sample['camera_info']
        self.camera = NS(header=NS(stamp=Stamp(10.), frame_id='camera'),
                         **{k: c[k] for k in ('width', 'height', 'distortion_model', 'K', 'D', 'R', 'P')})
        self.node.on_camera(self.camera)
        Stamp.now_value = 10.

    def tearDown(self):
        self.node.finish('aborted')
        self.temp.cleanup()

    def item(self):
        return (self.image, self.joints, self.camera, time.monotonic(), self.node.writer)

    def test_two_frames_record_and_inspect_with_exact_tf(self):
        response = self.node.start(None)
        self.assertTrue(response.success)
        self.node.record(self.item())
        self.image.header.stamp = Stamp(10.5)
        self.joints.header.stamp = Stamp(10.51)
        self.joints.position[1] = 0.28
        self.camera.header.stamp = Stamp(10.5)
        Stamp.now_value = 10.7
        self.node.record(self.item())
        self.node.finish('success')
        report, actions = inspect_episode(response.message)
        self.assertTrue(report['passed'], report)
        self.assertEqual(actions[0]['action'][6], 1.)
        self.assertTrue(all(call.args[3] == 0 for call in self.tf.lookup_transform.call_args_list))
        self.assertEqual(self.tf.lookup_transform.call_args_list[0].args[2].to_sec(), 10.)

    def test_missing_feedback_and_unsynced_frames_not_written(self):
        self.node.start(None)
        self.joints.name = ['arm']
        self.joints.position = [0.]
        with self.assertRaises(ValueError):
            self.node.record(self.item())
        self.assertEqual(self.node.writer.metadata['samples'], 0)
        self.joints.name, self.joints.position = ['arm', 'finger'], [0., 0.]
        self.joints.header.stamp = Stamp(10.1)
        with self.assertRaises(ValueError):
            self.node.record(self.item())
        self.assertEqual(self.node.writer.metadata['samples'], 0)

    def test_instruction_is_frozen_until_next_episode(self):
        self.node.start(None)
        instruction = self.node.writer.metadata['instruction']
        self.node.on_instruction(NS(data='pick blue'))
        self.assertEqual(self.node.writer.metadata['instruction'], instruction)
        self.node.finish('success')
        self.node.start(None)
        self.assertEqual(self.node.writer.metadata['instruction'], 'pick blue')

    def test_buffered_sample_before_episode_start_is_rejected(self):
        Stamp.now_value = 10.1
        self.node.start(None)
        with self.assertRaises(ValueError):
            self.node.record(self.item())
        self.assertEqual(self.node.writer.metadata['samples'], 0)

    def test_clock_reset_aborts_episode_and_start_while_paused_refused(self):
        self.node.guard.simulated = True
        self.assertFalse(self.node.start(None).success)
        self.node.on_clock(NS(clock=Stamp(10.)))
        response = self.node.start(None)
        self.assertTrue(response.success)
        self.node.on_clock(NS(clock=Stamp(5.)))
        self.assertIsNone(self.node.writer)
        metadata = json.loads((Path(response.message)/'episode.json').read_text(encoding='utf-8'))
        self.assertEqual(metadata['outcome'], 'aborted')

    def test_delayed_tf_is_retried_without_latest_transform_fallback(self):
        self.node.start(None)
        self.tf.lookup_transform.side_effect = [TimeoutError('not arrived'), transform(), transform()]
        with patch.object(self.module.time, 'sleep', return_value=None):
            self.node.record(self.item())
        self.assertEqual(self.node.writer.metadata['samples'], 1)
        self.assertTrue(all(call.args[2].to_sec() == 10. for call in self.tf.lookup_transform.call_args_list))
