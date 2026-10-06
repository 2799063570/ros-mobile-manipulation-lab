import copy
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from aubo_vla.dataset import EpisodeWriter, inspect_episode


def fixture():
    buffer = io.BytesIO()
    Image.new('RGB', (8, 8), (255, 0, 0)).save(buffer, 'PNG')
    metadata = dict(instruction='pick red', scene_id='test', base_frame='base_link', tcp_frame='tcp_link',
                    sample_rate=2., sync_slop=0.05, joint_names=['arm'])
    pose = dict(position=[0.5, 0., 0.3], orientation=[0., 0., 0., 1.])
    sample = dict(stamp=10., joint_stamp=10.01, image_frame='camera',
                  tcp_pose=pose, camera_pose=copy.deepcopy(pose),
                  joints=dict(names=['arm'], positions=[0.]), gripper=0.,
                  camera_info=dict(frame_id='camera', width=8, height=8, distortion_model='plumb_bob',
                                   K=[10., 0., 4., 0., 10., 4., 0., 0., 1.], D=[],
                                   R=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                                   P=[10., 0., 4., 0., 0., 10., 4., 0., 0., 0., 1., 0.]))
    return metadata, sample, buffer.getvalue()


class DatasetTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)/'episode'
        self.metadata, self.sample, self.png = fixture()
        self.writer = EpisodeWriter(self.directory, self.metadata)

    def tearDown(self):
        if not self.writer.stream.closed:
            self.writer.finish('aborted')
        self.temp.cleanup()

    def append_pair(self, gap=0.5):
        self.writer.append(self.sample, self.png)
        after = copy.deepcopy(self.sample)
        after['stamp'] += gap
        after['joint_stamp'] += gap
        after['tcp_pose']['position'][0] += 0.005
        after['gripper'] = 1.
        self.writer.append(after, self.png)

    def test_completed_export_uses_next_feedback_and_image_at_t(self):
        self.append_pair()
        self.writer.finish('success')
        report, actions = inspect_episode(self.directory)
        self.assertTrue(report['passed'], report)
        self.assertEqual(actions[0]['image'], 'images/000000.png')
        self.assertAlmostEqual(actions[0]['action'][0], 0.005)
        self.assertEqual(actions[0]['action'][6], 1.)
        self.assertEqual(actions[0]['action_label'], 'measured_next_state_delta')

    def test_gaps_and_incomplete_episode_block_export(self):
        self.append_pair(gap=1.)
        report, actions = inspect_episode(self.directory)
        self.assertFalse(report['passed'])
        self.assertEqual(actions, [])
        self.writer.finish('success')
        self.assertFalse(inspect_episode(self.directory)[0]['passed'])

    def test_sync_calibration_and_reset_rejected(self):
        self.writer.append(self.sample, self.png)
        for change in ('sync', 'camera', 'reset'):
            sample = copy.deepcopy(self.sample)
            sample['stamp'] = 10.5
            sample['joint_stamp'] = 10.51
            if change == 'sync':
                sample['joint_stamp'] = 11.
            elif change == 'camera':
                sample['camera_info']['K'][0] += 1.
            else:
                sample['stamp'] = 9.
            with self.assertRaises(ValueError):
                self.writer.append(sample, self.png)

    def test_checksum_and_path_escape_detected(self):
        self.append_pair()
        self.writer.finish('success')
        (self.directory/'images/000000.png').write_bytes(b'corrupt')
        self.assertFalse(inspect_episode(self.directory)[0]['passed'])
        lines = (self.directory/'samples.jsonl').read_text(encoding='utf-8').splitlines()
        sample = json.loads(lines[0])
        sample['image'] = '../outside.png'
        lines[0] = json.dumps(sample)
        (self.directory/'samples.jsonl').write_text('\n'.join(lines)+'\n', encoding='utf-8')
        report, _ = inspect_episode(self.directory)
        self.assertTrue(any('escapes' in error for error in report['errors']))

    def test_cli_and_no_overwrite(self):
        self.append_pair()
        self.writer.finish('success')
        export = Path(self.temp.name)/'actions.jsonl'
        script = Path(__file__).resolve().parents[1]/'scripts/inspect_episode.py'
        args = [sys.executable, str(script), str(self.directory), '--export', str(export)]
        result = subprocess.run(args, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        original = export.read_bytes()
        self.assertNotEqual(subprocess.run(args, capture_output=True).returncode, 0)
        self.assertEqual(export.read_bytes(), original)

    def test_existing_episode_directory_is_never_replaced(self):
        with self.assertRaises(FileExistsError):
            EpisodeWriter(self.directory, self.metadata)

    def test_writer_rejects_corrupt_png_and_wrong_dimensions_or_mode(self):
        invalid = [b'\x89PNG\r\n\x1a\nBROKEN', self.png[:-12]]
        for mode, size in [('RGB', (9, 8)), ('L', (8, 8))]:
            buffer = io.BytesIO()
            Image.new(mode, size).save(buffer, 'PNG')
            invalid.append(buffer.getvalue())
        for png in invalid:
            with self.subTest(png_size=len(png)):
                with self.assertRaises(ValueError):
                    self.writer.append(self.sample, png)
                self.assertEqual(self.writer.metadata['samples'], 0)
                self.assertEqual(list((self.directory/'images').iterdir()), [])

    def test_inspector_rejects_invalid_images_with_matching_checksum(self):
        self.append_pair()
        self.writer.finish('success')
        stream = self.directory/'samples.jsonl'
        lines = stream.read_text(encoding='utf-8').splitlines()
        sample = json.loads(lines[0])
        invalid = [b'\x89PNG\r\n\x1a\nBROKEN', self.png[:-12]]
        for mode, size in [('RGB', (9, 8)), ('L', (8, 8))]:
            buffer = io.BytesIO()
            Image.new(mode, size).save(buffer, 'PNG')
            invalid.append(buffer.getvalue())
        for png in invalid:
            with self.subTest(png_size=len(png)):
                (self.directory/sample['image']).write_bytes(png)
                sample['image_sha256'] = hashlib.sha256(png).hexdigest()
                lines[0] = json.dumps(sample)
                stream.write_text('\n'.join(lines)+'\n', encoding='utf-8')
                report, actions = inspect_episode(self.directory)
                self.assertFalse(report['passed'], report)
                self.assertEqual(actions, [])
