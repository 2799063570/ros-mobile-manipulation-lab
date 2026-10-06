"""Versioned RGB/TCP demonstrations and offline measured-motion export."""
import hashlib
import io
import json
import math
from pathlib import Path

from PIL import Image

from .actions import SCHEMA, pose_delta, quaternion, vector
from .timing import positive


def save_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n',
                         encoding='utf-8')
    temporary.replace(path)


def validate_sample(sample):
    positive(sample['stamp'], 'stamp')
    positive(sample['joint_stamp'], 'joint_stamp')
    if not sample['image_frame']:
        raise ValueError('Missing image frame')
    vector(sample['tcp_pose']['position'], 3, 'TCP position')
    quaternion(sample['tcp_pose']['orientation'])
    joints = sample['joints']
    if not joints['names'] or len(set(joints['names'])) != len(joints['names']):
        raise ValueError('Joint names must be unique and nonempty')
    vector(joints['positions'], len(joints['names']), 'joint positions')
    gripper = vector([sample['gripper']], 1, 'gripper')[0]
    if not 0 <= gripper <= 1:
        raise ValueError('Gripper feedback outside calibrated range')
    camera = sample['camera_info']
    if camera['frame_id'] != sample['image_frame']:
        raise ValueError('CameraInfo/image frame mismatch')
    if camera['width'] <= 0 or camera['height'] <= 0:
        raise ValueError('Invalid camera dimensions')
    k = vector(camera['K'], 9, 'camera K')
    if k[0] <= 0 or k[4] <= 0:
        raise ValueError('Camera intrinsics are not calibrated')
    vector(camera['D'], len(camera['D']), 'camera D')
    vector(camera['R'], 9, 'camera R')
    vector(camera['P'], 12, 'camera P')
    vector(sample['camera_pose']['position'], 3, 'camera position')
    quaternion(sample['camera_pose']['orientation'])


def validate_png(png, camera):
    """Verify encoded data and decode pixels before trusting an observation."""
    if not png.startswith(b'\x89PNG\r\n\x1a\n'):
        raise ValueError('Expected PNG bytes')
    try:
        with Image.open(io.BytesIO(png)) as image:
            if image.format != 'PNG' or image.mode != 'RGB':
                raise ValueError('Expected RGB PNG image')
            if image.size != (camera['width'], camera['height']):
                raise ValueError('PNG/CameraInfo dimensions mismatch')
            image.verify()
        # verify checks PNG structure/CRC; load also checks compressed pixels.
        with Image.open(io.BytesIO(png)) as image:
            image.load()
    except (OSError, SyntaxError, ValueError, Image.DecompressionBombError) as exc:
        raise ValueError('Invalid observation PNG: '+str(exc)) from exc


class EpisodeWriter:
    """Caller serializes access. A crash leaves state=recording, never complete."""
    def __init__(self, directory, metadata):
        self.directory = Path(directory)
        rate = positive(metadata['sample_rate'], 'sample_rate')
        positive(metadata['sync_slop'], 'sync_slop')
        if not metadata.get('instruction', '').strip():
            raise ValueError('Missing episode instruction')
        if not metadata.get('base_frame') or not metadata.get('tcp_frame'):
            raise ValueError('Missing frame contract')
        self.metadata = dict(metadata, dataset_version=1, action_schema=SCHEMA,
                             action_label='measured_next_state_delta', nominal_dt=1/rate,
                             state='recording', outcome=None, samples=0, rejected=0)
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory/'images').mkdir()
        self.stream = (self.directory/'samples.jsonl').open('x', encoding='utf-8')
        self.last_stamp = None
        self.camera = None
        save_json(self.directory/'episode.json', self.metadata)

    def append(self, sample, png):
        validate_sample(sample)
        validate_png(png, sample['camera_info'])
        if self.last_stamp is not None and sample['stamp'] <= self.last_stamp:
            raise ValueError('Non-increasing image timestamp')
        if abs(sample['joint_stamp']-sample['stamp']) > self.metadata['sync_slop']:
            raise ValueError('Image/joint synchronization exceeds tolerance')
        if self.camera is not None and self.camera != sample['camera_info']:
            raise ValueError('Camera calibration changed during episode')
        if sample['joints']['names'] != self.metadata['joint_names']:
            raise ValueError('Joint order changed during episode')
        index = self.metadata['samples']
        relative = 'images/%06d.png' % index
        (self.directory/relative).write_bytes(png)
        record = dict(sample, index=index, image=relative, image_sha256=hashlib.sha256(png).hexdigest())
        self.stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
        self.stream.flush()
        self.last_stamp = sample['stamp']
        self.camera = sample['camera_info']
        self.metadata['samples'] += 1

    def reject(self, reason):
        self.metadata['rejected'] += 1
        self.metadata['last_rejection'] = str(reason)

    def finish(self, outcome, reason=''):
        if outcome not in ('success', 'failure', 'aborted'):
            raise ValueError('Unknown outcome')
        self.stream.close()
        self.metadata.update(state='complete' if outcome != 'aborted' else 'aborted',
                             outcome=outcome, finish_reason=reason)
        save_json(self.directory/'episode.json', self.metadata)


def inspect_episode(directory, period_tolerance=0.20):
    directory = Path(directory).resolve()
    period_tolerance = float(period_tolerance)
    if not math.isfinite(period_tolerance) or not 0 <= period_tolerance < 1:
        raise ValueError('period_tolerance must be in [0, 1)')
    metadata = json.loads((directory/'episode.json').read_text(encoding='utf-8'))
    if metadata.get('dataset_version') != 1 or metadata.get('action_schema') != SCHEMA:
        raise ValueError('Unsupported dataset/action schema')
    nominal = positive(metadata['nominal_dt'], 'nominal_dt')
    slop = positive(metadata['sync_slop'], 'sync_slop')
    samples = [json.loads(line) for line in (directory/'samples.jsonl').read_text(
        encoding='utf-8').splitlines() if line.strip()]
    errors, actions = [], []
    valid = []
    camera = None
    for i, sample in enumerate(samples):
        try:
            validate_sample(sample)
            if sample['index'] != i or sample['joints']['names'] != metadata['joint_names']:
                raise ValueError('Index or joint order mismatch')
            if abs(sample['joint_stamp']-sample['stamp']) > slop:
                raise ValueError('Unsynchronized sample')
            if camera is not None and camera != sample['camera_info']:
                raise ValueError('Camera calibration changed')
            camera = sample['camera_info']
            path = (directory/sample['image']).resolve()
            if path.parent != directory/'images':
                raise ValueError('Image path escapes episode/images')
            png = path.read_bytes()
            if not png.startswith(b'\x89PNG\r\n\x1a\n') or hashlib.sha256(png).hexdigest() != sample['image_sha256']:
                raise ValueError('Image missing, invalid or checksum mismatch')
            validate_png(png, sample['camera_info'])
            valid.append(True)
        except (ValueError, KeyError, TypeError, OSError) as exc:
            errors.append('sample %d: %s' % (i, exc))
            valid.append(False)
    for i in range(len(samples)-1):
        if not valid[i] or not valid[i+1]:
            continue
        before, after = samples[i:i+2]
        dt = after['stamp']-before['stamp']
        if dt <= 0 or abs(dt-nominal) > nominal*period_tolerance + 1e-9:
            errors.append('pair %d: dt %.6f does not match nominal %.6f' % (i, dt, nominal))
            continue
        actions.append(dict(index=i, stamp=before['stamp'], next_stamp=after['stamp'], dt=dt,
                            image=before['image'], instruction=metadata['instruction'],
                            action_schema=SCHEMA, action_label=metadata['action_label'],
                            action=pose_delta(before['tcp_pose'], after['tcp_pose'], after['gripper'])))
    if metadata.get('state') != 'complete':
        errors.append('Episode is incomplete or aborted')
    if metadata.get('outcome') not in ('success', 'failure'):
        errors.append('Episode has no operator outcome label')
    if metadata['samples'] != len(samples):
        errors.append('Manifest sample count mismatch')
    if len(samples) < 2:
        errors.append('At least two samples are required')
    report = dict(passed=not errors, samples=len(samples), valid_pairs=len(actions),
                  errors=errors, outcome=metadata.get('outcome'),
                  action_schema=SCHEMA, action_label=metadata['action_label'],
                  nominal_dt=nominal, rejected=metadata.get('rejected', 0))
    return report, actions
