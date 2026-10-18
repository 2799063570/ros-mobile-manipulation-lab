#!/usr/bin/env python3
"""Read-only eye-on-base validation. ROS is only required by `collect`."""
import argparse
import copy
import csv
import json
import math
import os
import tempfile
import threading
import time
from collections import Counter, deque

import numpy as np
import yaml


FIELDS = ('robot_base_frame', 'robot_effector_frame',
          'tracking_base_frame', 'tracking_marker_frame')


def frame(value):
    if not isinstance(value, str) or not value.strip('/'):
        raise ValueError('frame names must be nonempty strings')
    return value.strip('/')


def vector(values, length, label):
    result = np.asarray(values, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError('{} must contain {} finite numbers'.format(label, length))
    return result


def quaternion(values):
    q = vector(values, 4, 'quaternion xyzw')
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError('zero quaternion')
    return q / norm


def matrix(pose):
    x, y, z, w = quaternion(pose['q'])
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = vector(pose['t'], 3, 'translation m')
    return result


def multiply_q(a, b):
    av, bv = a[:3], b[:3]
    return quaternion(np.r_[a[3]*bv + b[3]*av + np.cross(av, bv),
                            a[3]*b[3] - np.dot(av, bv)])


def marker_in_effector(robot, camera, calibration):
    # ^E T_M = inv(^B T_E) ^B T_C ^C T_M; never use a TF-derived C->M.
    transform = np.linalg.inv(matrix(robot)) @ matrix(calibration) @ matrix(camera)
    qr = quaternion(robot['q'])
    inverse_qr = np.r_[-qr[:3], qr[3]]
    q = multiply_q(multiply_q(inverse_qr, quaternion(calibration['q'])),
                   quaternion(camera['q']))
    return {'t': transform[:3, 3].tolist(), 'q': q.tolist()}


def mean_pose(poses):
    if not poses:
        raise ValueError('cannot average empty poses')
    qs = np.asarray([quaternion(p['q']) for p in poses])
    _, eigvec = np.linalg.eigh(qs.T @ qs)
    q = eigvec[:, -1]
    if q[3] < 0:
        q = -q
    return {'t': np.mean([vector(p['t'], 3, 'translation') for p in poses],
                         axis=0).tolist(), 'q': q.tolist()}


def distance(a, b):
    translation = float(np.linalg.norm(vector(a['t'], 3, 'translation') -
                                       vector(b['t'], 3, 'translation'))) * 1000
    cosine = float(abs(np.dot(quaternion(a['q']), quaternion(b['q']))))
    angle = math.degrees(2 * math.acos(min(1.0, cosine)))
    return translation, angle


def statistics(values):
    values = np.asarray(values, dtype=float)
    return dict(n=len(values), mean=float(np.mean(values)),
                rmse=float(np.sqrt(np.mean(values**2))),
                p95=float(np.percentile(values, 95)), max=float(np.max(values)))


def residuals(poses, reference):
    errors = [distance(p, reference) for p in poses]
    return {'position_mm': statistics([p for p, _ in errors]),
            'rotation_deg': statistics([r for _, r in errors])}


def pose_span(poses):
    translations = np.asarray([vector(p['t'], 3, 'translation') for p in poses])
    qs = np.asarray([quaternion(p['q']) for p in poses])
    # Vectorized per row, keeping memory linear in observation count.
    max_translation, min_cosine = 0.0, 1.0
    for t, q in zip(translations, qs):
        max_translation = max(max_translation, float(np.max(np.linalg.norm(translations-t, axis=1))))
        min_cosine = min(min_cosine, float(np.min(np.abs(qs @ q))))
    return max_translation*1000, math.degrees(2*math.acos(min(1.0, min_cosine)))


def rotation_excitation(poses):
    first = quaternion(poses[0]['q'])
    inverse_first = np.r_[-first[:3], first[3]]
    vectors = []
    for pose in poses:
        relative = multiply_q(inverse_first, quaternion(pose['q']))
        if relative[3] < 0:
            relative = -relative
        sine = np.linalg.norm(relative[:3])
        angle = 2*math.atan2(sine, relative[3])
        vectors.append(relative[:3]*angle/max(sine, 1e-12))
    singular = np.linalg.svd(np.asarray(vectors), compute_uv=False)
    return np.pad(np.degrees(singular), (0, max(0, 3-len(singular)))).tolist()


def yaml_pose(mapping):
    try:
        result = {'t': [float(mapping[k]) for k in ('x', 'y', 'z')],
                  'q': [float(mapping[k]) for k in ('qx', 'qy', 'qz', 'qw')]}
        matrix(result)
        result['q'] = quaternion(result['q']).tolist()
        return result
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('invalid transformation (x/y/z in m, qx/qy/qz/qw): {}'.format(error))


def load_calibration(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('parameters'), dict):
        raise ValueError('expected easy_handeye parameters and transformation')
    params = data['parameters']
    if params.get('eye_on_hand') is not False:
        raise ValueError('only eye_on_hand: false (eye-on-base) is supported')
    names = {k: frame(params.get(k)) for k in FIELDS}
    if len(set(names.values())) != len(FIELDS):
        raise ValueError('base, effector, camera and marker frames must be distinct')
    return dict(frames=names, transform=yaml_pose(data.get('transformation', {})),
                source=os.path.abspath(path))


def load_reference(path, frames):
    if not path:
        return None
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if (frame(data['parent_frame']) != frames['robot_effector_frame'] or
            frame(data['child_frame']) != frames['tracking_marker_frame']):
        raise ValueError('reference must be effector -> marker, with the exact saved frames')
    return yaml_pose(data['transformation'])


def analyze(session):
    if session.get('schema_version') != 1:
        raise ValueError('unsupported session schema')
    calibration = session['calibration']['transform']
    matrix(calibration)
    groups = session['poses']
    if not groups:
        raise ValueError('no captured poses')
    estimates, robot_means, rows = [], [], []
    for index, group in enumerate(groups):
        observations = group['observations']
        transformed = [marker_in_effector(o['base_effector'], o['camera_marker'],
                                          calibration) for o in observations]
        estimate = mean_pose(transformed)
        estimates.append(estimate)
        robot_means.append(mean_pose([o['base_effector'] for o in observations]))
        rows.append(dict(pose_id=index+1, observations=len(observations),
                         marker_in_effector=estimate,
                         within_pose=residuals(transformed, estimate)))
    center = mean_pose(estimates)  # Equal weight per pose, independent of detector rate.
    span_mm, span_deg = pose_span(robot_means)
    excitation = rotation_excitation(robot_means)
    warnings = ['Consistency residuals include vision, robot kinematics and synchronization; '
                'they do not establish absolute hand-eye accuracy.']
    if len(groups) < 10:
        warnings.append('Fewer than 10 independent poses; collect 15-20 held-out poses.')
    if span_mm < 100 or span_deg < 20:
        warnings.append('Limited robot pose coverage: aim for workspace coverage and varied rotations.')
    if excitation[1] < 5:
        warnings.append('Insufficient rotation around multiple axes; a small residual can be misleading.')
    if not session.get('reference'):
        warnings.append('No independently measured effector->marker reference; absolute bias is unknown.')
    for row, estimate in zip(rows, estimates):
        row['position_residual_mm'], row['rotation_residual_deg'] = distance(estimate, center)
    report = dict(schema_version=1, metric='held_out_eye_on_base_consistency',
                  absolute_accuracy_established=False,
                  poses=len(groups), observations=sum(r['observations'] for r in rows),
                  frames=session['calibration']['frames'],
                  mean_marker_in_effector=center,
                  between_pose=residuals(estimates, center),
                  robot_pose_span=dict(translation_mm=span_mm, rotation_deg=span_deg),
                  rotation_excitation_singular_values_deg=excitation,
                  warnings=warnings, per_pose=rows)
    if session.get('reference'):
        report['independent_reference_error'] = residuals(estimates, session['reference'])
        report['independent_reference'] = session['reference']
    return report


def atomic_json(path, value):
    directory = os.path.dirname(os.path.abspath(path))
    fd, temporary = tempfile.mkstemp(dir=directory, suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_report(directory, session):
    report = analyze(session)
    atomic_json(os.path.join(directory, 'report.json'), report)
    with open(os.path.join(directory, 'poses.csv'), 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['pose_id', 'observations', 'marker_x_m', 'marker_y_m', 'marker_z_m',
                         'position_residual_mm', 'rotation_residual_deg',
                         'within_pose_position_rmse_mm', 'within_pose_rotation_rmse_deg'])
        for row in report['per_pose']:
            writer.writerow([row['pose_id'], row['observations']] + row['marker_in_effector']['t'] +
                            [row['position_residual_mm'], row['rotation_residual_deg'],
                             row['within_pose']['position_mm']['rmse'],
                             row['within_pose']['rotation_deg']['rmse']])
    return report


def summary(report):
    pos = report['between_pose']['position_mm']
    rot = report['between_pose']['rotation_deg']
    return ('{} poses; consistency position RMSE/P95/max {:.3f}/{:.3f}/{:.3f} mm; '
            'rotation RMSE/P95/max {:.3f}/{:.3f}/{:.3f} deg').format(
                report['poses'], pos['rmse'], pos['p95'], pos['max'],
                rot['rmse'], rot['p95'], rot['max'])


def ros_pose(message):
    t, q = message.transform.translation, message.transform.rotation
    pose = dict(t=[t.x, t.y, t.z], q=[q.x, q.y, q.z, q.w])
    matrix(pose)
    return pose


def collect(args):
    import rospy
    import tf2_ros
    from std_srvs.srv import Trigger, TriggerResponse
    from tf2_msgs.msg import TFMessage

    calibration = load_calibration(args.calibration)
    reference = load_reference(args.reference, calibration['frames'])
    session = dict(schema_version=1, calibration=calibration, reference=reference,
                   acquisition=dict(window_s=args.window, min_samples=args.min_samples,
                                    max_age_s=args.max_age, stationary_mm=args.stationary_mm,
                                    stationary_deg=args.stationary_deg), poses=[], attempts=[])
    # A new directory prevents overwriting a previous experiment.
    os.makedirs(args.output, exist_ok=False)
    atomic_json(os.path.join(args.output, 'session.json'), session)
    rospy.init_node('handeye_validation')
    names = calibration['frames']
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
    listener = tf2_ros.TransformListener(buffer)
    queue, queue_lock, service_lock = deque(maxlen=1000), threading.Lock(), threading.Lock()

    def incoming(message):
        # Accept a direct measured optical->marker edge only, never an arbitrary TF path.
        for transform in message.transforms:
            if (transform.header.frame_id.lstrip('/') == names['tracking_base_frame'] and
                    transform.child_frame_id.lstrip('/') == names['tracking_marker_frame']):
                with queue_lock:
                    queue.append(transform)

    subscriber = rospy.Subscriber(args.tf_topic, TFMessage, incoming, queue_size=100)

    def reject_capture(reason, rejections, observations):
        session['attempts'].append(dict(accepted=False, reason=reason,
                                        rejections=dict(rejections), observations=observations))
        atomic_json(os.path.join(args.output, 'session.json'), session)
        return TriggerResponse(success=False, message=reason)

    def capture(_request):
        if not service_lock.acquire(blocking=False):
            return TriggerResponse(success=False, message='another capture/report is running')
        try:
            start_stamp = rospy.Time.now()
            with queue_lock:
                queue.clear()
            deadline = time.monotonic() + args.window
            samples, seen, rejected = [], set(), Counter()
            while time.monotonic() < deadline and not rospy.is_shutdown():
                with queue_lock:
                    messages = list(queue)
                    queue.clear()
                for message in messages:
                    stamp = message.header.stamp
                    key = stamp.to_nsec()
                    if key in seen:
                        rejected['duplicate_timestamp'] += 1
                        continue
                    seen.add(key)
                    age = (rospy.Time.now() - stamp).to_sec()
                    if key == 0 or stamp < start_stamp or age < 0 or age > args.max_age:
                        rejected['stale_or_invalid_timestamp'] += 1
                        continue
                    try:
                        robot = buffer.lookup_transform(names['robot_base_frame'],
                                                        names['robot_effector_frame'], stamp)
                        samples.append(dict(stamp_ns=key, base_effector=ros_pose(robot),
                                            camera_marker=ros_pose(message)))
                    except (tf2_ros.TransformException, ValueError) as error:
                        rejected[type(error).__name__] += 1
                time.sleep(0.01)  # Bounded by wall time even if /clock stops.
            if len(samples) < args.min_samples:
                reason = (
                    'only {} fresh synchronized observations (need {}); rejections={}; '
                    'require direct dynamic TF {} -> {} on {} and robot TF at image time').format(
                        len(samples), args.min_samples, dict(rejected),
                        names['tracking_base_frame'], names['tracking_marker_frame'], args.tf_topic)
                return reject_capture(reason, rejected, samples)
            robot_poses = [s['base_effector'] for s in samples]
            # Pairwise span catches motion even when the mean is near the first pose.
            mm, deg = pose_span(robot_poses)
            if mm > args.stationary_mm or deg > args.stationary_deg:
                reason = ('robot moved during capture: {:.3f} mm / {:.3f} deg; '
                          'stop and retry').format(mm, deg)
                return reject_capture(reason, rejected, samples)
            candidate = copy.deepcopy(session)
            candidate['poses'].append(dict(observations=samples, rejections=dict(rejected)))
            candidate['attempts'].append(dict(accepted=True, pose_id=len(candidate['poses'])))
            # Commit raw observations first. Reports can always be recomputed from them.
            atomic_json(os.path.join(args.output, 'session.json'), candidate)
            session.update(candidate)
            report = write_report(args.output, session)
            message = summary(report)
            rospy.loginfo(message)
            if len(session['poses']) < 10:
                rospy.logwarn('Collect 15-20 distinct held-out poses; early residuals are not an accuracy result.')
            return TriggerResponse(success=True, message=message)
        except (OSError, ValueError) as error:
            return TriggerResponse(success=False, message=str(error))
        finally:
            service_lock.release()

    def report_service(_request):
        with service_lock:
            try:
                return TriggerResponse(success=True, message=summary(write_report(args.output, session)))
            except (ValueError, OSError) as error:
                return TriggerResponse(success=False, message=str(error))

    capture_server = rospy.Service('~capture', Trigger, capture)
    report_server = rospy.Service('~report', Trigger, report_service)
    rospy.loginfo('Read-only validation of %s. Hold marker rigidly on effector. '
                  'Stop robot, then call %s/capture. Output: %s', calibration['source'],
                  rospy.get_name(), os.path.abspath(args.output))
    rospy.logwarn('Reports are consistency residuals, not absolute calibration accuracy. '
                  'Use original image timestamps and independent held-out robot poses.')
    rospy.spin()


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='mode', required=True)
    live = subparsers.add_parser('collect', help='ROS1 TF sampling, no motion commands')
    live.add_argument('--calibration', required=True, help='easy_handeye eye-on-base YAML')
    live.add_argument('--output', required=True, help='new output directory')
    live.add_argument('--reference', help='independently measured effector->marker YAML')
    live.add_argument('--tf-topic', default='/tf')
    live.add_argument('--window', type=positive, default=2.0)
    live.add_argument('--min-samples', type=int, default=10)
    live.add_argument('--max-age', type=positive, default=0.5)
    live.add_argument('--stationary-mm', type=positive, default=2.0)
    live.add_argument('--stationary-deg', type=positive, default=1.0)
    offline = subparsers.add_parser('analyze', help='recompute a captured session without ROS')
    offline.add_argument('session')
    offline.add_argument('--output', required=True, help='new output directory')
    args, remaps = parser.parse_known_args()
    if any(':=' not in value for value in remaps):
        parser.error('unrecognized arguments: ' + ' '.join(remaps))
    try:
        if args.mode == 'collect':
            if args.min_samples < 2:
                parser.error('--min-samples must be >= 2')
            collect(args)
        else:
            with open(args.session, encoding='utf-8') as stream:
                session = json.load(stream)
            analyze(session)
            os.makedirs(args.output, exist_ok=False)
            report = write_report(args.output, session)
            print(summary(report))
            for warning in report['warnings']:
                print('NOTE: ' + warning)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.exit(2, 'handeye validation: {}\n'.format(error))


if __name__ == '__main__':
    main()
