#!/usr/bin/python3
"""Bounded, read-only RGB-D stream check; exits nonzero on missing/stale data."""
import argparse
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', default='/camera')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--pointcloud', action='store_true')
    parser.add_argument('--imu', action='store_true')
    parser.add_argument('--stationary', action='store_true',
                        help='With --imu, check stationary gravity and gyro magnitude')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(rospy.myargv()[1:])
    if not 2 <= args.duration <= 300:
        parser.error('--duration must be between 2 and 300 seconds')
    if args.stationary and not args.imu:
        parser.error('--stationary requires --imu and a stationary camera')
    rospy.init_node('check_rgbd_camera', anonymous=True)
    specs = {'color': ('color/image_raw', Image),
             'depth': ('aligned_depth_to_color/image_raw', Image),
             'info': ('color/camera_info', CameraInfo)}
    if args.pointcloud:
        specs['points'] = ('depth/color/points', PointCloud2)
    if args.imu:
        specs['imu'] = ('imu', Imu)
    samples = {key: [] for key in specs}
    latest = {}
    imu_norms = []
    lock = threading.Lock()

    def receive(message, key):
        with lock:
            samples[key].append((time.monotonic(), message.header.stamp.to_sec()))
            latest[key] = message
            if key == 'imu':
                a, g = message.linear_acceleration, message.angular_velocity
                imu_norms.append((math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z),
                                  math.sqrt(g.x*g.x + g.y*g.y + g.z*g.z)))

    subscribers = [rospy.Subscriber(args.camera.rstrip('/') + '/' + topic,
                                   kind, receive, callback_args=key, queue_size=10)
                   for key, (topic, kind) in specs.items()]
    # Wall time also terminates the check when /use_sim_time is accidentally set.
    time.sleep(args.duration)
    for subscriber in subscribers:
        subscriber.unregister()
    with lock:
        sample_snapshot = {key: list(value) for key, value in samples.items()}
        message_snapshot = dict(latest)
        imu_snapshot = list(imu_norms)
    now = time.monotonic()
    errors = []
    report = {'camera': args.camera, 'duration_s': args.duration, 'streams': {}}
    for key, values in sample_snapshot.items():
        rate = ((len(values) - 1) / (values[-1][0] - values[0][0])
                if len(values) > 1 and values[-1][0] > values[0][0] else 0.0)
        report['streams'][key] = {'count': len(values), 'hz': round(rate, 2),
                                 'frame': message_snapshot[key].header.frame_id if values else ''}
        if len(values) < 2 or now - values[-1][0] > 1.0:
            errors.append(key + ': missing or stale stream')
        elif any(b[1] <= a[1] for a, b in zip(values, values[1:])):
            errors.append(key + ': timestamps are not increasing')
    if 'imu' in message_snapshot:
        imu = message_snapshot['imu']
        accel = [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]
        gyro = [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
        if not np.isfinite(accel + gyro).all():
            errors.append('Non-finite IMU data')
        else:
            report['imu_accel_m_s2'] = accel
            report['imu_gyro_rad_s'] = gyro
            report['imu_accel_norm_m_s2'] = float(np.linalg.norm(accel))
            if not np.any(accel):
                errors.append('IMU acceleration is all zero')
        if not np.isfinite(imu_snapshot).all():
            errors.append('Non-finite IMU samples')
        else:
            mean_accel, mean_gyro = np.mean(imu_snapshot, axis=0)
            report['imu_mean_accel_norm_m_s2'] = float(mean_accel)
            report['imu_mean_gyro_norm_rad_s'] = float(mean_gyro)
            if args.stationary and (not 8.8 <= mean_accel <= 10.8 or mean_gyro > 0.1):
                errors.append('Stationary IMU magnitude is implausible; check SDK/firmware compatibility')
    if all(key in message_snapshot for key in ('color', 'depth', 'info')):
        color, depth, info = (message_snapshot[key] for key in ('color', 'depth', 'info'))
        if not (color.width == depth.width == info.width and
                color.height == depth.height == info.height):
            errors.append('RGB/depth/intrinsics dimensions differ')
        if not (color.header.frame_id == depth.header.frame_id == info.header.frame_id):
            errors.append('RGB/depth/intrinsics optical frames differ')
        if info.K[0] <= 0 or info.K[4] <= 0:
            errors.append('Invalid focal length')
        report['resolution'] = [color.width, color.height]
        report['K'] = list(info.K)
        if depth.encoding not in ('16UC1', '32FC1'):
            errors.append('Unsupported depth encoding: ' + depth.encoding)
        else:
            values = CvBridge().imgmsg_to_cv2(depth).astype(np.float32)
            if depth.encoding == '16UC1':
                values *= 0.001
            valid = values[np.isfinite(values) & (values > 0)]
            report['depth_valid_fraction'] = float(valid.size / values.size)
            report['depth_median_m'] = float(np.median(valid)) if valid.size else None
            if not valid.size:
                errors.append('No valid depth pixels')
        stamps = np.sort([value[1] for value in sample_snapshot['depth']])
        color_stamps = np.asarray([value[1] for value in sample_snapshot['color'][1:-1]])
        if color_stamps.size:
            indices = np.searchsorted(stamps, color_stamps)
            before = stamps[np.clip(indices - 1, 0, len(stamps) - 1)]
            after = stamps[np.clip(indices, 0, len(stamps) - 1)]
            deltas = np.minimum(np.abs(color_stamps - before), np.abs(color_stamps - after))
            report['rgb_depth_p95_ms'] = float(np.percentile(deltas, 95) * 1000)
            if report['rgb_depth_p95_ms'] > 80:
                errors.append('RGB/depth timestamp mismatch exceeds 80 ms')
    report['errors'] = errors
    report['ok'] = not errors
    output = json.dumps(report, indent=2, allow_nan=False)
    print(output)
    if args.output:
        args.output.write_text(output + '\n')
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
