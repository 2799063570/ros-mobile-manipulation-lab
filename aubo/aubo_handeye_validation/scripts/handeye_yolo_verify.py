#!/usr/bin/env python3
"""Slow, single-target eye-on-base validation using existing YOLO 3-D detections."""
import copy
import inspect
import json
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np
import yaml


def finite_vector(values, size):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('expected {} finite values'.format(size))
    return result


def quaternion_angle(a, b):
    a, b = finite_vector(a, 4), finite_vector(b, 4)
    if min(np.linalg.norm(a), np.linalg.norm(b)) < 1e-9:
        raise ValueError('zero quaternion')
    return 2*math.acos(min(1., float(abs(np.dot(a/np.linalg.norm(a), b/np.linalg.norm(b))))))


def downward_quaternion(yaw):
    """Quaternion for TCP roll=pi, pitch=0 and the requested base-frame yaw."""
    return [math.cos(yaw/2), math.sin(yaw/2), 0.0, 0.0]


def stable_target(samples, jitter, table_z, height_tolerance, require_depth):
    """Samples are unique, fresh, unambiguous observations from one class."""
    if not samples:
        raise ValueError('missing observations')
    xyz = np.asarray([finite_vector(s['xyz'], 3) for s in samples])
    heights = finite_vector([s['height'] for s in samples], len(samples))
    if not len(samples) or np.any(heights <= 0):
        raise ValueError('missing observations or invalid object height')
    if require_depth and not all(s['depth_valid'] for s in samples):
        raise ValueError('depth mode requires validated depth for every observation')
    if np.max(np.abs(xyz[:, 2] - table_z - heights/2)) > height_tolerance:
        raise ValueError('object center disagrees with configured table/object height')
    raw_angles = [s.get('grasp_angle', 0.0) for s in samples]
    if any(angle is None for angle in raw_angles):
        raise ValueError('grasp angle unavailable; YOLO OBB orientation is required')
    angles = np.asarray(raw_angles, dtype=float)
    if angles.shape != (len(samples),) or not np.isfinite(angles).all():
        raise ValueError('grasp angle unavailable; YOLO OBB orientation is required')
    angle = 0.5 * math.atan2(float(np.mean(np.sin(2*angles))),
                             float(np.mean(np.cos(2*angles))))
    angle_errors = (angles-angle+math.pi/2) % math.pi-math.pi/2
    angle_tolerance = float(samples[0].get('grasp_angle_tolerance', math.radians(8)))
    if float(np.max(np.abs(angle_errors))) > angle_tolerance:
        raise ValueError('grasp angle is unstable')
    center = np.median(xyz, axis=0)
    if np.max(np.linalg.norm(xyz-center, axis=1)) > jitter:
        raise ValueError('target is moving or unstable')
    tops = xyz[:, 2] + heights/2
    if float(np.ptp(tops)) > jitter*2:
        raise ValueError('inconsistent target surface height')
    return dict(xyz=center.tolist(), height=float(np.median(heights)), grasp_angle=angle,
                surface_z=float(np.max(tops)), stamp_ns=samples[-1]['stamp_ns'])


def verification_waypoints(current, orientation, target, config):
    """Lift, align in place, translate overhead, then descend to a noncontact hover."""
    start = finite_vector(current, 3)
    xyz = finite_vector(target['xyz'], 3)
    bounds = config['workspace']
    for i, axis in enumerate('xy'):
        if not bounds['min_'+axis] <= xyz[i] <= bounds['max_'+axis]:
            raise ValueError('target outside verification workspace')
    hover = target['surface_z'] + config['hover_clearance']
    high = max(start[2], target['surface_z'] + config['travel_clearance'], hover)
    if start[2] < config['table_z'] + config['minimum_tcp_table_clearance']:
        raise ValueError('starting TCP is too low; move to a clear observation pose first')
    if high > config['max_tcp_z'] or hover > config['max_tcp_z']:
        raise ValueError('required overhead clearance exceeds max_tcp_z')
    q = finite_vector(orientation, 4).tolist()
    yaw = config['downward_yaw'] + target.get('grasp_angle', 0.0)
    # roll=pi, pitch=0: local TCP +Z points along base -Z.
    down = downward_quaternion(yaw)
    return [dict(xyz=[start[0], start[1], high], q=q),
            dict(xyz=[start[0], start[1], high], q=down),
            dict(xyz=[xyz[0], xyz[1], high], q=down),
            dict(xyz=[xyz[0], xyz[1], hover], q=down)]


def polynomial_peak(coefficients):
    candidates = [0., 1.]
    for root in np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(coefficients)):
        if abs(root.imag) < 1e-8 and 0 < root.real < 1:
            candidates.append(float(root.real))
    return float(np.max(np.abs(np.polynomial.polynomial.polyval(candidates, coefficients))))


def slow_time_factor(positions, velocities, accelerations, times, tcp_positions, config):
    """Additional uniform dilation after MoveIt retiming; derivatives scale too."""
    times = finite_vector(times, len(times))
    positions = np.asarray(positions, dtype=float)
    velocities = np.asarray(velocities, dtype=float)
    accelerations = np.asarray(accelerations, dtype=float)
    tcp = np.asarray(tcp_positions, dtype=float)
    if (len(times) < 2 or positions.ndim != 2 or positions.shape != velocities.shape or
            positions.shape != accelerations.shape or positions.shape[0] != len(times) or
            tcp.shape != (len(times), 3) or not all(np.isfinite(v).all() for v in
                (positions, velocities, accelerations, tcp))):
        raise ValueError('invalid retimed trajectory or missing velocity/acceleration fields')
    dt = np.diff(times)
    if times[0] < 0 or np.any(dt <= 0):
        raise ValueError('trajectory timestamps must strictly increase')
    dq = np.abs(np.diff(positions, axis=0))
    if np.max(dq) > config['max_joint_step']:
        raise ValueError('joint jump in Cartesian trajectory; no pose-planning fallback')
    # Check continuous quintic joint interpolation, including extrema between samples.
    # Uniform time dilation preserves this spline and scales v by 1/k, a by 1/k^2.
    peak_v, peak_a = 0., 0.
    for i, interval in enumerate(dt):
        for j in range(positions.shape[1]):
            delta = positions[i+1, j]-positions[i, j]
            v0, v1 = velocities[i, j]*interval, velocities[i+1, j]*interval
            a0, a1 = accelerations[i, j]*interval**2, accelerations[i+1, j]*interval**2
            coefficients = [positions[i, j], v0, a0/2,
                            10*delta-6*v0-4*v1-1.5*a0+.5*a1,
                            -15*delta+8*v0+7*v1+1.5*a0-a1,
                            6*delta-3*v0-3*v1-.5*a0+.5*a1]
            velocity = np.polynomial.polynomial.polyder(coefficients)
            acceleration = np.polynomial.polynomial.polyder(velocity)
            peak_v = max(peak_v, polynomial_peak(velocity)/interval)
            peak_a = max(peak_a, polynomial_peak(acceleration)/interval**2)
    return float(max(1., peak_v/config['max_joint_speed'],
        math.sqrt(peak_a/config['max_joint_acceleration']),
        np.max(np.abs(velocities))/config['max_joint_speed'],
        math.sqrt(np.max(np.abs(accelerations))/config['max_joint_acceleration']),
        np.max(2*dq/dt[:, None])/config['max_joint_speed'],
        np.max(2*np.linalg.norm(np.diff(tcp, axis=0), axis=1)/dt)/config['max_tcp_speed']))


def validate_config(config):
    for key in ('velocity_scaling', 'acceleration_scaling', 'max_joint_speed',
                'max_joint_acceleration', 'max_tcp_speed', 'max_joint_step', 'eef_step',
                'hover_clearance', 'travel_clearance', 'max_tcp_z', 'target_max_age',
                'target_jitter', 'height_tolerance', 'sample_timeout', 'sample_window',
                'planning_time', 'state_max_age', 'execution_timeout', 'orientation_tolerance',
                'minimum_tcp_table_clearance', 'target_recheck_distance', 'max_start_error',
                'target_recheck_timeout', 'final_position_tolerance'):
        value = float(config[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError('{} must be finite and positive'.format(key))
        config[key] = value
    for key in ('table_z', 'downward_yaw', 'grasp_angle_tolerance'):
        config[key] = float(config[key])
        if not math.isfinite(config[key]):
            raise ValueError('invalid '+key)
    if config['grasp_angle_tolerance'] <= 0 or config['grasp_angle_tolerance'] > math.pi/2:
        raise ValueError('grasp_angle_tolerance must be between 0 and pi/2')
    if (config['velocity_scaling'] > .10 or config['acceleration_scaling'] > .10 or
            config['max_joint_speed'] > .10 or config['max_joint_acceleration'] > .20 or
            config['max_tcp_speed'] > .03):
        raise ValueError('verification speed caps exceeded; use the normal task for faster motion')
    if config['hover_clearance'] < .05 or config['travel_clearance'] < config['hover_clearance']:
        raise ValueError('hover clearance must be >= 0.05 m and travel clearance >= hover clearance')
    if not 0 < float(config['min_confidence']) <= 1 or int(config['min_samples']) < 3:
        raise ValueError('invalid confidence or min_samples < 3')
    config['min_samples'] = int(config['min_samples'])
    if config['sample_timeout'] < config['sample_window'] or config['sample_window'] < .2:
        raise ValueError('sample window must be >= 0.2 s and <= sample timeout')
    for axis in 'xy':
        low, high = float(config['workspace']['min_'+axis]), float(config['workspace']['max_'+axis])
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise ValueError('invalid workspace bounds')
    if not config['target_class'] or config['height_mode'] not in ('depth', 'table'):
        raise ValueError('set a target_class and height_mode depth|table')
    return config


def calibration_snapshot(path, target_frame):
    with open(os.path.expanduser(path), encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    params = data['parameters']
    if params.get('eye_on_hand') is not False or params.get('robot_base_frame') != target_frame:
        raise ValueError('need eye-on-base calibration with the exact target base frame')
    if not params.get('tracking_base_frame'):
        raise ValueError('calibration is missing the camera optical frame')
    transform = data['transformation']
    finite_vector([transform[k] for k in ('x', 'y', 'z')], 3)
    q = finite_vector([transform[k] for k in ('qx', 'qy', 'qz', 'qw')], 4)
    if np.linalg.norm(q) < 1e-9:
        raise ValueError('calibration quaternion is zero')
    return data


class HandeyeYoloVerify:
    def __init__(self):
        from actionlib import SimpleActionClient
        from aubo_perception.msg import DetectedObjectArray
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from moveit_msgs.msg import DisplayTrajectory, ExecuteTrajectoryAction
        from moveit_msgs.srv import GetPositionFK, GetStateValidity
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String, Bool
        from std_srvs.srv import Trigger
        import tf2_ros

        self.config = validate_config(rospy.get_param('~'))
        c = self.config
        self.calibration = calibration_snapshot(c['calibration_file'], c['target_frame'])
        self.lock, self.data_lock, self.dispatch_lock = threading.Lock(), threading.Lock(), threading.Lock()
        self.cancel = threading.Event()
        self.history, self.joints = deque(maxlen=300), {}
        self.scene_ready = False
        self.last_detection_stamp = -1
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.arm = moveit_commander.MoveGroupCommander(c['planning_group'], wait_for_servers=30.)
        self.arm.set_end_effector_link(c['end_effector_link'])
        self.arm.set_pose_reference_frame(c['target_frame'])
        self.arm.set_max_velocity_scaling_factor(c['velocity_scaling'])
        self.arm.set_max_acceleration_scaling_factor(c['acceleration_scaling'])
        self.arm.set_planning_time(c['planning_time'])
        self.arm.set_goal_position_tolerance(.002)
        self.arm.set_goal_orientation_tolerance(.01)
        self.active_joints = self.arm.get_active_joints()
        self.fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        self.validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        rospy.wait_for_service('/compute_fk', timeout=30.)
        rospy.wait_for_service('/check_state_validity', timeout=30.)
        self.executor = SimpleActionClient('/execute_trajectory', ExecuteTrajectoryAction)
        if not self.executor.wait_for_server(rospy.Duration(30.)):
            raise RuntimeError('MoveIt execute_trajectory action unavailable')
        self.preview = rospy.Publisher('~display_trajectory', DisplayTrajectory, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('~goal', PoseStamped, queue_size=1, latch=True)
        self.status = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.target_tf_frame = rospy.get_param('~target_tf_frame', 'handeye_verify_target')
        self.grasp_goal_tf_frame = rospy.get_param('~grasp_goal_tf_frame', 'handeye_verify_grasp_goal')
        self.target_tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.transform_stamped_type = TransformStamped
        self.subscribers = [rospy.Subscriber(c['detections_topic'], DetectedObjectArray,
                                            self.detections, queue_size=5),
                            rospy.Subscriber(c['joint_states_topic'], JointState,
                                             self.joint_states, queue_size=20),
                            rospy.Subscriber('/hybrid/planning_scene_ready', Bool,
                                             self.scene_state, queue_size=1)]
        self.services = [rospy.Service('~plan', Trigger, lambda _: self.run(False)),
                         rospy.Service('~move', Trigger, lambda _: self.run(True)),
                         rospy.Service('~stop', Trigger, self.stop)]
        rospy.on_shutdown(self.cancel_motion)
        self.publish_status('IDLE: call ~plan for preview or ~move for one slow verification move')

    def publish_status(self, text):
        self.status.publish(text)
        rospy.loginfo('[handeye_yolo_verify] %s', text)

    def joint_states(self, message):
        if len(message.name) != len(message.position) or not all(math.isfinite(v) for v in message.position):
            return
        with self.data_lock:
            for index, (name, position) in enumerate(zip(message.name, message.position)):
                stamp = message.header.stamp.to_sec()
                previous = self.joints.get(name)
                if previous is not None and stamp <= previous[1]:
                    continue
                if len(message.velocity) == len(message.name) and math.isfinite(message.velocity[index]):
                    speed = abs(message.velocity[index])
                elif previous is not None:
                    speed = abs((position-previous[0])/(stamp-previous[1]))
                else:
                    speed = float('inf')
                self.joints[name] = (position, stamp, speed)

    def scene_state(self, message):
        self.scene_ready = message.data

    def detections(self, message):
        c = self.config
        stamp = message.header.stamp.to_nsec()
        age = (rospy.Time.now()-message.header.stamp).to_sec()
        candidate = None
        if (message.observation_valid and message.header.frame_id == c['target_frame'] and
                message.sensor_frame == self.calibration['parameters']['tracking_base_frame'] and
                stamp > 0 and 0 <= age <= c['target_max_age']):
            matches = [o for o in message.objects if (o.class_name or o.color) == c['target_class']]
            if len(matches) == 1:
                obj = matches[0]
                p = obj.pose.position
                if (math.isfinite(obj.confidence) and obj.confidence >= c['min_confidence'] and
                        all(math.isfinite(v) for v in (p.x, p.y, p.z, obj.object_height))):
                    candidate = dict(xyz=[p.x, p.y, p.z], height=obj.object_height,
                                     depth_valid=obj.depth_valid, stamp_ns=stamp,
                                     grasp_angle=(float(obj.grasp_angle)
                                                  if obj.grasp_geometry_valid and
                                                  math.isfinite(obj.grasp_angle) else None),
                                     grasp_angle_tolerance=c['grasp_angle_tolerance'])
        if candidate is not None:
            transform = self.transform_stamped_type()
            transform.header.stamp = message.header.stamp
            transform.header.frame_id = c['target_frame']
            transform.child_frame_id = self.target_tf_frame
            transform.transform.translation.x = candidate['xyz'][0]
            transform.transform.translation.y = candidate['xyz'][1]
            transform.transform.translation.z = candidate['xyz'][2]
            object_angle = candidate['grasp_angle'] if candidate['grasp_angle'] is not None else 0.0
            half_angle = 0.5 * object_angle
            transform.transform.rotation.z = math.sin(half_angle)
            transform.transform.rotation.w = math.cos(half_angle)
            self.target_tf_broadcaster.sendTransform(transform)
            if candidate['grasp_angle'] is not None:
                goal_transform = self.transform_stamped_type()
                goal_transform.header.stamp = message.header.stamp
                goal_transform.header.frame_id = c['target_frame']
                goal_transform.child_frame_id = self.grasp_goal_tf_frame
                goal_transform.transform.translation.x = candidate['xyz'][0]
                goal_transform.transform.translation.y = candidate['xyz'][1]
                goal_transform.transform.translation.z = (
                    candidate['xyz'][2] + candidate['height']/2 + c['hover_clearance'])
                goal_quaternion = downward_quaternion(
                    c['downward_yaw'] + candidate['grasp_angle'])
                (goal_transform.transform.rotation.x,
                 goal_transform.transform.rotation.y,
                 goal_transform.transform.rotation.z,
                 goal_transform.transform.rotation.w) = goal_quaternion
                self.target_tf_broadcaster.sendTransform(goal_transform)
        with self.data_lock:
            # Duplicates do not count as new frames. Invalid frames break continuity.
            if stamp <= self.last_detection_stamp:
                return
            self.last_detection_stamp = stamp
            if candidate is None:
                self.history.clear()
            else:
                self.history.append(candidate)

    def current_state(self, stationary=False):
        now = rospy.Time.now().to_sec()
        with self.data_lock:
            values = {name: self.joints.get(name) for name in self.active_joints}
        if any(value is None or not 0 <= now-value[1] <= self.config['state_max_age']
               or value[1] <= 0 for value in values.values()):
            raise RuntimeError('missing/stale real joint feedback')
        if stationary and any(value[2] > .02 for value in values.values()):
            raise RuntimeError('robot must be stationary before planning/execution')
        state = self.robot.get_current_state()
        # Use the measured arm joints, retaining non-arm joints from RobotCommander.
        return self.state_at(state, self.active_joints, [values[n][0] for n in self.active_joints])

    @staticmethod
    def state_at(state, names, positions):
        result = copy.deepcopy(state)
        values = dict(zip(result.joint_state.name, result.joint_state.position))
        values.update(zip(names, positions))
        result.joint_state.name = list(values)
        result.joint_state.position = list(values.values())
        result.joint_state.velocity = []
        result.joint_state.effort = []
        result.joint_state.header.stamp = rospy.Time(0)
        result.is_diff = False
        return result

    def fk_pose(self, state):
        from moveit_msgs.srv import GetPositionFKRequest
        request = GetPositionFKRequest()
        request.header.frame_id = self.config['target_frame']
        request.fk_link_names = [self.config['end_effector_link']]
        request.robot_state = state
        result = self.fk(request)
        if result.error_code.val != 1 or len(result.pose_stamped) != 1:
            raise RuntimeError('MoveIt FK failed')
        return result.pose_stamped[0].pose

    @staticmethod
    def xyz(pose):
        return [pose.position.x, pose.position.y, pose.position.z]

    @staticmethod
    def xyzw(pose):
        q = pose.orientation
        return [q.x, q.y, q.z, q.w]

    @staticmethod
    def pose(waypoint):
        from geometry_msgs.msg import Pose
        result = Pose()
        result.position.x, result.position.y, result.position.z = waypoint['xyz']
        result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = waypoint['q']
        return result

    def check_cancel(self):
        if self.cancel.is_set() or rospy.is_shutdown():
            raise RuntimeError('verification stopped')

    def acquire_target(self):
        start_stamp = rospy.Time.now().to_nsec()
        deadline = time.monotonic() + self.config['sample_timeout']
        last_error = 'waiting for one unambiguous target'
        while time.monotonic() < deadline:
            self.check_cancel()
            now = rospy.Time.now().to_nsec()
            with self.data_lock:
                samples = [copy.deepcopy(s) for s in self.history if s['stamp_ns'] > start_stamp and
                           0 <= (now-s['stamp_ns'])/1e9 <= self.config['target_max_age']]
            if len(samples) >= self.config['min_samples']:
                span = (samples[-1]['stamp_ns']-samples[0]['stamp_ns'])/1e9
                if span >= self.config['sample_window']:
                    try:
                        return stable_target(samples, self.config['target_jitter'], self.config['table_z'],
                                             self.config['height_tolerance'], self.config['height_mode'] == 'depth')
                    except ValueError as error:
                        last_error = str(error)
            time.sleep(.02)
        raise RuntimeError('no stable YOLO target: '+last_error)

    def plan(self, target):
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.msg import DisplayTrajectory
        c = self.config
        if not self.scene_ready or 'hybrid_sorting_table' not in self.scene.get_known_object_names():
            raise RuntimeError('verification table collision scene is not ready')
        start = self.current_state(stationary=True)
        current = self.fk_pose(start)
        waypoints = verification_waypoints(self.xyz(current), self.xyzw(current), target, c)
        self.arm.clear_pose_targets()
        self.arm.clear_path_constraints()
        self.arm.set_start_state(start)
        kwargs = dict(avoid_collisions=True)
        if 'jump_threshold' in inspect.signature(self.arm.compute_cartesian_path).parameters:
            kwargs['jump_threshold'] = 0.
        trajectory, fraction = self.arm.compute_cartesian_path(
            [self.pose(p) for p in waypoints], c['eef_step'], **kwargs)
        self.check_cancel()
        if not math.isfinite(fraction) or fraction < 1.:
            raise RuntimeError('Cartesian path incomplete ({:.1%}); motion refused'.format(fraction))
        # No fallback to joint-space pose planning: the lift/align/overhead order matters.
        trajectory = self.arm.retime_trajectory(start, trajectory, c['velocity_scaling'], c['acceleration_scaling'])
        points, names = trajectory.joint_trajectory.points, trajectory.joint_trajectory.joint_names
        if len(points) < 2 or set(names) != set(self.active_joints):
            raise RuntimeError('empty trajectory or unexpected controlled joints')
        tcp_positions = []
        initial = finite_vector(self.xyz(current), 3)
        goal = waypoints[-1]
        for point in points:
            self.check_cancel()
            if len(point.positions) != len(names):
                raise RuntimeError('invalid trajectory joint dimensions')
            state = self.state_at(start, names, point.positions)
            if not self.validity(robot_state=state, group_name=c['planning_group']).valid:
                raise RuntimeError('planned state is in collision or violates constraints')
            pose = self.fk_pose(state)
            xyz = finite_vector(self.xyz(pose), 3)
            tcp_positions.append(xyz.tolist())
            if xyz[2] < c['table_z']+c['minimum_tcp_table_clearance'] or xyz[2] > c['max_tcp_z']:
                raise RuntimeError('TCP path exceeds vertical limits')
            lateral = np.linalg.norm(xyz[:2]-initial[:2]) > .003
            down = quaternion_angle(self.xyzw(pose), goal['q']) <= c['orientation_tolerance']
            changed = quaternion_angle(self.xyzw(pose), self.xyzw(current)) > c['orientation_tolerance']
            if lateral and (not down or xyz[2] < goal['xyz'][2]-.003):
                raise RuntimeError('TCP must stay downward and above hover height during transfer')
            if changed and not down and xyz[2] < waypoints[0]['xyz'][2]-.003:
                raise RuntimeError('orientation alignment must occur at overhead clearance')
        if (np.linalg.norm(np.asarray(tcp_positions[-1])-goal['xyz']) > .003 or
                quaternion_angle(self.xyzw(pose), goal['q']) > c['orientation_tolerance']):
            raise RuntimeError('planned endpoint does not match downward hover target')
        # Confirm trajectory starts at the measured state; never permit an initial jump.
        measured = dict(zip(start.joint_state.name, start.joint_state.position))
        if max(abs(points[0].positions[i]-measured[n]) for i, n in enumerate(names)) > c['max_start_error']:
            raise RuntimeError('trajectory start does not match measured joints')
        factor = slow_time_factor([p.positions for p in points], [p.velocities for p in points],
                                  [p.accelerations for p in points],
                                  [p.time_from_start.to_sec() for p in points], tcp_positions, c)
        for point in points:
            point.time_from_start = rospy.Duration.from_sec(point.time_from_start.to_sec()*factor)
            point.velocities = [v/factor for v in point.velocities]
            point.accelerations = [a/(factor*factor) for a in point.accelerations]
        # Keep a fresh start stamp; execution should not inherit an old planned header.
        trajectory.joint_trajectory.header.stamp = rospy.Time(0)
        goal_message = PoseStamped()
        goal_message.header.frame_id = c['target_frame']
        goal_message.header.stamp = rospy.Time.now()
        goal_message.pose = self.pose(goal)
        self.goal_pub.publish(goal_message)
        self.preview.publish(DisplayTrajectory(trajectory_start=start, trajectory=[trajectory]))
        return start, trajectory, goal

    def recheck_target(self, target, after_stamp_ns):
        c = self.config
        deadline = time.monotonic() + c['target_recheck_timeout']
        last_reason = 'waiting for new observations after planning'
        valid_count, latest_age = 0, None
        while time.monotonic() < deadline:
            self.check_cancel()
            now = rospy.Time.now().to_nsec()
            with self.data_lock:
                samples = [copy.deepcopy(s) for s in self.history
                           if s['stamp_ns'] > after_stamp_ns and
                           0 <= (now-s['stamp_ns'])/1e9 <= c['target_max_age']]
            valid_count = len(samples)
            latest_age = ((now-max(s['stamp_ns'] for s in samples))/1e9
                          if samples else None)
            if len(samples) >= c['min_samples']:
                span = (samples[-1]['stamp_ns']-samples[0]['stamp_ns'])/1e9
                if span >= c['sample_window']:
                    try:
                        latest = stable_target(samples, c['target_jitter'], c['table_z'],
                                               c['height_tolerance'], c['height_mode'] == 'depth')
                    except ValueError as error:
                        last_reason = str(error)
                    else:
                        position_delta = float(np.linalg.norm(
                            np.asarray(latest['xyz'])-target['xyz']))
                        if position_delta > c['target_recheck_distance']:
                            raise RuntimeError(
                                'YOLO target moved after planning ({:.1f} mm); motion refused'.format(
                                    position_delta*1000))
                        angle_delta = abs((latest['grasp_angle']-target['grasp_angle']+
                                           math.pi/2) % math.pi-math.pi/2)
                        if angle_delta > c['grasp_angle_tolerance']:
                            raise RuntimeError(
                                'YOLO grasp angle changed after planning ({:.1f} deg); motion refused'.format(
                                    math.degrees(angle_delta)))
                        return
                else:
                    last_reason = 'new observations span only {:.2f} s'.format(span)
            time.sleep(.02)
        age_text = 'none' if latest_age is None else '{:.3f} s'.format(latest_age)
        raise RuntimeError(
            'post-plan target confirmation timed out: {} fresh observations, latest age {}; {}. '
            'Motion refused'.format(valid_count, age_text, last_reason))

    def cancel_motion(self):
        # Serialize cancellation with dispatch so a stop cannot race with send_goal.
        with self.dispatch_lock:
            self.cancel.set()
            self.executor.cancel_goal()

    def stop(self, _request):
        from std_srvs.srv import TriggerResponse
        self.cancel_motion()
        self.publish_status('STOP_REQUESTED')
        return TriggerResponse(success=True, message='verification trajectory cancellation requested')

    def execute(self, start, trajectory):
        from moveit_msgs.msg import ExecuteTrajectoryGoal
        measured = self.current_state(stationary=True)
        before = dict(zip(start.joint_state.name, start.joint_state.position))
        now = dict(zip(measured.joint_state.name, measured.joint_state.position))
        if max(abs(now[n]-before[n]) for n in self.active_joints) > self.config['max_start_error']:
            raise RuntimeError('robot moved during planning; motion refused')
        with self.dispatch_lock:
            self.check_cancel()
            self.executor.send_goal(ExecuteTrajectoryGoal(trajectory=trajectory))
        duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
        deadline = time.monotonic() + max(self.config['execution_timeout'], duration*1.5+10)
        while time.monotonic() < deadline:
            self.check_cancel()
            self.current_state()  # Stop if real joint feedback is lost during execution.
            status = self.executor.get_state()
            if status == 3:
                result = self.executor.get_result()
                if result is None or result.error_code.val != 1:
                    raise RuntimeError('MoveIt execution result reports failure')
                return
            if status in (2, 4, 5, 8, 9):
                raise RuntimeError('MoveIt execution failed, action state {}'.format(status))
            time.sleep(.05)
        raise RuntimeError('verification execution timed out')

    def record(self, report):
        directory = os.path.expanduser(self.config['output_directory'])
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, 'move_{}_{}.json'.format(time.time_ns(), os.getpid()))
        with open(path, 'x', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        return path

    def run(self, move):
        from std_srvs.srv import TriggerResponse
        with self.dispatch_lock:
            if not self.lock.acquire(False):
                return TriggerResponse(success=False, message='verification is busy')
            self.cancel.clear()
        sent, report = False, None
        try:
            self.publish_status('ACQUIRING_TARGET')
            target = self.acquire_target()
            self.publish_status('PLANNING')
            start, trajectory, goal = self.plan(target)
            duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
            report = dict(mode='move' if move else 'plan_only', target=target, goal=goal,
                          frame=self.config['target_frame'], tcp=self.config['end_effector_link'],
                          duration_s=duration, config=self.config, calibration=self.calibration,
                          note='Robot feedback error is tracking error, not independent hand-eye accuracy.')
            # Require writable evidence output before dispatching real motion.
            report['state'] = 'planned'
            path = self.record(report)
            if not move:
                self.publish_status('PLANNED: {:.1f} s; {}'.format(duration, path))
                return TriggerResponse(success=True, message='preview only; {:.1f} s; {}'.format(duration, path))
            self.publish_status('REVALIDATING_TARGET')
            self.recheck_target(target, rospy.Time.now().to_nsec())
            self.publish_status('MOVING: lift, align downward, transfer, hover')
            sent = True
            self.execute(start, trajectory)
            actual = self.fk_pose(self.current_state())
            position_error = float(np.linalg.norm(np.asarray(self.xyz(actual))-goal['xyz'])*1000)
            rotation_error = math.degrees(quaternion_angle(self.xyzw(actual), goal['q']))
            if (position_error > self.config['final_position_tolerance']*1000 or
                    rotation_error > math.degrees(self.config['orientation_tolerance'])):
                raise RuntimeError('final robot feedback does not match commanded hover pose')
            report.update(state='completed', actual_tcp_xyz=self.xyz(actual), actual_tcp_q=self.xyzw(actual),
                          tracking_position_error_mm=position_error, tracking_rotation_error_deg=rotation_error)
            path = self.record(report)
            message = 'hover reached; tracking error {:.2f} mm / {:.2f} deg; {}'.format(
                position_error, rotation_error, path)
            self.publish_status('COMPLETED: '+message)
            return TriggerResponse(success=True, message=message)
        except Exception as error:
            if sent:
                self.cancel_motion()
            if report is not None:
                report.update(state='failed', error=str(error))
                try:
                    self.record(report)
                except OSError as record_error:
                    rospy.logerr('Could not save verification failure: %s', record_error)
            self.publish_status('FAILED: '+str(error))
            return TriggerResponse(success=False, message=str(error))
        finally:
            try:
                self.arm.clear_pose_targets()
            finally:
                self.lock.release()


def main():
    global rospy, moveit_commander
    import rospy
    import moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('handeye_yolo_verify')
    try:
        HandeyeYoloVerify()
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
