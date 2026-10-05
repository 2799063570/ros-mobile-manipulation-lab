#!/usr/bin/python3
"""Record synchronized expert observations; never invoke robot execution."""
import json
from pathlib import Path
import threading
import time
import uuid

import cv2
import message_filters
import rospy
import tf2_ros
from cv_bridge import CvBridge
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from aubo_vla.actions import vector
from aubo_vla.dataset import EpisodeWriter
from aubo_vla.timing import ClockGuard, positive, source_age


def transform_pose(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return dict(position=[t.x, t.y, t.z], orientation=[q.x, q.y, q.z, q.w])


class Recorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.writer = None
        self.pending = None
        self.camera = None
        self.output_root = Path(rospy.get_param('~output_root', '/tmp/aubo_vla_episodes'))
        self.rate = positive(rospy.get_param('~sample_rate', 2.0), 'sample_rate')
        self.slop = positive(rospy.get_param('~sync_slop', 0.05), 'sync_slop')
        self.max_age = positive(rospy.get_param('~max_age', 1.0), 'max_age')
        self.guard = ClockGuard(bool(rospy.get_param('/use_sim_time', False)),
                                rospy.get_param('~pause_timeout', 2.0))
        self.base = rospy.get_param('~base_frame', 'base_link')
        self.tcp = rospy.get_param('~tcp_frame', 'tcp_link')
        self.joint_names = rospy.get_param('~joint_names')
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError('joint_names must be nonempty and unique')
        self.gripper_joint = rospy.get_param('~gripper_joint', 'joint1')
        self.open, self.closed = vector([rospy.get_param('~gripper_open', 0.0),
                                        rospy.get_param('~gripper_closed', 0.28)], 2, 'gripper calibration')
        if abs(self.closed-self.open) < 1e-9:
            raise ValueError('Gripper calibration endpoints must differ')
        self.instruction = rospy.get_param('~instruction', 'pick up the red block')
        self.scene = rospy.get_param('~scene_id', 'unspecified')
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.convert = CvBridge()
        self.pub = rospy.Publisher('~status', String, queue_size=1, latch=True)
        self.image_topic = rospy.get_param('~image_topic', '/workspace_camera/color/image_raw')
        self.camera_topic = rospy.get_param('~camera_info_topic', '/workspace_camera/color/camera_info')
        self.joint_topic = rospy.get_param('~joint_states_topic', '/aubo_i5/joint_states')
        self.image_sub = message_filters.Subscriber(self.image_topic, Image, queue_size=1,
                                                   buff_size=4*1024*1024)
        self.joint_sub = message_filters.Subscriber(self.joint_topic, JointState, queue_size=20)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.joint_sub], queue_size=30, slop=self.slop, allow_headerless=False)
        self.sync.registerCallback(self.on_sample)
        rospy.Subscriber(self.camera_topic, CameraInfo, self.on_camera, queue_size=1)
        if self.guard.simulated:
            rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=1)
        rospy.Subscriber('~instruction', String, self.on_instruction, queue_size=1)
        rospy.Service('~start', Trigger, self.start)
        rospy.Service('~finish_success', Trigger, lambda _: self.finish('success'))
        rospy.Service('~finish_failure', Trigger, lambda _: self.finish('failure'))
        rospy.Service('~abort', Trigger, lambda _: self.finish('aborted', 'operator abort'))
        rospy.on_shutdown(lambda: self.finish('aborted', 'node shutdown'))
        self.emit('idle')

    def emit(self, state, **extra):
        self.pub.publish(String(data=json.dumps(dict(state=state, observation_only=True,
                                                    **extra), allow_nan=False)))

    def on_camera(self, msg):
        with self.lock:
            self.camera = msg

    def on_instruction(self, msg):
        if not 0 < len(msg.data.strip()) <= 1000:
            rospy.logwarn('Ignored invalid recorder instruction')
            return
        with self.lock:
            self.instruction = msg.data.strip()  # Frozen in manifest by start().

    def on_clock(self, msg):
        with self.lock:
            if self.guard.update(msg.clock.to_sec(), time.monotonic()):
                self.pending = None
                if self.writer:
                    self._finish('aborted', 'simulation paused or clock reset')

    def on_sample(self, image, joints):
        with self.lock:
            if self.writer:
                self.pending = (image, joints, self.camera, time.monotonic(), self.writer)

    def start(self, _):
        with self.lock:
            if self.writer:
                return TriggerResponse(False, 'An episode is already recording')
            if self.guard.paused(time.monotonic()):
                return TriggerResponse(False, 'Simulation clock is unavailable or paused')
            directory = self.output_root/('episode_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
            try:
                self.writer = EpisodeWriter(directory, dict(
                    instruction=self.instruction, scene_id=self.scene,
                    base_frame=self.base, tcp_frame=self.tcp, sample_rate=self.rate,
                    sync_slop=self.slop, joint_names=self.joint_names,
                    image_topic=self.image_topic, camera_info_topic=self.camera_topic,
                    joint_states_topic=self.joint_topic,
                    gripper_calibration=dict(joint=self.gripper_joint, open=self.open, closed=self.closed),
                    simulated=self.guard.simulated,
                    created_unix=time.time(), started_ros=rospy.Time.now().to_sec(),
                    ros_node=rospy.get_name()))
            except (ValueError, OSError) as exc:
                return TriggerResponse(False, str(exc))
            self.pending = None
            self.emit('recording', directory=str(directory), samples=0)
            return TriggerResponse(True, str(directory))

    def _finish(self, outcome, reason):
        writer, self.writer = self.writer, None
        self.pending = None
        writer.finish(outcome, reason)
        self.emit(writer.metadata['state'], directory=str(writer.directory), **{
            k: writer.metadata[k] for k in ('outcome', 'samples', 'rejected', 'finish_reason')})

    def finish(self, outcome, reason='operator label'):
        with self.lock:
            if not self.writer:
                return TriggerResponse(False, 'No episode is recording')
            self._finish(outcome, reason)
            return TriggerResponse(True, outcome)

    def poses_at(self, image, received, writer):
        # TF often arrives just after the matching image. Retry exact-time
        # lookup briefly using wall time; never block on a paused ROS clock.
        deadline = received+min(0.25, self.max_age)
        while True:
            with self.lock:
                if self.writer is not writer or self.guard.paused(time.monotonic()):
                    raise ValueError('Episode ended or paused while waiting for TF')
            try:
                tcp = self.tf.lookup_transform(self.base, self.tcp, image.header.stamp, rospy.Duration(0))
                camera = self.tf.lookup_transform(self.base, image.header.frame_id,
                                                   image.header.stamp, rospy.Duration(0))
                return tcp, camera
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                if time.monotonic() >= deadline or rospy.is_shutdown():
                    raise
                time.sleep(0.01)

    def record(self, item):
        image, joints, camera, received, writer = item
        stamp = image.header.stamp.to_sec()
        if stamp < writer.metadata['started_ros']:
            raise ValueError('Sample predates episode start')
        source_age(stamp, rospy.Time.now().to_sec(), self.max_age)
        if time.monotonic()-received > self.max_age:
            raise ValueError('Pending sample expired')
        if camera is None:
            raise ValueError('No CameraInfo received')
        if camera.width != image.width or camera.height != image.height:
            raise ValueError('CameraInfo/image dimensions mismatch')
        camera_stamp = camera.header.stamp.to_sec()
        if camera_stamp and abs(camera_stamp-stamp) > self.max_age:
            raise ValueError('CameraInfo source is stale')
        if len(joints.name) != len(joints.position) or len(set(joints.name)) != len(joints.name):
            raise ValueError('Invalid JointState feedback')
        positions = dict(zip(joints.name, joints.position))
        required = self.joint_names + [self.gripper_joint]
        if any(name not in positions for name in required):
            raise ValueError('Missing arm or gripper joint feedback: '+', '.join(required))
        raw_gripper = positions[self.gripper_joint]
        gripper = (raw_gripper-self.open)/(self.closed-self.open)
        if not -0.05 <= gripper <= 1.05:
            raise ValueError('Gripper feedback outside calibrated range')
        tcp, camera_tf = self.poses_at(image, received, writer)
        rgb = self.convert.imgmsg_to_cv2(image, 'bgr8')
        ok, png = cv2.imencode('.png', rgb)
        if not ok:
            raise ValueError('PNG encoding failed')
        sample = dict(stamp=stamp, joint_stamp=joints.header.stamp.to_sec(),
                      image_frame=image.header.frame_id, tcp_pose=transform_pose(tcp),
                      camera_pose=transform_pose(camera_tf),
                      joints=dict(names=self.joint_names, positions=[positions[n] for n in self.joint_names]),
                      gripper=max(0., min(1., gripper)), gripper_joint_position=raw_gripper,
                      camera_info=dict(frame_id=camera.header.frame_id, width=camera.width,
                                       height=camera.height, distortion_model=camera.distortion_model,
                                       K=list(camera.K), D=list(camera.D), R=list(camera.R), P=list(camera.P)))
        with self.lock:
            if self.writer is not writer or self.guard.paused(time.monotonic()):
                return
            source_age(stamp, rospy.Time.now().to_sec(), self.max_age)
            writer.append(sample, png.tobytes())
            self.emit('recording', directory=str(writer.directory),
                      samples=writer.metadata['samples'], rejected=writer.metadata['rejected'])

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.writer and self.guard.paused(time.monotonic()):
                    self._finish('aborted', 'simulation paused')
                item, self.pending = self.pending, None
                if item and item[4].last_stamp is not None:
                    stamp = item[0].header.stamp.to_sec()
                    if stamp <= item[4].last_stamp:
                        self._finish('aborted', 'image timestamp rollback or duplicate')
                        item = None
                    elif stamp-item[4].last_stamp < 1/self.rate - 1e-6:
                        item = None
            if item:
                try:
                    self.record(item)
                except Exception as exc:
                    with self.lock:
                        if self.writer is item[4]:
                            self.writer.reject(exc)
                            self.emit('rejected', reason=str(exc),
                                      directory=str(self.writer.directory),
                                      samples=self.writer.metadata['samples'],
                                      rejected=self.writer.metadata['rejected'])
                    rospy.logwarn_throttle(5, 'VLA recording rejected sample: %s', str(exc))
            time.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('vla_recorder')
    Recorder().run()
