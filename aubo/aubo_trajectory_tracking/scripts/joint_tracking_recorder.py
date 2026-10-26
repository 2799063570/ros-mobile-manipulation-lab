#!/usr/bin/env python3
"""Read-only synchronization of generated joint targets and measured joints.

Gazebo targets approximate dispatched commands. SDK targets are producer positions,
not cabinet execution timestamps: SDK buffer lead remains part of their difference.
"""
import bisect
from collections import deque
import math
import threading
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String, Float64MultiArray


class TargetHistory:
    def __init__(self):
        self.rows = deque(maxlen=1000)

    def add(self, stamp, position, velocity):
        if self.rows and stamp < self.rows[-1][0]:
            self.rows.clear()
        if self.rows and stamp == self.rows[-1][0]:
            return
        self.rows.append((stamp, list(position), list(velocity)))

    def at(self, stamp, max_gap=.06):
        if not self.rows:
            return None
        rows = list(self.rows)
        index = bisect.bisect_left([row[0] for row in rows], stamp)
        if index < len(rows) and abs(rows[index][0]-stamp) < 1e-8:
            return rows[index][1], rows[index][2], 0.
        if index == 0 or index == len(rows):
            return None  # Never fabricate an extrapolated target.
        left, right = rows[index-1], rows[index]
        if right[0]-left[0] > max_gap:
            return None
        weight = (stamp-left[0])/(right[0]-left[0])
        interpolate = lambda a, b: [x+weight*(y-x) for x, y in zip(a, b)]
        return interpolate(left[1], right[1]), interpolate(left[2], right[2]), max(stamp-left[0], right[0]-stamp)


def ordered(message, names, field):
    values = getattr(message, field)
    if len(values) != len(message.name) or len(set(message.name)) != len(message.name):
        return None
    lookup = dict(zip(message.name, values))
    if any(name not in lookup or not math.isfinite(lookup[name]) for name in names):
        return None
    return [lookup[name] for name in names]


class JointRecorder:
    def __init__(self):
        self.lock = threading.RLock()
        self.history = TargetHistory()
        self.pending = deque(maxlen=100)
        self.previous = None
        self.run_start = None
        self.stop_deadline = None
        self.backend = None
        self.dropped = 0
        self.diagnostic_stamp = None
        self.names = rospy.get_param('/aubo_cartesian_jog/joint_names', [])
        if len(self.names) != 6:
            raise ValueError('Joint recorder requires six backend joint names')
        tracker = rospy.get_param('~tracker_ns', '/trajectory_tracking')
        jog = rospy.get_param(tracker+'/jog_ns', '/cartesian_jog')
        topic = rospy.get_param(tracker+'/joint_states_topic', '/cartesian_jog/sdk_joint_states')
        self.samples = rospy.Publisher(tracker+'/joint_samples', Float64MultiArray, queue_size=100)
        self.servo = rospy.Publisher(tracker+'/servo_samples', Float64MultiArray, queue_size=100)
        self.status = rospy.Publisher(tracker+'/joint_recording_state', String, queue_size=1, latch=True)
        self.subs = [rospy.Subscriber(jog+'/joint_target', JointState, self.target, queue_size=100),
                     rospy.Subscriber(jog+'/servo_diagnostics', Float64MultiArray, self.diagnostics, queue_size=100),
                     rospy.Subscriber(topic, JointState, self.feedback, queue_size=100),
                     rospy.Subscriber(tracker+'/run_start', Header, self.start, queue_size=1),
                     rospy.Subscriber(tracker+'/state', String, self.state, queue_size=1)]
        self.status.publish(String('等待本轮运行和底层关节目标遥测'))
        self.worker = threading.Thread(target=self.flush_loop, daemon=True)
        self.worker.start()

    def active(self):
        return self.run_start is not None and (self.stop_deadline is None or time.monotonic() <= self.stop_deadline)

    def start(self, msg):
        with self.lock:
            self.run_start = None if msg.stamp.is_zero() else msg.stamp.to_sec()
            self.stop_deadline = None
            self.pending.clear()
            self.dropped = 0
            self.diagnostic_stamp = None
            self.samples.publish(Float64MultiArray())
            self.servo.publish(Float64MultiArray())
            if self.run_start is not None:
                self.status.publish(String('正在记录六轴关节；停止后继续记录 2 秒减速尾段'))

    def state(self, msg):
        with self.lock:
            if msg.data.startswith(('COMPLETED:', 'STOPPED:', 'FAULT:')) and self.run_start is not None and self.stop_deadline is None:
                self.stop_deadline = time.monotonic()+2.

    def target(self, msg):
        position, velocity = ordered(msg, self.names, 'position'), ordered(msg, self.names, 'velocity')
        if position is None or velocity is None or msg.header.stamp.is_zero():
            return
        with self.lock:
            self.history.add(msg.header.stamp.to_sec(), position, velocity)
            self.drain()

    def feedback(self, msg):
        position = ordered(msg, self.names, 'position')
        if position is None or msg.header.stamp.is_zero():
            return
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            if self.previous and stamp <= self.previous[0]:
                return
            velocity = ordered(msg, self.names, 'velocity')
            source = 1.
            if velocity is None:
                source = 0.
                if not self.previous or stamp-self.previous[0] > .15:
                    self.previous = stamp, position
                    return
                velocity = [(q-old)/(stamp-self.previous[0]) for q, old in zip(position, self.previous[1])]
            self.previous = stamp, position
            if self.active() and stamp >= self.run_start:
                self.pending.append((time.monotonic(), stamp, position, velocity, source))
                self.drain()

    def drain(self):
        while self.pending:
            arrived, stamp, measured, velocity, source = self.pending[0]
            target = self.history.at(stamp)
            if target is None or self.backend is None:
                if time.monotonic()-arrived < .15:
                    return
                self.pending.popleft()
                self.dropped += 1
                self.status.publish(String('关节目标缺失或时间无法对齐，已跳过 %d 帧；不会用实测值冒充目标' % self.dropped))
                continue
            self.pending.popleft()
            if self.run_start is None or stamp < self.run_start:
                continue
            reference, desired_velocity, gap = target
            error = [desired-actual for desired, actual in zip(reference, measured)]
            # time, target[6], actual[6], error[6], target_velocity[6], actual_velocity[6], interpolation_gap, backend, velocity_source
            values = [stamp-self.run_start]+reference+measured+error+desired_velocity+velocity+[gap, self.backend, source]
            self.samples.publish(Float64MultiArray(data=values))

    def diagnostics(self, msg):
        if len(msg.data) != 14 or not all(math.isfinite(v) for v in msg.data):
            return
        with self.lock:
            self.backend = msg.data[13]
            if self.active() and msg.data[0] >= self.run_start and (self.diagnostic_stamp is None or msg.data[0] > self.diagnostic_stamp):
                self.diagnostic_stamp = msg.data[0]
                self.servo.publish(Float64MultiArray(data=[msg.data[0]-self.run_start]+list(msg.data[1:])))
            self.drain()

    def flush_loop(self):
        while not rospy.is_shutdown():
            with self.lock:
                self.drain()
            time.sleep(.02)


if __name__ == '__main__':
    rospy.init_node('joint_tracking_recorder')
    JointRecorder()
    rospy.spin()
