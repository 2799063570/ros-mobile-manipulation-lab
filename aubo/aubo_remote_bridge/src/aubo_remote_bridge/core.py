"""Transport-independent polling/freshness logic; deliberately has no motion API."""
import math
import secrets
import struct
import threading
from .protocol import Frame, POLL, JOINT_NAMES, SOURCE_NONE, SOURCE_ROS, decode_input


class BridgeCore:
    def __init__(self, period=0.02, timeout=0.1, session=None):
        if not 0 < period <= timeout <= 0.5:
            raise ValueError('require 0 < poll period <= reply timeout <= 0.5 seconds')
        self.period, self.timeout = period, timeout
        self.session = secrets.randbits(32) if session is None else session
        self.sequence = 0
        self.pending = None
        self.next_poll = 0.0
        self.last_time = None
        self.last_input = None
        self.rejected = 0

    def request(self, now):
        if self.pending is not None:
            if now - self.pending[1] < self.timeout:
                return None
            self.pending = None
            self.last_input = None
            self.last_time = None
        if now < self.next_poll:
            return None
        self.sequence += 1
        if self.sequence > 65535:
            self.sequence = 1
            self.session = (self.session + 1) & 0xffffffff
            self.last_input = None
            self.last_time = None
        self.pending = (self.sequence, now)
        self.next_poll = now + self.period
        return Frame(POLL, self.sequence, struct.pack('<I', self.session))

    def receive(self, frame, now):
        if (self.pending is None or frame.sequence != self.pending[0]
                or not 0 <= now - self.pending[1] < self.timeout):
            self.rejected += 1
            return None
        try:
            state = decode_input(frame, self.session)
        except ValueError:
            self.rejected += 1
            return None
        self.pending = None
        self.last_input, self.last_time = state, now
        return state

    def online(self, now):
        return self.last_time is not None and 0 <= now - self.last_time < self.timeout


class JointFeedback:
    def __init__(self, max_age=0.5):
        self.max_age = max_age
        self.lock = threading.Lock()
        self.received = None
        self.mask = 0
        self.joints = (0,) * 6

    def update(self, names, positions, now):
        # Each message replaces the complete sample; omitted joints never inherit old values.
        joints, mask = [0] * 6, 0
        valid = len(names) == len(positions) and len(set(names)) == len(names)
        if valid:
            values = dict(zip(names, positions))
            for index, name in enumerate(JOINT_NAMES):
                value = values.get(name)
                if value is not None and math.isfinite(value):
                    scaled = value * 1000
                    if math.isfinite(scaled) and -32768 <= scaled <= 32767:
                        mrad = round(scaled)
                        joints[index] = mrad
                        mask |= 1 << index
        with self.lock:
            self.joints, self.mask, self.received = tuple(joints), mask, now

    def snapshot(self, now):
        with self.lock:
            if self.received is None or not 0 <= now - self.received <= self.max_age:
                return SOURCE_NONE, 0, (0,) * 6
            return (SOURCE_ROS if self.mask else SOURCE_NONE), self.mask, self.joints
