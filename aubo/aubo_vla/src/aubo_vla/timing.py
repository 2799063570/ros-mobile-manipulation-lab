"""Monotonic liveness and simulation epochs, independent of rospy."""
import math


def positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be finite and positive')
    return value


class ClockGuard:
    def __init__(self, simulated, pause_timeout=2.0):
        self.simulated = simulated
        self.pause_timeout = positive(pause_timeout, 'pause_timeout')
        self.stamp = None
        self.wall = None
        self.epoch = 0

    def update(self, stamp, wall):
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError('Invalid clock stamp')
        changed = self.stamp is not None and (
            stamp < self.stamp or wall-self.wall > self.pause_timeout)
        if changed:
            self.epoch += 1
        if stamp != self.stamp:
            self.wall = wall
        self.stamp = stamp
        return changed

    def paused(self, wall):
        return self.simulated and (self.stamp is None or self.wall is None or
                                   wall-self.wall > self.pause_timeout)


def source_age(stamp, now, max_age, future_tolerance=0.05):
    if not math.isfinite(stamp) or stamp <= 0 or not math.isfinite(now):
        raise ValueError('Image must have a positive finite source timestamp')
    age = now-stamp
    if age < -future_tolerance or age > max_age:
        raise ValueError('Image source timestamp is future or stale')
    return age
