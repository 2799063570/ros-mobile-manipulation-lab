#!/usr/bin/env python3
"""ROS-independent state machine used by the semantic drive supervisor."""

from __future__ import division

import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DetectionGate(object):
    """Require the same semantic label in consecutive detector messages."""

    def __init__(self, confirmation_frames):
        self.confirmation_frames = max(1, int(confirmation_frames))
        self._label = None
        self._count = 0
        self._blocked_label = None

    def update(self, label):
        if not label:
            self.reset()
            return None
        if label == self._blocked_label:
            self._label = None
            self._count = 0
            return None
        self._blocked_label = None
        if label == self._label:
            self._count += 1
        else:
            self._label = label
            self._count = 1
        if self._count < self.confirmation_frames:
            return None
        confirmed = self._label
        self._label = None
        self._count = 0
        self._blocked_label = confirmed
        return confirmed

    def reset(self):
        self._label = None
        self._count = 0
        self._blocked_label = None


class SemanticDriveCore(object):
    """Apply stop, speed-limit and closed-loop turn actions to a nominal command."""

    FOLLOW_LINE = "FOLLOW_LINE"
    SLOW_FOLLOW = "SLOW_FOLLOW"
    STOPPED = "STOPPED"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"

    def __init__(self, config):
        self.stop_hold_time = max(0.0, float(config.get("stop_hold_time", 3.0)))
        self.slow_scale = clamp(float(config.get("slow_scale", 0.35)), 0.0, 1.0)
        self.slow_duration = max(0.0, float(config.get("slow_duration", 4.0)))
        self.turn_angle = abs(float(config.get("turn_angle", math.pi / 2.0)))
        self.turn_kp = float(config.get("turn_kp", 1.4))
        self.turn_min_speed = abs(float(config.get("turn_min_speed", 0.12)))
        self.turn_max_speed = abs(float(config.get("turn_max_speed", 0.45)))
        self.turn_min_speed = min(self.turn_min_speed, self.turn_max_speed)
        self.turn_tolerance = abs(float(config.get("turn_tolerance", 0.05236)))
        self.turn_settle_time = max(0.0, float(config.get("turn_settle_time", 0.25)))
        self.cooldown = max(0.0, float(config.get("cooldown", 4.0)))

        self.state = self.FOLLOW_LINE
        self._stop_until = 0.0
        self._slow_until = 0.0
        self._turn_target = None
        self._turn_settled_since = None
        self._cooldowns = {}

    @property
    def turning(self):
        return self.state in (self.TURN_LEFT, self.TURN_RIGHT)

    def cancel_motion_action(self):
        self.state = self.FOLLOW_LINE
        self._stop_until = 0.0
        self._turn_target = None
        self._turn_settled_since = None

    def trigger(self, action, now, current_yaw=None):
        action = str(action).strip().lower()
        if now < self._cooldowns.get(action, 0.0):
            return False
        if self.state == self.STOPPED and action in ("turn_left", "turn_right"):
            return False
        if self.turning and action in ("turn_left", "turn_right"):
            return False

        if action == "stop":
            self.state = self.STOPPED
            self._stop_until = now + self.stop_hold_time
            self._turn_target = None
            self._turn_settled_since = None
        elif action == "slow_down":
            self._slow_until = max(self._slow_until, now + self.slow_duration)
            if not self.turning and self.state != self.STOPPED:
                self.state = self.SLOW_FOLLOW
        elif action == "resume":
            self._slow_until = 0.0
            if not self.turning and self.state != self.STOPPED:
                self.state = self.FOLLOW_LINE
        elif action in ("turn_left", "turn_right"):
            if current_yaw is None:
                return False
            direction = 1.0 if action == "turn_left" else -1.0
            self._turn_target = normalize_angle(current_yaw + direction * self.turn_angle)
            self._turn_settled_since = None
            self.state = self.TURN_LEFT if direction > 0.0 else self.TURN_RIGHT
        else:
            return False

        self._cooldowns[action] = now + self.cooldown
        return True

    def command(self, linear_x, angular_z, now, current_yaw=None):
        if self.state == self.STOPPED:
            if now < self._stop_until:
                return 0.0, 0.0, self.STOPPED
            self.state = self.SLOW_FOLLOW if now < self._slow_until else self.FOLLOW_LINE

        if self.turning:
            if current_yaw is None or self._turn_target is None:
                return 0.0, 0.0, "ODOM_REQUIRED"
            error = normalize_angle(self._turn_target - current_yaw)
            if abs(error) <= self.turn_tolerance:
                if self._turn_settled_since is None:
                    self._turn_settled_since = now
                if now - self._turn_settled_since >= self.turn_settle_time:
                    self._turn_target = None
                    self._turn_settled_since = None
                    self.state = (
                        self.SLOW_FOLLOW if now < self._slow_until else self.FOLLOW_LINE
                    )
                    return 0.0, 0.0, self.state
                return 0.0, 0.0, self.state

            self._turn_settled_since = None
            speed = clamp(
                self.turn_kp * error, -self.turn_max_speed, self.turn_max_speed
            )
            if abs(speed) < self.turn_min_speed:
                speed = math.copysign(self.turn_min_speed, error)
            return 0.0, speed, self.state

        if now < self._slow_until:
            self.state = self.SLOW_FOLLOW
            return linear_x * self.slow_scale, angular_z * self.slow_scale, self.state

        self.state = self.FOLLOW_LINE
        return linear_x, angular_z, self.state
