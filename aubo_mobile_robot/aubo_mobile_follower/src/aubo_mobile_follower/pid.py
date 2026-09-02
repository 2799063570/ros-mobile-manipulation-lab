#!/usr/bin/env python3
"""Small ROS-independent PID controller for follower nodes."""

from __future__ import division


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class FilteredPid(object):
    """PID with derivative filtering, integral limiting and anti-windup.

    ``timestamp`` belongs to the measurement, not the control-loop timer.  A
    repeated timestamp returns the previous output so one camera/lidar sample
    cannot be integrated more than once.
    """

    def __init__(
        self,
        kp=0.0,
        ki=0.0,
        kd=0.0,
        derivative_filter_alpha=0.25,
        integral_limit=1.0,
    ):
        self.set_gains(kp, ki, kd, derivative_filter_alpha, integral_limit)
        self.reset()

    def set_gains(self, kp, ki, kd, derivative_filter_alpha, integral_limit):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.derivative_filter_alpha = clamp(
            float(derivative_filter_alpha), 0.0, 1.0
        )
        self.integral_limit = max(0.0, float(integral_limit))
        if hasattr(self, "integral"):
            if self.ki == 0.0:
                self.integral = 0.0
            else:
                self.integral = clamp(
                    self.integral, -self.integral_limit, self.integral_limit
                )

    def reset(self):
        self.integral = 0.0
        self.derivative = 0.0
        self._previous_error = None
        self._previous_time = None
        self._last_output = 0.0

    def update(self, error, timestamp, lower, upper):
        error = float(error)
        timestamp = float(timestamp)
        lower = float(lower)
        upper = float(upper)
        if lower > upper:
            raise ValueError("PID lower output limit must not exceed upper limit")

        if self._previous_time is not None and timestamp <= self._previous_time:
            return self._last_output

        delta = None
        if self._previous_time is not None:
            delta = timestamp - self._previous_time

        if delta is not None and delta > 1.0e-6:
            raw_derivative = (error - self._previous_error) / delta
            alpha = self.derivative_filter_alpha
            self.derivative = alpha * raw_derivative + (1.0 - alpha) * self.derivative

            candidate_integral = self.integral
            if self.ki != 0.0:
                candidate_integral = clamp(
                    self.integral + error * delta,
                    -self.integral_limit,
                    self.integral_limit,
                )
            candidate_output = (
                self.kp * error
                + self.ki * candidate_integral
                + self.kd * self.derivative
            )
            # Conditional integration: do not wind up farther into saturation.
            if (
                lower <= candidate_output <= upper
                or (candidate_output > upper and error < 0.0)
                or (candidate_output < lower and error > 0.0)
            ):
                self.integral = candidate_integral

        output = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        self._last_output = clamp(output, lower, upper)
        self._previous_error = error
        self._previous_time = timestamp
        return self._last_output
