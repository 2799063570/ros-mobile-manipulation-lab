#!/usr/bin/env python3
"""跟随节点共用的 PID：独立于 ROS，便于离线验证控制逻辑。"""

from __future__ import division


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class FilteredPid(object):
    """带微分低通滤波、积分限幅和抗饱和的 PID。

    timestamp 必须取自传感器测量时刻，而非控制定时器时刻。
    重复或乱序帧不更新积分与微分，但输出仍须服从本次的速度上下限。
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
            # 动态调参或接近目标时可能收紧限速，旧测量不能绕过新的限制。
            self._last_output = clamp(self._last_output, lower, upper)
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
            # 输出饱和时，仅允许能使输出退出饱和区的误差继续积分。
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
