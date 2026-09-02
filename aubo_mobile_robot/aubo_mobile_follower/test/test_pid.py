#!/usr/bin/env python3

import unittest

from aubo_mobile_follower.pid import FilteredPid


class FilteredPidTest(unittest.TestCase):
    def test_proportional_output_is_limited(self):
        pid = FilteredPid(kp=2.0)
        self.assertEqual(pid.update(0.1, 1.0, -1.0, 1.0), 0.2)
        self.assertEqual(pid.update(2.0, 2.0, -1.0, 1.0), 1.0)

    def test_repeated_measurement_timestamp_is_not_integrated_twice(self):
        pid = FilteredPid(ki=1.0, integral_limit=10.0)
        pid.update(1.0, 1.0, -10.0, 10.0)
        first = pid.update(1.0, 2.0, -10.0, 10.0)
        repeated = pid.update(1.0, 2.0, -10.0, 10.0)
        self.assertEqual(first, 1.0)
        self.assertEqual(repeated, first)

    def test_derivative_is_low_pass_filtered(self):
        pid = FilteredPid(kd=1.0, derivative_filter_alpha=0.25)
        pid.update(0.0, 1.0, -10.0, 10.0)
        self.assertEqual(pid.update(4.0, 2.0, -10.0, 10.0), 1.0)

    def test_integral_does_not_wind_up_into_saturation(self):
        pid = FilteredPid(ki=1.0, integral_limit=10.0)
        pid.update(2.0, 1.0, -1.0, 1.0)
        pid.update(2.0, 2.0, -1.0, 1.0)
        self.assertEqual(pid.integral, 0.0)

    def test_reset_clears_all_state(self):
        pid = FilteredPid(ki=1.0, kd=1.0)
        pid.update(1.0, 1.0, -10.0, 10.0)
        pid.update(2.0, 2.0, -10.0, 10.0)
        pid.reset()
        self.assertEqual(pid.update(2.0, 3.0, -10.0, 10.0), 0.0)

    def test_disabled_integral_does_not_accumulate_before_runtime_enable(self):
        pid = FilteredPid(kp=1.0, ki=0.0, integral_limit=10.0)
        pid.update(1.0, 1.0, -10.0, 10.0)
        pid.update(1.0, 2.0, -10.0, 10.0)
        pid.set_gains(1.0, 1.0, 0.0, 0.25, 10.0)
        self.assertEqual(pid.update(1.0, 3.0, -10.0, 10.0), 2.0)


if __name__ == "__main__":
    unittest.main()
