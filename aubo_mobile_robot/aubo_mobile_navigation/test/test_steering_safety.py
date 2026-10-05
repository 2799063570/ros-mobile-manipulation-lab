#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch
from types import SimpleNamespace as S


with patch.dict(sys.modules, {k: MagicMock() for k in ['rospy','geometry_msgs','geometry_msgs.msg','sensor_msgs','sensor_msgs.msg','std_msgs','std_msgs.msg']}):
    spec=importlib.util.spec_from_file_location('safety',str(Path(__file__).resolve().parents[1] / 'scripts/laser_safety_filter.py'))
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class SteeringSafetyTest(unittest.TestCase):
    def setUp(self):
        self.node = m.LaserSafetyFilter.__new__(m.LaserSafetyFilter)
        values = dict(
            robot_half_width=.3,
            safety_margin=.12,
            stop_distance=.35,
            reverse_stop_distance=.35,
            reaction_time=.25,
            max_deceleration=.8,
            max_angular_deceleration=.7,
            rotation_clearance=.48,
            linear_deadband=.001,
            angular_deadband=.001,
            emergency_distance=.18,
        )
        for name, value in values.items():
            setattr(self.node, name, value)

    def check(self, x, y, linear, angular):
        scan = S(
            ranges=[math.hypot(x, y)] * 3,
            angle_min=math.atan2(y, x),
            angle_increment=0.,
            range_min=.01,
            range_max=10.,
        )
        command = S(linear=S(x=linear), angular=S(z=angular))
        return self.node._hazard_counts(scan, command)

    def test_swept_steering_clearance(self):
        self.assertEqual(self.check(.4, 0, .04, .03)[0], 0)
        self.assertEqual(self.check(.35, 0, .04, .03)[0], 3)
        self.assertEqual(self.check(0, .42, .04, .03)[0], 3)
        self.assertEqual(self.check(-.35, 0, -.04, .03)[0], 3)
        self.assertEqual(self.check(.4, 0, 0, .03)[0], 3)
        self.assertTrue(self.check(.17, 0, .04, .03)[1])


if __name__ == '__main__':
    unittest.main()
