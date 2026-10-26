#!/usr/bin/env python3
import os
import sys
import threading
import time
import unittest
from collections import deque
from unittest.mock import Mock
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from joint_tracking_recorder import TargetHistory, JointRecorder, ordered


class JointTrackingTest(unittest.TestCase):
    def test_history_and_order(self):
        history = TargetHistory()
        history.add(1., [0.]*6, [1.]*6)
        history.add(1.04, [.04]*6, [1.]*6)
        target = history.at(1.02)
        self.assertAlmostEqual(target[0][0], .02)
        self.assertIsNone(history.at(.99))
        self.assertIsNone(history.at(1.05))
        history.add(2., [1.]*6, [1.]*6)
        self.assertIsNone(history.at(1.5), 'Never interpolate across missing target data')
        msg = JointState(name=['b', 'a'], position=[2., 1.])
        self.assertEqual(ordered(msg, ['a', 'b'], 'position'), [1., 2.])
        msg.name = ['a', 'a']
        self.assertIsNone(ordered(msg, ['a', 'b'], 'position'))

    def test_measurement_join_and_missing_targets(self):
        recorder = JointRecorder.__new__(JointRecorder)
        recorder.lock = threading.RLock()
        recorder.names = ['j'+str(i) for i in range(6)]
        recorder.history = TargetHistory()
        recorder.pending = deque()
        recorder.previous = None
        recorder.run_start = 1000.
        recorder.stop_deadline = None
        recorder.backend = 0.
        recorder.dropped = 0
        recorder.samples, recorder.servo, recorder.status = Mock(), Mock(), Mock()
        recorder.history.add(1000., [1.]*6, [.1]*6)
        recorder.history.add(1000.04, [2.]*6, [.1]*6)
        message = JointState(header=Header(stamp=rospy.Time.from_sec(1000.02)), name=recorder.names,
                             position=[1.4]*6, velocity=[.3]*6)
        recorder.feedback(message)
        row = recorder.samples.publish.call_args[0][0].data
        self.assertEqual(len(row), 34)
        self.assertAlmostEqual(row[0], .02)
        for j in range(6):
            self.assertAlmostEqual(row[1+j], 1.5)
            self.assertAlmostEqual(row[7+j], 1.4)
            self.assertAlmostEqual(row[13+j], .1)
        self.assertEqual(row[32:], [0., 1.])
        count = recorder.samples.publish.call_count
        recorder.pending.append((time.monotonic()-.3, 1001., [1.]*6, [0.]*6, 1.))
        recorder.drain()
        self.assertEqual(recorder.dropped, 1)
        self.assertEqual(recorder.samples.publish.call_count, count, 'Missing reference must not be replaced by measurement')
        recorder.start(Header())
        self.assertIsNone(recorder.run_start)
        self.assertFalse(recorder.samples.publish.call_args[0][0].data)


if __name__ == '__main__':
    unittest.main()
