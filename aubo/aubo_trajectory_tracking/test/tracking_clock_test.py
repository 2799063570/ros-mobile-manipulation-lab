#!/usr/bin/env python3
"""Slow simulated clock must slow phase progression, never the wall watchdog."""
import threading
import time
import unittest
import rospy
import rostest
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float64MultiArray, Header
from std_srvs.srv import Trigger, SetBool, SetBoolResponse


class ClockTest(unittest.TestCase):
    def test_slow_clock_and_pause(self):
        self.enabled = False
        self.paused = False
        self.done = False
        self.state = ''
        self.elapsed = 0.
        self.position = [0., 0., .3]
        self.velocity = [0., 0., 0.]
        clock = rospy.Publisher('/clock', Clock, queue_size=1)
        joints = rospy.Publisher('/test/joints', JointState, queue_size=1)
        state = rospy.Publisher('/test/jog/state', String, queue_size=1)
        def enable(req):
            self.enabled = req.data
            return SetBoolResponse(True, 'test')
        service = rospy.Service('/test/jog/set_enabled', SetBool, enable)
        def command(msg):
            self.velocity = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        def status(msg):
            self.state = msg.data
        def metrics(msg):
            self.elapsed = msg.data[0]
        subs = [rospy.Subscriber('/test/jog/command', TwistStamped, command),
                rospy.Subscriber('/trajectory_tracking/state', String, status),
                rospy.Subscriber('/trajectory_tracking/metrics', Float64MultiArray, metrics)]
        def plant():
            simulated = 10.
            last = time.monotonic()
            while not self.done and not rospy.is_shutdown():
                now = time.monotonic()
                dt = (now-last)*.5 if not self.paused else 0.
                last = now
                simulated += dt
                if self.enabled:
                    self.position = [p+v*dt for p, v in zip(self.position, self.velocity)]
                stamp = rospy.Time.from_sec(simulated)
                clock.publish(Clock(stamp))
                joints.publish(JointState(header=Header(stamp=stamp), name=['x', 'y', 'z'], position=self.position))
                state.publish(String('ENABLED_IDLE' if self.enabled else 'DISABLED'))
                time.sleep(.01)
        worker = threading.Thread(target=plant, daemon=True)
        worker.start()
        def wait(predicate, timeout=5):
            until = time.monotonic()+timeout
            while time.monotonic()<until:
                if predicate(): return
                time.sleep(.01)
            self.fail('Timeout: '+self.state)
        try:
            rospy.wait_for_service('/trajectory_tracking/preview', 5)
            time.sleep(.4)
            response = rospy.ServiceProxy('/trajectory_tracking/preview', Trigger)()
            self.assertTrue(response.success, response.message)
            response = rospy.ServiceProxy('/trajectory_tracking/start', Trigger)()
            self.assertTrue(response.success, response.message)
            wait(lambda: self.state.startswith('RUNNING'))
            began = time.monotonic()
            wait(lambda: self.elapsed >= 1., 4)
            self.assertGreater(time.monotonic()-began, 1.7, 'Phase must use simulation time')
            self.assertLess(time.monotonic()-began, 2.5)
            before = self.elapsed
            self.paused = True
            wait(lambda: self.state.startswith('FAULT'), 1.)
            self.assertFalse(self.enabled, 'Wall watchdog must disable during a clock pause')
            self.assertLess(self.elapsed-before, .1)
            self.paused = False
            time.sleep(.2)
            self.assertTrue(self.state.startswith('FAULT'), 'Clock resume must not rearm')
        finally:
            self.done = True
            worker.join(1)
            service.shutdown()
            for sub in subs: sub.unregister()


if __name__ == '__main__':
    rospy.init_node('tracking_clock_test')
    rostest.rosrun('aubo_trajectory_tracking', 'tracking_clock', ClockTest)
