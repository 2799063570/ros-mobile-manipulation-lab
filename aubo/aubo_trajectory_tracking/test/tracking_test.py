#!/usr/bin/env python3
import math
import time
import threading
import unittest
import rospy
import rostest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, SetBool, SetBoolResponse


class TrackingTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.enabled = False
        self.position = [0., 0., .3]
        self.velocity = [0., 0., 0.]
        self.commands = []
        self.state = ''
        self.reference = None
        self.feedback_on = True
        self.state_on = True
        self.feedback_nan = False
        self.frozen_stamp = None
        self.joint_pub = rospy.Publisher('/test/joints', JointState, queue_size=1)
        self.state_pub = rospy.Publisher('/test/jog/state', String, queue_size=1)
        self.service = rospy.Service('/test/jog/set_enabled', SetBool, self.enable)
        self.subs = [rospy.Subscriber('/test/jog/command', TwistStamped, self.command),
                     rospy.Subscriber('/trajectory_tracking/state', String, self.status),
                     rospy.Subscriber('/trajectory_tracking/reference_path', Path, self.path)]
        self.last = time.monotonic()
        self.timer = rospy.Timer(rospy.Duration(.01), self.plant)
        self.preview = rospy.ServiceProxy('/trajectory_tracking/preview', Trigger)
        self.start = rospy.ServiceProxy('/trajectory_tracking/start', Trigger)
        self.stop = rospy.ServiceProxy('/trajectory_tracking/stop', Trigger)
        rospy.wait_for_service('/trajectory_tracking/preview', 10)
        self.wait(lambda: self.state.startswith('IDLE'), 5)

    def tearDown(self):
        self.stop()
        self.timer.shutdown()
        self.service.shutdown()
        for sub in self.subs:
            sub.unregister()

    def enable(self, req):
        with self.lock:
            self.enabled = req.data
            if req.data:
                self.commands = []
            else:
                self.velocity = [0., 0., 0.]
        return SetBoolResponse(True, 'fixture')

    def command(self, msg):
        with self.lock:
            self.velocity = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
            self.commands.append((time.monotonic(), list(self.velocity)))

    def status(self, msg):
        self.state = msg.data

    def path(self, msg):
        self.reference = msg

    def plant(self, event):
        with self.lock:
            now = time.monotonic()
            dt = min(now-self.last, .03)
            self.last = now
            if self.enabled:
                self.position = [p+v*dt for p, v in zip(self.position, self.velocity)]
            if self.feedback_on:
                msg = JointState()
                msg.header.stamp = self.frozen_stamp or rospy.Time.now()
                msg.name = ['x', 'y', 'z']
                msg.position = list(self.position)
                if self.feedback_nan:
                    msg.position[0] = float('nan')
                self.joint_pub.publish(msg)
            if self.state_on:
                self.state_pub.publish(String('ENABLED_IDLE' if self.enabled else 'DISABLED'))

    def wait(self, predicate, timeout=4):
        until = time.monotonic()+timeout
        while time.monotonic()<until and not rospy.is_shutdown():
            if predicate():
                return
            time.sleep(.01)
        self.fail('Timed out; tracker state: '+self.state)

    def configure(self, shape='circle', plane='xy'):
        params = dict(shape=shape, plane=plane, radius=.002, amplitude=.002, length=.006,
                      duration=3., kp=1., max_speed=.03, max_error=.02, settle_tolerance=.0003)
        for key, value in params.items():
            rospy.set_param('/trajectory_tracking/'+key, value)
        time.sleep(.15)

    def begin(self):
        res = self.preview()
        self.assertTrue(res.success, res.message)
        res = self.start()
        self.assertTrue(res.success, res.message)
        self.wait(lambda: self.state.startswith('RUNNING'))

    def test_tracking_and_guards(self):
        time.sleep(.2)
        self.assertFalse(self.start().success, 'Must preview before enabling')
        self.assertFalse(self.enabled)
        for shape, plane in [('circle', 'xy'), ('sine', 'xz'), ('circle', 'yz')]:
            self.configure(shape, plane)
            with self.lock:
                origin = list(self.position)
            self.begin()
            self.wait(lambda: self.reference is not None)
            self.assertEqual(len(self.reference.poses), 1001)
            first = self.reference.poses[0].pose.position
            self.assertAlmostEqual(first.x, origin[0], places=4)
            self.wait(lambda: self.state.startswith('COMPLETED'), 8)
            self.assertFalse(self.enabled)
            with self.lock:
                self.assertGreater(len(self.commands), 20)
                self.assertTrue(all(sum(v*v for v in values) < 1e-15 for _, values in self.commands[:5]), 'Neutral handshake required')
                self.assertLessEqual(max(math.sqrt(sum(v*v for v in values)) for _, values in self.commands), .030001)
                delta = [p-o for p, o in zip(self.position, origin)]
            expected = [0., 0., 0.] if shape == 'circle' else [.006, 0., 0.]
            self.assertLess(math.sqrt(sum((a-b)**2 for a, b in zip(delta, expected))), .0005)
            self.reference = None
        self.configure()
        rospy.set_param('/trajectory_tracking/radius', .4)
        self.assertFalse(self.preview().success)
        rospy.set_param('/trajectory_tracking/radius', float('nan'))
        self.assertFalse(self.preview().success)
        self.configure()
        rospy.set_param('/trajectory_tracking/duration', 2.)
        rospy.set_param('/trajectory_tracking/radius', .03)
        rospy.set_param('/trajectory_tracking/max_speed', .001)
        self.assertFalse(self.preview().success, 'Must reject excessive analytic feedforward speed')
        for failure in ['feedback', 'state', 'nan', 'stamp', 'error', 'stop']:
            self.configure()
            self.begin()
            if failure == 'feedback':
                self.feedback_on = False
            elif failure == 'state':
                self.state_on = False
            elif failure == 'nan':
                self.feedback_nan = True
            elif failure == 'stamp':
                self.frozen_stamp = rospy.Time.now()
            elif failure == 'error':
                with self.lock:
                    self.position[0] += .1
            else:
                self.assertTrue(self.stop().success)
            self.wait(lambda: self.state.startswith('STOPPED' if failure == 'stop' else 'FAULT'))
            self.assertFalse(self.enabled)
            self.assertFalse(self.start().success, 'Never automatically rearm after stop or fault')
            self.feedback_on = self.state_on = True
            self.feedback_nan = False
            self.frozen_stamp = None
            time.sleep(.2)


if __name__ == '__main__':
    rospy.init_node('tracking_test')
    rostest.rosrun('aubo_trajectory_tracking', 'tracking_integration', TrackingTest)
