#!/usr/bin/env python3
import math
import time
import threading
import unittest
import rospy
import rostest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, SetBool, SetBoolResponse


class TrackingTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.enabled = False
        self.position = [0., 0., .3]
        self.velocity = [0., 0., 0.]
        self.plant_velocity = [0., 0., 0.]
        self.plant_gain = 1.
        self.plant_lag = 0.
        self.commands = []
        self.state = ''
        self.readiness = ''
        self.tracking_samples = []
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
                     rospy.Subscriber('/trajectory_tracking/reference_path', Path, self.path),
                     rospy.Subscriber('/trajectory_tracking/readiness', String, self.ready_status),
                     rospy.Subscriber('/trajectory_tracking/samples', Float64MultiArray, self.sample)]
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

    def sample(self, msg):
        if not msg.data:
            self.tracking_samples = []
        else:
            self.tracking_samples.append(list(msg.data))

    def ready_status(self, msg):
        self.readiness = msg.data

    def path(self, msg):
        self.reference = msg

    def plant(self, event):
        with self.lock:
            now = time.monotonic()
            dt = min(now-self.last, .03)
            self.last = now
            if self.enabled:
                alpha = 1. if self.plant_lag == 0 else 1.-math.exp(-dt/self.plant_lag)
                self.plant_velocity = [v+alpha*(self.plant_gain*cmd-v) for v, cmd in zip(self.plant_velocity, self.velocity)]
                self.position = [p+v*dt for p, v in zip(self.position, self.plant_velocity)]
            else:
                self.plant_velocity = [0., 0., 0.]
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
        self.wait(lambda: self.readiness.startswith('PREVIEW_READY:'))
        initial = self.start()
        self.assertFalse(initial.success, 'Must preview before enabling')
        self.assertIn('更新预览', initial.message)
        self.wait(lambda: self.readiness.startswith('PREVIEW_READY:'))
        self.assertFalse(self.enabled)
        self.configure()
        self.feedback_on = False
        time.sleep(.2)
        rejected = self.preview()
        self.assertFalse(rejected.success)
        self.assertIn('反馈', rejected.message)
        self.wait(lambda: self.readiness.startswith('WAITING:'))
        self.assertFalse(self.enabled, 'Readiness checks must not enable motion')
        self.feedback_on = True
        self.wait(lambda: self.readiness.startswith('PREVIEW_READY:'))
        self.state_on = False
        time.sleep(.4)
        rejected = self.preview()
        self.assertFalse(rejected.success)
        self.assertIn('控制器状态超时', rejected.message)
        self.state_on = True
        self.wait(lambda: self.readiness.startswith('PREVIEW_READY:'))
        self.assertTrue(self.preview().success)
        self.wait(lambda: self.readiness.startswith('START_READY:'))
        with self.lock:
            self.position[0] += .001
        self.wait(lambda: self.readiness.startswith('PREVIEW_READY:'))
        rejected = self.start()
        self.assertFalse(rejected.success)
        self.assertIn('偏离预览起点', rejected.message)
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
            self.assertGreater(len(self.tracking_samples), 20)
            self.assertTrue(all(len(row) == 16 for row in self.tracking_samples))
            for row in self.tracking_samples:
                u = max(0., min(1., row[0]/3.))
                phase = u**3*(10.+u*(-15.+6.*u))
                a = .002*(math.cos(2*math.pi*phase)-1) if shape == 'circle' else .006*phase
                b = .002*math.sin(2*math.pi*phase)
                delta = {'xy': [a, b, 0.], 'xz': [a, 0., b], 'yz': [0., a, b]}[plane]
                for axis in range(3):
                    self.assertAlmostEqual(row[1+axis], origin[axis]+delta[axis], places=8,
                                           msg='Reference must match the physical feedback timestamp')
                self.assertAlmostEqual(row[7], math.sqrt(sum((row[1+i]-row[4+i])**2 for i in range(3))), places=8)
            self.assertTrue(all(a[0] <= b[0] for a, b in zip(self.tracking_samples, self.tracking_samples[1:])))
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

        self.plant_gain, self.plant_lag = .8, .08
        results = []
        for kp, ki, kd, ff in [(.5, 0., 0., 0.), (1., 0., 0., 0.), (1., 0., 0., .10), (1., .05, .1, .10)]:
            self.configure()
            for key, value in dict(radius=.03, duration=8., max_speed=.08, max_error=.04,
                                   kp=kp, ki=ki, kd=kd, acceleration_ff_time=ff, settle_tolerance=.0005).items():
                rospy.set_param('/trajectory_tracking/'+key, value)
            self.begin()
            self.wait(lambda: self.state.startswith('COMPLETED'), 15)
            self.assertTrue(all(math.sqrt(sum(v*v for v in command)) <= .080001 for _,command in self.commands))
            errors = [row[7] for row in self.tracking_samples if row[13] == 0]
            results.append((max(errors), math.sqrt(sum(e*e for e in errors)/len(errors))))
        rospy.loginfo('Lagged plant old P / bounded P / acceleration FF / optional PID peak,RMS (m): %s', results)
        self.assertLess(results[1][0], .85*results[0][0], 'Must reduce peak tracking error in lagged plant')
        self.assertLess(results[1][1], .9*results[0][1], 'Must reduce RMS tracking error in lagged plant')

        self.assertLess(results[2][0], results[1][0], 'Acceleration FF must reduce peak in lagged plant')
        self.assertLess(results[2][1], results[1][1], 'Acceleration FF must reduce RMS in lagged plant')


if __name__ == '__main__':
    rospy.init_node('tracking_test')
    rostest.rosrun('aubo_trajectory_tracking', 'tracking_integration', TrackingTest)
