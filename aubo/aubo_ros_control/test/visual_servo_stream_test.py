#!/usr/bin/env python3
"""Production PBVS frontend with a delayed, local SDK boundary and 5ms cabinet."""
import collections
import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger


class StreamTest(unittest.TestCase):
    def test_target_reversal_loss_disable_and_feedback_fault(self):
        lock = threading.RLock()
        names = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
        joints = [0.0]*6
        pending = collections.deque()
        points, states = [], []
        control = dict(goal=.20, targets=True, feedback=True, align=False, drifted=False)
        mode = rospy.get_param('~mode')
        feedback_pub = rospy.Publisher('/jog_ros_control/raw_joint_states', JointState, queue_size=2)
        target_pub = rospy.Publisher('/visual_servo/target_pose', PoseStamped, queue_size=1)

        def batch(message):
            with lock:
                tail = max(time.monotonic(), pending[-1][0] if pending else 0)
                for begin in range(0, len(message.data), 6):
                    point = tuple(message.data[begin:begin+6])
                    self.assertEqual(6, len(point))
                    points.append(point)
                    tail += .005
                    pending.append((tail, point))

        batch_sub = rospy.Subscriber('/jog_sdk/point_batches', Float64MultiArray, batch)
        state_sub = rospy.Subscriber('/visual_servo/state', String,
                                     lambda message: states.append(message.data))

        def tick(_event):
            with lock:
                if 'WAITING_FOR_SDK' in states and not points and not control['drifted']:
                    joints[0] = 1e-5  # Encoder drift after SDK captured its held reference.
                    control['drifted'] = True
                while pending and pending[0][0] <= time.monotonic():
                    joints[:] = pending.popleft()[1]
                current = list(joints)
                goal, send_target, send_feedback = control['goal'], control['targets'], control['feedback']
                if control['align']:
                    goal = sum(current)
            if send_feedback:
                feedback = JointState()
                feedback.header.stamp = rospy.Time.now()
                feedback.name, feedback.position = names, current
                feedback_pub.publish(feedback)
            if send_target:
                target = PoseStamped()
                target.header.stamp = rospy.Time.now()
                target.header.frame_id = 'tcp_link' if mode == 'eye_in_hand' else 'base_link'
                target.pose.position.x = goal - sum(current) if mode == 'eye_in_hand' else goal
                target.pose.position.z = .3 if mode == 'eye_in_hand' else 0
                target.pose.orientation.w = 1
                target_pub.publish(target)

        timer = rospy.Timer(rospy.Duration(.01), tick)
        enable = rospy.ServiceProxy('/visual_servo/set_enabled', SetBool)
        reset = rospy.ServiceProxy('/visual_servo/reset', Trigger)
        try:
            rospy.wait_for_service('/visual_servo/set_enabled', timeout=8)
            # A denied enable is not queued: the disabled startup must send no points.
            time.sleep(.15)
            self.assertEqual([], points)
            deadline = time.monotonic()+4
            enabled = False
            while time.monotonic() < deadline:
                if enable(True).success:
                    enabled = True
                    break
                time.sleep(.05)
            self.assertTrue(enabled, 'SDK readiness never recovered')
            self.assertTrue(control['drifted'])
            time.sleep(1.3)
            self.assertIn('TRACKING', states)
            self.assertGreater(len(points), 100)
            self.assertGreater(sum(joints), .01)
            with lock:
                control['goal'] = -.15
                reversal_index = len(points)
            time.sleep(1.5)
            with lock:
                reverse_points = list(points[reversal_index:])
            self.assertTrue(any(b[0] < a[0]-1e-8 for a, b in zip(reverse_points, reverse_points[1:])))

            # Zero visual error while the old stream is moving: ALIGNED must
            # wait for smoothing/feedback to settle instead of resetting q/v.
            with lock:
                control['align'] = True
            deadline = time.monotonic()+4
            while time.monotonic() < deadline and states[-1] != 'ALIGNED':
                time.sleep(.02)
            self.assertEqual('ALIGNED', states[-1])
            # ALIGNED gates on generated state. The last received batch can
            # still predate that state when its SDK RPC takes 90ms. Observe a
            # subsequently returned sequence before differentiating it.
            deadline = time.monotonic()+.5
            residual = math.inf
            while time.monotonic() < deadline:
                with lock:
                    residual = max(abs(a-b)/.005 for a,b in zip(points[-1],points[-2]))
                if residual < 1.1e-6:
                    break
                time.sleep(.01)
            with lock:
                self.assertLess(residual, 1.1e-6)

            # Ordinary image loss follows the configured stop policy, without
            # jumping the integrated command back to delayed physical feedback.
            with lock:
                control['targets'] = False
            time.sleep(2.0)
            self.assertEqual('HOLD', states[-1])
            with lock:
                self.assertLess(max(abs(points[-1][i]-points[-30][i]) for i in range(6)), 1e-5)
                control['goal'], control['targets'], control['align'] = .25, True, False
            time.sleep(.7)
            self.assertTrue(enable(False).success)
            time.sleep(2.0)
            self.assertIn('STOPPING', states)
            self.assertEqual('DISABLED', states[-1])
            self.assertTrue(reset().success)
            with lock:
                frozen = points[-1]
            time.sleep(.3)
            with lock:
                self.assertLess(max(abs(a-b) for a,b in zip(frozen,points[-1])), 1e-5)

            self.assertTrue(enable(True).success)
            time.sleep(.4)
            with lock:
                control['feedback'] = False
            time.sleep(.4)
            self.assertIn('HOLD', states[-10:])
            self.assertFalse(enable(True).success)
            with lock:
                control['feedback'] = True
            self.assertTrue(enable(False).success)
            time.sleep(2.0)
            self.assertTrue(reset().success)

            with lock:
                recorded = list(points)
            previous, old_v, old_a = (0.0,)*6, [0.0]*6, [0.0]*6
            for point in recorded:
                for i in range(6):
                    self.assertTrue(math.isfinite(point[i]))
                    velocity = (point[i]-previous[i])/.005
                    acceleration = (velocity-old_v[i])/.005
                    jerk = (acceleration-old_a[i])/.005
                    self.assertLessEqual(abs(velocity), (.5 if i<3 else .7)+1e-7)
                    self.assertLessEqual(abs(acceleration), (.8 if i<3 else 1.2)+1e-6)
                    self.assertLessEqual(abs(jerk), (8 if i<3 else 12)+1e-4)
                    old_v[i], old_a[i] = velocity, acceleration
                previous = point
        finally:
            timer.shutdown()
            batch_sub.unregister()
            state_sub.unregister()


if __name__ == '__main__':
    rospy.init_node('visual_servo_stream_test')
    rostest.rosrun('aubo_ros_control', 'visual_servo_stream', StreamTest)
