#!/usr/bin/env python3
"""The plain eye-in-hand controller reaches its viewpoint before tracking."""
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool


class InitialObservationTest(unittest.TestCase):
    def test_visible_target_does_not_skip_observation(self):
        lock = threading.Lock()
        joints = [0.0] * 6
        states = []
        names = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
        feedback_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        target_pub = rospy.Publisher('/visual_servo/target_pose', PoseStamped,
                                     queue_size=1)

        def output(message, index):
            with lock:
                joints[index] = message.data

        subscribers = [rospy.Subscriber('/initial_observation/q%d' % i,
                                        Float64, output, callback_args=i)
                       for i in range(6)]
        subscribers.append(rospy.Subscriber('/visual_servo/state', String,
                                            lambda message: states.append(message.data)))

        def tick(_event):
            with lock:
                positions = list(joints)
            feedback = JointState()
            feedback.header.stamp = rospy.Time.now()
            feedback.name, feedback.position = names, positions
            feedback_pub.publish(feedback)
            target = PoseStamped()
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = 'tcp_link'
            target.pose.position.x = 0.3
            target.pose.position.z = 0.3
            target.pose.orientation.w = 1.0
            target_pub.publish(target)

        timer = rospy.Timer(rospy.Duration(0.01), tick)
        try:
            rospy.wait_for_service('/visual_servo/set_enabled', timeout=10)
            time.sleep(0.2)
            self.assertTrue(rospy.ServiceProxy('/visual_servo/set_enabled', SetBool)(True).success)
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and 'TRACKING' not in states:
                with lock:
                    position = joints[0]
                if position < 0.09:
                    self.assertNotIn('TRACKING', states)
                time.sleep(0.02)
            self.assertIn('SEARCH_INITIAL', states)
            self.assertIn('TRACKING', states)
            with lock:
                self.assertGreater(joints[0], 0.09)
        finally:
            timer.shutdown()
            for subscriber in subscribers:
                subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('initial_observation_test')
    rostest.rosrun('aubo_ros_control', 'initial_observation_test', InitialObservationTest)
