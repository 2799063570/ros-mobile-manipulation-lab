#!/usr/bin/env python3
"""Reject unsafe configuration before the fake SDK is even connected."""
import copy
import os
import signal
import subprocess
import tempfile
import unittest
import time

import rospkg
import rospy
import rostest
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class StartupTest(unittest.TestCase):
    def test_invalid_order_and_model_limits(self):
        if rospy.get_param('~real_model', False):
            self.skipTest('separate real-model smoke test')
        package = rospkg.RosPack().get_path('aubo_ros_control')
        with open(os.path.join(package, 'config/visual_servo_common.yaml')) as source:
            baseline = yaml.safe_load(source)
        baseline.update(backend='sdk', servo_mode='eye_to_hand', collision_check=False,
                        desired_target_position=[0., 0., 0.], desired_target_rpy=[0., 0., 0.])
        reordered = list(baseline['joint_names'])
        reordered[0], reordered[1] = reordered[1], reordered[0]
        cases = [
            ({'joint_names': reordered}, {}, 'joint_names'),
            ({}, {'shoulder_joint': {'has_velocity_limits': True, 'max_velocity': .2}}, 'shoulder_joint'),
            ({}, {'shoulder_joint': {'has_acceleration_limits': True, 'max_acceleration': .2}}, 'shoulder_joint'),
            ({'joint_lower_limits': [-4.] * 6}, {}, 'shoulder_joint'),
        ]
        for overrides, model_limits, expected in cases:
            with self.subTest(overrides=overrides, model_limits=model_limits):
                params = copy.deepcopy(baseline)
                params.update(overrides)
                rospy.set_param('/validation_case', params)
                rospy.set_param('/robot_description_planning/joint_limits', model_limits)
                with tempfile.TemporaryFile(mode='w+') as log:
                    process = subprocess.Popen(
                        ['rosrun', 'aubo_ros_control', 'aubo_visual_servo_fake_sdk_node',
                         '__name:=validation_case'], stdout=log, stderr=subprocess.STDOUT)
                    try:
                        code = process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                        self.fail('unsafe configuration was not rejected during startup')
                    log.seek(0)
                    output = log.read()
                self.assertEqual(code, 1, output)
                self.assertIn(expected, output)
                self.assertNotIn('[fake_sdk]', output)

    def test_real_model_holds_feedback_and_accepts_explicit_enable(self):
        if not rospy.get_param('~real_model', False):
            self.skipTest('synthetic startup rejection test')
        package = rospkg.RosPack().get_path('aubo_ros_control')
        with open(os.path.join(package, 'config/visual_servo_common.yaml')) as source:
            params = yaml.safe_load(source)
        params.update(backend='gazebo', servo_mode='eye_to_hand', collision_check=True,
                      initial_search_enabled=False, desired_target_position=[0., 0., 0.],
                      desired_target_rpy=[0., 0., 0.], gazebo_command_topics=[
                          '/validation_case/q%d' % i for i in range(6)])
        rospy.set_param('/validation_case', params)
        home = [0., 0., 1.6292, 0., 1.6628, 0.]
        received = [[] for _ in range(6)]
        subscribers = [rospy.Subscriber('/validation_case/q%d' % i, Float64,
                                        lambda msg, i: received[i].append(msg.data), callback_args=i)
                       for i in range(6)]
        feedback = JointState()
        feedback.name = params['joint_names'] + ['joint1', 'joint2']
        feedback.position = home + [0., 0.]
        publisher = rospy.Publisher('/joint_states', JointState, queue_size=1)

        def tick(_event):
            feedback.header.stamp = rospy.Time.now()
            publisher.publish(feedback)

        timer = rospy.Timer(rospy.Duration(.01), tick)
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(
                ['rosrun', 'aubo_ros_control', 'aubo_visual_servo_node', '__name:=validation_case'],
                stdout=log, stderr=subprocess.STDOUT)
            try:
                rospy.wait_for_service('/visual_servo/set_enabled', timeout=10)
                enable = rospy.ServiceProxy('/visual_servo/set_enabled', SetBool)
                deadline = time.monotonic() + 5
                while not enable(True).success:
                    self.assertLess(time.monotonic(), deadline, 'real model never became ready')
                    time.sleep(.05)
                time.sleep(.3)
                self.assertTrue(enable(False).success)
                process.send_signal(signal.SIGINT)
                self.assertEqual(process.wait(timeout=6), 0)
                for i, values in enumerate(received):
                    self.assertGreater(len(values), 10)
                    self.assertLess(max(abs(value-home[i]) for value in values), 1e-8)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                timer.shutdown()
                for subscriber in subscribers:
                    subscriber.unregister()
                log.seek(0)
                rospy.loginfo(log.read())


if __name__ == '__main__':
    rospy.init_node('visual_servo_startup_test')
    rostest.rosrun('aubo_ros_control', 'visual_servo_startup', StartupTest)
