#!/usr/bin/env python3
"""Exercise the compiled mission against fake services/actions on an isolated ROS master."""
import os
import subprocess
import threading
import time
import unittest

import tf
import actionlib
import roslib.packages
import rospy
import rostest
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


class CppMissionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('cpp_mission_test')

    def wait(self, predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        self.fail('timed out; states: ' + str(self.states))

    def setUp(self):
        self.states = []
        self.stops = 0
        self.homes = 0
        self.release = threading.Event()
        self.block_home = False
        self.block_stop = False
        self.ack_stop = True
        self.complete = True
        self.goals = []
        self.base_pose = [-1.2, .4, .6]
        self.return_abort = False
        self.return_hold = False
        self.navigation_goals = 0
        self.hold_navigation = False
        self.navigation_cancelled = threading.Event()
        self.state_pub = rospy.Publisher('/sorting/state', String, queue_size=10, latch=True)
        self.lock_pub = rospy.Publisher('/sorting/base_locked', Bool, queue_size=10, latch=True)
        self.sub = rospy.Subscriber('/nav_sorting/state', String,
                                    lambda msg: self.states.append(msg.data.split('|')[0].strip()))
        self.servers = [
            rospy.Service('/sorting/home', Trigger, self.home),
            rospy.Service('/sorting/prepare_work', Trigger, lambda _: self.operation('PREPARING')),
            rospy.Service('/sorting/move_to_observation', Trigger, lambda _: self.operation('OBSERVING')),
            rospy.Service('/sorting/start', Trigger, lambda _: self.operation('SORTING')),
            rospy.Service('/sorting/stop', Trigger, self.stop),
        ]
        self.navigation = actionlib.SimpleActionServer('/test_move_base', MoveBaseAction,
                                                       execute_cb=self.navigate, auto_start=False)
        self.navigation.start()
        rospy.set_param('/nav_sorting_mission', {
            'navigation_action': '/test_move_base',
            'auto_start': False,
            'return_to_start': self._testMethodName.startswith('test_return'),
            'return_frame': 'map',
            'sorting_initialization_timeout': 2.0,
            'sorting_operation_timeout': 10.0,
            'sorting_stop_timeout': .25,
            'server_timeout': .5,
            'navigation_timeout': 5.0,
            'navigation_retries': 0,
            'home_before_navigation': True,
            'near_field_enabled': False,
            'base_recovery_enabled': False,
        })
        self.broadcaster = tf.TransformBroadcaster()
        def publish_tf(_):
            if self._testMethodName == 'test_return_missing_pose_blocks_mission':
                return
            x, y, yaw = self.base_pose
            self.broadcaster.sendTransform((x, y, 0), tf.transformations.quaternion_from_euler(0, 0, yaw),
                                           rospy.Time.now(), 'base_footprint', 'map')
        self.tf_timer = rospy.Timer(rospy.Duration(.02), publish_tf)
        self.state_pub.publish(String(data='IDLE'))
        self.lock_pub.publish(Bool(data=False))
        executable = os.environ.get('NAV_MISSION_BINARY') or roslib.packages.find_node(
            'aubo_mobile_nav_sorting', 'nav_sorting_mission_cpp')[0]
        self.log = open('/tmp/nav_mission_test_{}.log'.format(self._testMethodName), 'w')
        self.process = subprocess.Popen([executable], stdout=self.log, stderr=subprocess.STDOUT)
        rospy.wait_for_service('/nav_sorting/start', timeout=5)
        self.wait(lambda: 'IDLE' in self.states)
        # Ensure latched readiness and base lock reach the mission before start.
        time.sleep(.1)
        self.start = rospy.ServiceProxy('/nav_sorting/start', Trigger)
        self.recover = rospy.ServiceProxy('/nav_sorting/recover_stop', Trigger)

    def tearDown(self):
        self.release.set()
        self.tf_timer.shutdown()
        self.process.terminate()
        try:
            self.process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.log.close()
        for service in self.servers:
            service.shutdown()
        self.navigation.action_server.status_timer.shutdown()
        self.navigation.action_server.goal_sub.unregister()
        self.navigation.action_server.cancel_sub.unregister()
        self.navigation.action_server.status_pub.unregister()
        self.navigation.action_server.result_pub.unregister()
        self.navigation.action_server.feedback_pub.unregister()
        self.sub.unregister()
        self.state_pub.unregister()
        self.lock_pub.unregister()

    def terminal(self, state='READY'):
        self.lock_pub.publish(Bool(data=False))
        self.state_pub.publish(String(data=state))

    def operation(self, state):
        self.lock_pub.publish(Bool(data=True))
        self.state_pub.publish(String(data=state))
        if self.complete:
            threading.Timer(.06, self.terminal).start()
        return TriggerResponse(success=True, message='accepted')

    def home(self, _):
        self.homes += 1
        if self.block_home:
            self.release.wait(30)
        return self.operation('HOMING')

    def stop(self, _):
        self.stops += 1
        if self.block_stop:
            self.release.wait(30)
        if self.ack_stop:
            self.terminal('STOPPED')
        return TriggerResponse(success=True, message='accepted')

    def navigate(self, goal):
        self.goals.append(goal)
        self.navigation_goals += 1
        self.base_pose = [2.15, 0, 0]  # Subsequent TF must not overwrite the saved start.
        if self.return_abort and self.navigation_goals == 2:
            self.navigation.set_aborted(MoveBaseResult())
            return
        if self.hold_navigation or (self.return_hold and self.navigation_goals == 2):
            deadline = time.monotonic() + 4
            while not self.navigation.is_preempt_requested() and time.monotonic() < deadline:
                time.sleep(.01)
            if self.navigation.is_preempt_requested():
                self.navigation_cancelled.set()
            self.navigation.set_preempted(MoveBaseResult())
        else:
            self.navigation.set_succeeded(MoveBaseResult())

    def test_arm_lock_cancels_navigation(self):
        self.hold_navigation = True
        self.assertTrue(self.start().success)
        self.wait(lambda: self.navigation_goals == 1)
        self.lock_pub.publish(Bool(data=True))
        self.wait(lambda: 'FAILED' in self.states)
        self.assertTrue(self.navigation_cancelled.wait(1))
        self.assertEqual(self.navigation_goals, 1)

    def test_unknown_sorting_state_cannot_start_motion(self):
        self.state_pub.publish(String(data='UNRECOGNIZED | READY'))
        time.sleep(.1)
        self.assertTrue(self.start().success)
        self.wait(lambda: 'FAILED' in self.states)
        self.assertEqual(self.homes, 0)
        self.assertEqual(self.navigation_goals, 0)

    def test_return_reaches_saved_start_after_stowing(self):
        self.assertTrue(self.start().success)
        self.wait(lambda: 'SUCCEEDED' in self.states)
        self.assertEqual(self.navigation_goals, 2)
        self.assertEqual(self.homes, 2)
        target = self.goals[-1].target_pose
        self.assertEqual(target.header.frame_id, 'map')
        self.assertAlmostEqual(target.pose.position.x, -1.2)
        self.assertAlmostEqual(target.pose.position.y, .4)
        q = target.pose.orientation
        self.assertAlmostEqual(tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2], .6)
        self.assertLess(self.states.index('RETURNING_TO_START'), self.states.index('SUCCEEDED'))

    def test_return_navigation_failure_is_not_success(self):
        self.return_abort = True
        self.assertTrue(self.start().success)
        self.wait(lambda: 'FAILED' in self.states)
        self.assertEqual(self.navigation_goals, 2)
        self.assertNotIn('SUCCEEDED', self.states)

    def test_return_can_be_stopped(self):
        self.return_hold = True
        self.assertTrue(self.start().success)
        self.wait(lambda: self.navigation_goals == 2)
        self.assertTrue(rospy.ServiceProxy('/nav_sorting/stop', Trigger)().success)
        self.wait(lambda: 'STOPPED' in self.states)
        self.assertTrue(self.navigation_cancelled.wait(1))
        self.assertNotIn('SUCCEEDED', self.states)

    def test_return_missing_pose_blocks_mission(self):
        self.assertTrue(self.start().success)
        self.wait(lambda: 'FAILED' in self.states)
        self.assertEqual(self.homes, 0)
        self.assertEqual(self.navigation_goals, 0)

    def test_successful_mission(self):
        self.assertTrue(self.start().success)
        self.wait(lambda: 'SUCCEEDED' in self.states)
        self.assertEqual(self.navigation_goals, 1)
        self.assertEqual(self.stops, 0)

    def test_operation_timeout_is_cancelled(self):
        self.complete = False
        self.assertTrue(self.start().success)
        self.wait(lambda: 'FAILED' in self.states)
        self.assertIn('STOPPING', self.states)
        self.assertEqual(self.stops, 1)
        self.assertEqual(self.navigation_goals, 0)

    def test_missing_ack_recovery_requires_fresh_messages(self):
        self.complete = False
        self.ack_stop = False
        self.assertTrue(self.start().success)
        self.wait(lambda: 'STOP_UNCONFIRMED' in self.states)
        self.assertFalse(self.start().success)
        # An old latched terminal and unlock cannot confirm the new stop RPC.
        self.terminal('STOPPED')
        time.sleep(.1)
        count = self.states.count('STOP_UNCONFIRMED')
        self.assertTrue(self.recover().success)
        self.wait(lambda: self.states.count('STOP_UNCONFIRMED') > count)
        self.assertFalse(self.start().success)
        self.ack_stop = True
        self.assertTrue(self.recover().success)
        self.wait(lambda: self.states[-1] == 'STOPPED')
        self.complete = True
        self.assertTrue(self.start().success)
        self.wait(lambda: 'SUCCEEDED' in self.states)

    def test_late_start_blocks_recovery_until_late_stop_finishes(self):
        self.block_home = True
        self.complete = False
        self.assertTrue(self.start().success)
        self.wait(lambda: 'STOP_UNCONFIRMED' in self.states)
        self.assertFalse(self.recover().success)
        self.assertFalse(self.start().success)
        self.release.set()
        self.wait(lambda: self.stops >= 2)
        time.sleep(.1)
        self.assertTrue(self.recover().success)
        self.wait(lambda: self.states[-1] == 'STOPPED')
        self.assertEqual(self.navigation_goals, 0)

    def test_stop_callback_remains_responsive_with_blocked_rpc(self):
        self.complete = False
        self.block_stop = True
        self.assertTrue(self.start().success)
        self.wait(lambda: self.homes == 1)
        begin = time.monotonic()
        self.assertTrue(rospy.ServiceProxy('/nav_sorting/stop', Trigger)().success)
        self.assertLess(time.monotonic() - begin, .2)
        self.wait(lambda: 'STOP_UNCONFIRMED' in self.states)
        self.assertFalse(self.recover().success)
        self.release.set()
        time.sleep(.15)
        self.assertTrue(self.recover().success)
        self.wait(lambda: self.states[-1] == 'STOPPED')

    def test_concurrent_start_admits_only_one_worker(self):
        self.complete = False
        responses = []
        def start():
            responses.append(rospy.ServiceProxy('/nav_sorting/start', Trigger)().success)
        workers = [threading.Thread(target=start) for _ in range(8)]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()
        self.assertEqual(responses.count(True), 1)
        self.wait(lambda: 'FAILED' in self.states)
        self.assertEqual(self.homes, 1)


if __name__ == '__main__':
    rostest.rosrun('aubo_mobile_nav_sorting', 'cpp_mission', CppMissionTest)
