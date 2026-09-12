#!/usr/bin/env python3
"""ROS integration test: real controller, synthetic plant and planning service.

No SDK connection or physical robot is used. The prismatic test chain makes
the expected Cartesian endpoint exact, independent of an IK plugin.
"""
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint


class HybridControlTest(unittest.TestCase):
    def test_hybrid_lifecycle(self):
        self.lock = threading.Lock()
        self.q = [0.0] * 6
        self.states = []
        self.target = 0.24
        self.visible = True
        self.feedback_enabled = True
        self.mode = "success"
        self.calls = 0
        self.plan_entered = threading.Event()
        self.plan_release = threading.Event()
        names = ["shoulder_joint", "upperArm_joint", "foreArm_joint",
                 "wrist1_joint", "wrist2_joint", "wrist3_joint"]
        self.joints = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.targets = rospy.Publisher('/visual_servo/target_pose', PoseStamped, queue_size=1)
        self.scene_ready = rospy.Publisher(
            '/hybrid/planning_scene_ready', Bool, queue_size=1, latch=True)
        self.scene_ready.publish(False)

        def output(msg, index):
            with self.lock:
                self.q[index] = msg.data

        self.subscribers = [rospy.Subscriber('/hybrid_test/q%d' % i, Float64,
                                             output, callback_args=i) for i in range(6)]
        self.subscribers.append(rospy.Subscriber('/visual_servo/state', String,
                                                 lambda msg: self.states.append(msg.data)))

        def tick(_event):
            with self.lock:
                q = list(self.q)
            if self.feedback_enabled:
                joint = JointState()
                joint.header.stamp = rospy.Time.now()
                joint.name, joint.position = names, q
                self.joints.publish(joint)
            if self.visible:
                target = PoseStamped()
                target.header.stamp = rospy.Time.now()
                target.header.frame_id = 'base_link'
                target.pose.position.x = self.target
                target.pose.orientation.w = 1.0
                self.targets.publish(target)

        def plan(req):
            self.calls += 1
            mode = self.mode
            self.plan_entered.set()
            if mode == 'delayed':
                self.plan_release.wait(8.0)
            response = GetMotionPlanResponse()
            response.motion_plan_response.error_code.val = -1 if mode == 'failure' else 1
            trajectory = response.motion_plan_response.trajectory.joint_trajectory
            # Deliberately reverse the order to verify joint-name remapping.
            trajectory.joint_names = names[::-1]
            start = list(req.motion_plan_request.start_state.joint_state.position)
            goal = req.motion_plan_request.goal_constraints[0].position_constraints[0]
            end = list(start)
            end[0] += goal.constraint_region.primitive_poses[0].position.x - sum(start)
            duration = 0.05 if mode == 'fast' else 2.0
            for q, seconds in ((start, 0.0), (end, duration)):
                point = JointTrajectoryPoint()
                point.positions = q[::-1]
                point.time_from_start = rospy.Duration(seconds)
                trajectory.points.append(point)
            if mode == 'invalid':
                trajectory.points[-1].positions[0] = float('nan')
            return response

        self.service = rospy.Service('/plan_kinematic_path', GetMotionPlan, plan)
        self.timer = rospy.Timer(rospy.Duration(0.01), tick)
        rospy.wait_for_service('/visual_servo/set_enabled', timeout=10)
        enable = rospy.ServiceProxy('/visual_servo/set_enabled', SetBool)
        try:
            time.sleep(0.3)
            enable(True)
            # No plan or motion is allowed until the collision scene confirms
            # that the table has reached move_group.
            self.wait_for(lambda: self.states[-1:] == ['WAITING'], 2)
            time.sleep(0.3)
            self.assertEqual(0, self.calls)
            self.assertLess(max(abs(value) for value in self.q), 0.002)
            self.scene_ready.publish(True)
            self.wait_for(lambda: 'ALIGNED' in self.states, 20)
            self.assertIn('PLANNING', self.states)
            self.assertIn('APPROACH', self.states)
            self.assertIn('TRACKING', self.states)
            self.assertLess(abs(sum(self.q) - self.target), 0.012)
            # Small online displacement must converge without another plan.
            calls = self.calls
            self.states.clear()
            self.target += 0.025
            self.wait_for(lambda: 'TRACKING' in self.states, 3)
            self.wait_for(lambda: self.states[-1:] == ['ALIGNED'], 12)
            self.assertEqual(calls, self.calls)

            # Loss discards the trajectory and holds; no blind search/coast.
            self.visible = False
            self.wait_for(lambda: self.states[-1:] == ['WAITING'], 3)
            time.sleep(0.2)
            held = list(self.q)
            time.sleep(0.4)
            self.assertLess(max(abs(a-b) for a, b in zip(held, self.q)), 0.002)
            self.visible = True

            # A geometrically valid MoveIt path with permissive timestamps is
            # retimed to the controller's safer velocity limits, not rejected.
            enable(False)
            self.mode = 'fast'
            self.target = sum(self.q) + 0.25
            self.states.clear()
            enable(True)
            self.wait_for(lambda: 'APPROACH' in self.states, 5)
            self.wait_for(lambda: 'TRACKING' in self.states, 8)
            self.assertNotIn('HOLD', self.states)

            for mode in ('failure', 'invalid'):
                enable(False)
                self.mode = mode
                self.target = sum(self.q) + 0.25
                self.states.clear()
                enable(True)
                self.wait_for(lambda: 'HOLD' in self.states, 5)
                self.assertNotIn('APPROACH', self.states)

            # Disable while a plan is in flight: its late response cannot move.
            enable(False)
            self.mode = 'delayed'
            self.plan_entered.clear()
            self.plan_release.clear()
            enable(True)
            self.assertTrue(self.plan_entered.wait(5))
            enable(False)
            self.states.clear()
            held = list(self.q)
            self.plan_release.set()
            time.sleep(0.5)
            self.assertNotIn('APPROACH', self.states)
            self.assertLess(max(abs(a-b) for a, b in zip(held, self.q)), 0.002)

            # A hung planning service must not block the control watchdog.
            self.plan_entered.clear()
            self.plan_release.clear()
            enable(True)
            self.assertTrue(self.plan_entered.wait(5))
            self.wait_for(lambda: self.states[-1:] == ['HOLD'], 4)
            self.plan_release.set()
            time.sleep(0.3)
            self.assertEqual('HOLD', self.states[-1])

            # Target drift while approaching invalidates the remaining path.
            enable(False)
            self.mode = 'success'
            self.target = sum(self.q) + 0.25
            self.states.clear()
            enable(True)
            self.wait_for(lambda: 'APPROACH' in self.states, 5)
            self.states.clear()
            self.target += 0.10
            self.wait_for(lambda: 'WAITING' in self.states, 2)
            self.wait_for(lambda: self.states[-1:] == ['APPROACH'], 5)
            self.visible = False
            self.wait_for(lambda: self.states[-1:] == ['WAITING'], 2)
            time.sleep(0.2)
            held = list(self.q)
            time.sleep(0.3)
            self.assertLess(max(abs(a-b) for a, b in zip(held, self.q)), 0.002)
            self.visible = True

            enable(False)
            self.mode = 'success'
            self.states.clear()
            enable(True)
            self.wait_for(lambda: 'APPROACH' in self.states, 5)
            self.feedback_enabled = False
            self.wait_for(lambda: self.states[-1:] == ['HOLD'], 2)
            self.feedback_enabled = True
            time.sleep(0.3)
            self.assertEqual('HOLD', self.states[-1])
        finally:
            self.plan_release.set()
            enable(False)
            self.timer.shutdown()
            self.service.shutdown()

    def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(predicate(), 'Timed out; states=%s' % self.states)


if __name__ == '__main__':
    rospy.init_node('hybrid_control_test')
    rostest.rosrun('aubo_ros_control', 'hybrid_control', HybridControlTest)
