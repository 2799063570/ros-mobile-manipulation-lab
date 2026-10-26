#!/usr/bin/env python3
"""Full AUBO model with a ROS-only position plant; never connects to SDK/Gazebo."""
import threading
import time
import unittest
import rospy
import rostest
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Float64MultiArray
from nav_msgs.msg import Path
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger


class PreparationTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.q = [0., 0., 1.6292, 0., 1.6628, 0.]
        self.commanded = list(self.q)
        self.preparation = ''
        self.tracker = ''
        self.bottom = ''
        self.candidate = None
        self.samples = []
        self.names = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint',
                      'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
        self.scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        self.pub = rospy.Publisher('/preparation_test/joints', JointState, queue_size=1)
        self.subs = [rospy.Subscriber('/preparation_test/q'+str(i), Float64,
                                     lambda m, j=i: self.command(m, j)) for i in range(6)]
        self.subs += [rospy.Subscriber('/trajectory_preparation/state', String, self.prep_state),
                      rospy.Subscriber('/trajectory_tracking/state', String, self.track_state),
                      rospy.Subscriber('/cartesian_jog/state', String, self.bottom_state),
                      rospy.Subscriber('/trajectory_preparation/candidate_path', Path, self.path),
                      rospy.Subscriber('/trajectory_tracking/samples', Float64MultiArray, self.sample)]
        self.timer = rospy.Timer(rospy.Duration(.01), self.plant)
        for name in ('analyze', 'move', 'check_current', 'stop'):
            rospy.wait_for_service('/trajectory_preparation/'+name, 20)
        rospy.wait_for_service('/trajectory_tracking/preview', 20)
        time.sleep(.5)

    def tearDown(self):
        self.call('stop')
        self.timer.shutdown()

    def command(self, msg, index):
        with self.lock:
            self.commanded[index] = msg.data

    def plant(self, event):
        with self.lock:
            self.q = [q+.7*(desired-q) for q, desired in zip(self.q, self.commanded)]
            msg = JointState()
            msg.name = self.names + ['joint1', 'joint2']
            msg.position = list(self.q) + [0., 0.]
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

    def prep_state(self, msg):
        self.preparation = msg.data

    def bottom_state(self, msg):
        self.bottom = msg.data

    def track_state(self, msg):
        self.tracker = msg.data

    def path(self, msg):
        self.candidate = msg

    def sample(self, msg):
        if not msg.data:
            self.samples = []
        else:
            self.samples.append(msg.data)

    def call(self, name, ns='/trajectory_preparation'):
        return rospy.ServiceProxy(ns+'/'+name, Trigger)()

    def wait(self, predicate, timeout=20):
        until = time.monotonic()+timeout
        while time.monotonic()<until and not rospy.is_shutdown():
            if predicate():
                return
            time.sleep(.02)
        self.fail('Timeout: '+self.preparation+' / '+self.tracker)

    def test_search_move_and_tracking(self):
        # Test the user's 80 mm / 55 s circle against the actual AUBO kinematics.
        rospy.set_param('/trajectory_tracking/radius', .08)
        rospy.set_param('/trajectory_tracking/duration', 55.)
        rospy.set_param('/trajectory_tracking/max_speed', .1)
        rospy.set_param('/trajectory_tracking/max_extent', .25)
        with self.lock:
            before = list(self.q)
        result = self.call('analyze')
        self.assertTrue(result.success, result.message)
        self.wait(lambda: self.candidate is not None)
        self.assertGreater(len(self.candidate.poses), 300)
        with self.lock:
            self.assertLess(max(abs(a-b) for a, b in zip(self.q, before)), 1e-5,
                            'Searching must not move the robot')
        # A stale recommendation cannot move after the trajectory changes.
        rospy.set_param('/trajectory_tracking/radius', .01)
        self.assertFalse(self.call('move').success)
        rospy.set_param('/trajectory_tracking/duration', 10.)
        result = self.call('analyze')
        self.assertTrue(result.success, result.message)
        result = self.call('move')
        self.assertTrue(result.success, result.message)
        self.wait(lambda: self.preparation.startswith('ARRIVED'), 35)
        result = self.call('preview', '/trajectory_tracking')
        self.assertTrue(result.success, result.message)
        result = self.call('start', '/trajectory_tracking')
        self.assertTrue(result.success, result.message)
        self.wait(lambda: self.tracker.startswith('COMPLETED'), 18)
        self.assertGreater(len(self.samples), 100)
        self.assertTrue(all(len(row) == 16 for row in self.samples))
        self.wait(lambda: self.bottom == 'DISABLED')
        # Sine shares the same solver and fixed-orientation path check.
        rospy.set_param('/trajectory_tracking/shape', 'sine')
        rospy.set_param('/trajectory_tracking/length', .02)
        rospy.set_param('/trajectory_tracking/amplitude', .005)
        result = self.call('check_current')
        self.assertTrue(result.success, result.message)
        # Both the preparation checker and servo must see explicitly published obstacles.
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        obstacle = CollisionObject()
        obstacle.header.frame_id = 'base_link'
        obstacle.id = 'test_workspace_block'
        obstacle.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [3., 3., 3.]
        pose = Pose()
        pose.orientation.w = 1.
        obstacle.primitives = [box]
        obstacle.primitive_poses = [pose]
        scene.world.collision_objects = [obstacle]
        self.scene_pub.publish(scene)
        time.sleep(.3)
        self.assertFalse(self.call('check_current').success, 'Known scene collision must reject preview')
        obstacle.operation = CollisionObject.REMOVE
        self.scene_pub.publish(scene)
        time.sleep(.3)
        self.wait(lambda: rospy.ServiceProxy('/cartesian_jog/reset', Trigger)().success, 4)
        self.wait(lambda: self.bottom == 'DISABLED')
        self.assertTrue(self.call('check_current').success, 'Removing the obstacle must restore the path')
        # An unreachable positive-X sweep must be rejected, regardless of speed budget.
        rospy.set_param('/trajectory_tracking/length', .49)
        rospy.set_param('/trajectory_tracking/amplitude', .001)
        rospy.set_param('/trajectory_tracking/duration', 200.)
        result = self.call('check_current')
        self.assertFalse(result.success, 'Must reject an unreachable fixed-orientation path')


if __name__ == '__main__':
    rospy.init_node('preparation_integration_test')
    rostest.rosrun('aubo_trajectory_tracking', 'preparation_integration', PreparationTest)
