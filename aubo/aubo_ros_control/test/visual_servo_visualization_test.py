#!/usr/bin/env python3
"""Passive monitor fixture; no servo, SDK, or joint command publishers."""
import math
import time
import unittest

import rospy
import rostest
import tf2_ros
from aubo_ros_control.msg import VisualServoStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker


class VisualizationTest(unittest.TestCase):
    def test_geometry_staleness_and_delayed_tf(self):
        mode=rospy.get_param('~mode')
        latest=dict(status=None,markers=None)
        flags=dict(joints=True,target=True,frame='view_tcp' if mode=='eye_in_hand' else 'base_link',nan=False,q=.1)
        feedback=rospy.Publisher('/view_test/joints',JointState,queue_size=1)
        target=rospy.Publisher('/view_test/target',PoseStamped,queue_size=1)
        state=rospy.Publisher('/view_test/state',String,queue_size=1)
        status_sub=rospy.Subscriber('/visual_servo/status',VisualServoStatus,lambda m:latest.update(status=m))
        marker_sub=rospy.Subscriber('/visual_servo/markers',MarkerArray,lambda m:latest.update(markers=m))
        names=['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']

        def tick(_):
            state.publish(String('TRACKING'))
            if flags['joints']:
                msg=JointState(); msg.header.stamp=rospy.Time.now()
                msg.name=names; msg.position=[flags['q'],0,0,0,0,0]; feedback.publish(msg)
            if flags['target']:
                msg=PoseStamped(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id=flags['frame']
                msg.pose.position.x=float('nan') if flags['nan'] else .2
                msg.pose.position.y=.02; msg.pose.position.z=.3; msg.pose.orientation.w=1
                target.publish(msg)

        def wait(predicate):
            end=time.monotonic()+4
            while time.monotonic()<end:
                if latest['status'] and predicate(latest['status']): return latest['status']
                time.sleep(.02)
            self.fail('monitor condition did not arrive: '+str(latest['status']))

        timer=rospy.Timer(rospy.Duration(.02),tick)
        try:
            s=wait(lambda s:s.error_valid)
            self.assertAlmostEqual(s.tcp_position.x,.1,places=5)
            self.assertEqual(s.error_frame,'view_tcp' if mode=='eye_in_hand' else 'base_link')
            expected=[.15,.02,.20] if mode=='eye_in_hand' else [.11,0,.33]
            for actual,wanted in zip([s.position_error.x,s.position_error.y,s.position_error.z],expected):
                self.assertAlmostEqual(actual,wanted,places=5)
            self.assertAlmostEqual(s.position_error_norm,math.sqrt(sum(v*v for v in expected)),places=5)
            self.assertAlmostEqual(s.goal_position.x,.08 if mode=='eye_in_hand' else .21,places=5)
            if mode=='eye_in_hand':
                self.assertAlmostEqual(s.goal_position.y,.15,places=5) # Tool is rotated 90deg.
            flags['q']=.11
            wait(lambda s:s.feedback_valid and abs(s.tcp_position.x-.11)<1e-6)
            time.sleep(.12)
            trail=next(m for m in latest['markers'].markers if m.id==4)
            self.assertEqual(trail.action,Marker.ADD)
            self.assertGreaterEqual(len(trail.points),2)
            flags['q']=.1
            wait(lambda s:s.feedback_valid and abs(s.tcp_position.x-.1)<1e-6)
            flags['target']=False
            wait(lambda s:not s.target_valid and not s.error_valid and s.feedback_valid)
            time.sleep(.12)
            markers={m.id:m for m in latest['markers'].markers}
            for i in [1,2,3]: self.assertEqual(markers[i].action,Marker.DELETE)
            self.assertEqual(markers[0].action,Marker.ADD)

            flags['nan']=True; flags['target']=True
            time.sleep(.3)
            self.assertFalse(latest['status'].error_valid)
            flags['nan']=False; flags['frame']='delayed_target_frame'
            time.sleep(.25)
            self.assertFalse(latest['status'].error_valid)
            broadcaster=tf2_ros.StaticTransformBroadcaster()
            transform=TransformStamped(); transform.header.stamp=rospy.Time.now()
            transform.header.frame_id='view_tcp' if mode=='eye_in_hand' else 'base_link'
            transform.child_frame_id='delayed_target_frame'; transform.transform.translation.x=.05
            transform.transform.rotation.w=1; broadcaster.sendTransform(transform)
            s=wait(lambda s:s.error_valid)
            self.assertAlmostEqual(s.position_error.x,.20 if mode=='eye_in_hand' else .16,places=5)

            flags['joints']=False
            wait(lambda s:not s.feedback_valid and not s.error_valid)
            time.sleep(.12)
            markers={m.id:m for m in latest['markers'].markers}
            self.assertEqual(markers[0].action,Marker.DELETE)
            self.assertEqual(markers[3].action,Marker.DELETE)
        finally:
            timer.shutdown(); status_sub.unregister(); marker_sub.unregister()


if __name__=='__main__':
    rospy.init_node('visualization_test')
    rostest.rosrun('aubo_ros_control','visual_servo_visualization',VisualizationTest)
