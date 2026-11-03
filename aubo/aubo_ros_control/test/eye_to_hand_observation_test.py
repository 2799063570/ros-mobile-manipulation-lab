#!/usr/bin/env python3
"""Production controller with synthetic feedback only: no SDK/hardware."""
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool


class ObservationTest(unittest.TestCase):
    def test_initial_observation_coast_return_reacquire_and_aligned_loss(self):
        lock = threading.RLock()
        q = [0.]*6
        observation = [.06, 0., 0., 0., 0., 0.]
        outputs = [[] for _ in range(6)]
        states = []
        settings = dict(targets=False, goal=.24, align=False)
        feedback = rospy.Publisher('/joint_states', JointState, queue_size=1)
        targets = rospy.Publisher('/visual_servo/target_pose', PoseStamped, queue_size=1)
        names = ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']
        def output(message, i):
            with lock:
                q[i] = message.data
                outputs[i].append(message.data)
        def state(message):
            with lock: states.append((message.data, list(q)))
        subscribers = [rospy.Subscriber('/observation/q%d'%i, Float64, output, callback_args=i, queue_size=100)
                       for i in range(6)]
        subscribers.append(rospy.Subscriber('/visual_servo/state', String, state, queue_size=100))
        def tick(event):
            with lock:
                current = list(q); send = settings['targets']
                goal = sum(current) if settings['align'] else settings['goal']
            sample = JointState(); sample.header.stamp = rospy.Time.now()
            sample.name, sample.position = names, current
            feedback.publish(sample)
            if send:
                target = PoseStamped(); target.header.stamp = rospy.Time.now()
                target.header.frame_id = 'base_link'; target.pose.position.x = goal
                target.pose.orientation.w = 1.; targets.publish(target)
        timer = rospy.Timer(rospy.Duration(.01), tick)
        enable = rospy.ServiceProxy('/visual_servo/set_enabled', SetBool)
        def wait(predicate, timeout=8., message='condition timed out'):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                with lock:
                    if predicate(): return
                time.sleep(.01)
            self.fail(message+'; states='+str([s[0] for s in states]))
        try:
            rospy.wait_for_service('/visual_servo/set_enabled', timeout=8)
            time.sleep(.2)
            deadline = time.monotonic()+3
            while not enable(True).success:
                self.assertLess(time.monotonic(), deadline); time.sleep(.03)
            # No camera target is required to reach the explicitly enabled viewpoint.
            wait(lambda: any(s[0]=='MOVE_TO_OBSERVATION' for s in states))
            wait(lambda: states[-1][0]=='WAITING' and max(abs(a-b) for a,b in zip(q,observation))<.004)
            self.assertFalse(any(s[0]=='TRACKING' for s in states))
            with lock: settings['targets']=True
            wait(lambda: states[-1][0]=='TRACKING' and sum(q)>.12)
            with lock:
                start=len(states); settings['targets']=False
            wait(lambda: any(s[0]=='COAST' for s in states[start:]))
            wait(lambda: states[-1][0]=='RETURN_TO_OBSERVATION', timeout=10.)
            # Reappearance during the return must not interrupt the saved-joint move.
            with lock:
                return_index=len(states)-1; settings['targets']=True; settings['goal']=.26
            wait(lambda: len(states)>return_index+1 and states[-1][0]!='RETURN_TO_OBSERVATION', timeout=12.)
            with lock:
                next_state, arrived=states[return_index+1]
            self.assertIn(next_state, ('WAITING','HOLD'))
            self.assertLess(max(abs(a-b) for a,b in zip(arrived,observation)), .004)
            wait(lambda: states[-1][0]=='TRACKING' and sum(q)>.12)
            with lock: settings['align']=True
            wait(lambda: states[-1][0]=='ALIGNED')
            with lock:
                aligned_index=len(states); settings['targets']=False
            wait(lambda: any(s[0]=='RETURN_TO_OBSERVATION' for s in states[aligned_index:]), timeout=10.)
            wait(lambda: states[-1][0]=='WAITING' and max(abs(a-b) for a,b in zip(q,observation))<.004, timeout=12.)
            with lock: frozen=list(q)
            time.sleep(.3)
            with lock: self.assertLess(max(abs(a-b) for a,b in zip(q,frozen)), 1e-5)
            # An explicit stop cancels a return, including when the target reappears.
            with lock:
                settings.update(targets=True, align=False, goal=.28)
            wait(lambda: states[-1][0]=='TRACKING' and sum(q)>.12)
            with lock: settings['targets']=False
            wait(lambda: states[-1][0]=='RETURN_TO_OBSERVATION', timeout=10.)
            with lock: return_start=list(q)
            wait(lambda: states[-1][0]=='RETURN_TO_OBSERVATION' and
                 max(abs(a-b) for a,b in zip(q,return_start))>.001)
            self.assertTrue(enable(False).success)
            with lock: settings['targets']=True
            wait(lambda: states[-1][0]=='DISABLED')
            with lock: frozen=list(q); stopped_index=len(states)
            time.sleep(.4)
            with lock:
                self.assertLess(max(abs(a-b) for a,b in zip(q,frozen)), 1e-5)
                self.assertFalse(any(s[0] in ('TRACKING','RETURN_TO_OBSERVATION') for s in states[stopped_index:]))
            # Check the actual command sequence including return and stop tails.
            for i, points in enumerate(outputs):
                old_q=old_v=old_a=0.
                for value in points:
                    v=(value-old_q)/.005; a=(v-old_v)/.005; j=(a-old_a)/.005
                    self.assertLessEqual(abs(v),(.35 if i<3 else .5)+1e-7)
                    self.assertLessEqual(abs(a),(.4 if i<3 else .6)+1e-6)
                    self.assertLessEqual(abs(j),(8 if i<3 else 12)+1e-4)
                    old_q,old_v,old_a=value,v,a
        finally:
            try: enable(False)
            except rospy.ServiceException: pass
            timer.shutdown()
            for subscriber in subscribers: subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('eye_to_hand_observation_test')
    rostest.rosrun('aubo_ros_control','eye_to_hand_observation',ObservationTest)
