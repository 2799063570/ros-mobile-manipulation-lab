#!/usr/bin/env python3
"""Exercises an automatic service runner against a toy position plant only."""
import json
import subprocess
import threading
import time
import unittest
from pathlib import Path
import rospy
import rostest
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Header,Float64MultiArray
from std_srvs.srv import SetBool,SetBoolResponse,Trigger,TriggerResponse


class RunnerFixture(unittest.TestCase):
    def test_matrix_and_abort(self):
        self.lock=threading.RLock();self.q=[0.,0.,.3];self.v=[0.,0.,0.]
        self.enabled=False;self.started=0;self.run_start=None;self.inject=False;self.injected=False
        joint=rospy.Publisher('/runner_fixture/joints',JointState,queue_size=10)
        state=rospy.Publisher('/cartesian_jog/state',String,queue_size=10)
        joints=rospy.Publisher('/trajectory_tracking/joint_samples',Float64MultiArray,queue_size=100)
        servo=rospy.Publisher('/trajectory_tracking/servo_samples',Float64MultiArray,queue_size=100)
        def enable(req):
            with self.lock:
                self.enabled=req.data
                if req.data:self.started+=1
                else:self.v=[0.,0.,0.]
            return SetBoolResponse(True,'fake plant only')
        def command(msg):
            with self.lock:self.v=[msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z]
        def run(msg):
            with self.lock:self.run_start=None if msg.stamp.is_zero() else msg.stamp
        services=[rospy.Service('/cartesian_jog/set_enabled',SetBool,enable),
                  rospy.Service('/trajectory_preparation/check_current',Trigger,
                                lambda req:TriggerResponse(True,'toy model check'))]
        subs=[rospy.Subscriber('/cartesian_jog/command',TwistStamped,command),
              rospy.Subscriber('/trajectory_tracking/run_start',Header,run)]
        previous=[time.monotonic()]
        def tick(event):
            now=time.monotonic();dt=min(.03,now-previous[0]);previous[0]=now
            with self.lock:
                if self.enabled:self.q=[q+v*dt for q,v in zip(self.q,self.v)]
                if self.inject and self.enabled and self.run_start and not self.injected:
                    self.q[0]+=.015;self.injected=True
                msg=JointState();msg.header.stamp=rospy.Time.now();msg.name=['x','y','z'];msg.position=list(self.q)
                joint.publish(msg);state.publish(String('ENABLED_IDLE' if self.enabled else 'DISABLED'))
                if self.run_start:
                    t=(msg.header.stamp-self.run_start).to_sec()
                    joints.publish(Float64MultiArray(data=[t]+[0.]*33))
                    servo.publish(Float64MultiArray(data=[t]+[0.]*11+[1.,0.]))
        timer=rospy.Timer(rospy.Duration(.01),tick)
        path=Path(__file__).resolve().parents[1]/'scripts/run_tracking_experiments.py'
        root=Path('/tmp/aubo_real_runner_fixture_20261102')
        try:
            rospy.wait_for_service('/trajectory_tracking/preview',5)
            result=subprocess.run(['/usr/bin/python3',str(path),'--execute','--backend','gazebo',
                         '--repeats','1','--output',str(root)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                         text=True,timeout=55)
            self.assertEqual(result.returncode,0,result.stdout)
            manifests=sorted(root.glob('real_pid_*/experiment.json'),key=lambda p:p.stat().st_mtime)
            data=json.loads(manifests[-1].read_text())
            self.assertEqual(data['state'],'COMPLETE')
            self.assertEqual(len(data['outcomes']),6)
            self.assertTrue(all(not row['quality'] for row in data['outcomes']))
            self.assertEqual(self.started,6)
            self.assertFalse(self.enabled)
            self.assertEqual(rospy.get_param('/trajectory_tracking/ki'),0.)
            self.assertEqual(rospy.get_param('/trajectory_tracking/kd'),0.)
            self.inject=True
            result=subprocess.run(['/usr/bin/python3',str(path),'--execute','--backend','gazebo',
                         '--repeats','1','--output',str(root)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                         text=True,timeout=12)
            self.assertEqual(result.returncode,1,result.stdout)
            manifests=sorted(root.glob('real_pid_*/experiment.json'),key=lambda p:p.stat().st_mtime)
            data=json.loads(manifests[-1].read_text())
            self.assertEqual(data['state'],'ABORTED')
            self.assertLessEqual(self.started,7,'Fault must prevent remaining starts')
            self.assertFalse(self.enabled)
        finally:
            timer.shutdown()
            for sub in subs:sub.unregister()
            for srv in services:srv.shutdown()


if __name__=='__main__':
    rospy.init_node('aubo_cartesian_jog')
    rostest.rosrun('aubo_trajectory_tracking','experiment_runner',RunnerFixture)
