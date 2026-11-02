#!/usr/bin/env python3
"""ROS topic/file integration; only synthetic telemetry is published."""
import json
import math
import time
import unittest
from pathlib import Path
import rospy
import rostest
import rosgraph
from std_msgs.msg import Header, String, Float64MultiArray


class CaptureTransportTest(unittest.TestCase):
    def wait(self, predicate, seconds=6):
        until=time.monotonic()+seconds
        while time.monotonic()<until and not rospy.is_shutdown():
            value=predicate()
            if value:
                return value
            time.sleep(.03)
        self.fail('Timed out waiting for collector output')

    def test_two_runs(self):
        topics={'metadata':('run_metadata',String), 'start':('run_start',Header),
                'state':('state',String), 'tcp':('samples',Float64MultiArray),
                'joints':('joint_samples',Float64MultiArray), 'servo':('servo_samples',Float64MultiArray)}
        pubs={key:rospy.Publisher('/trajectory_tracking/'+name,cls,queue_size=100)
              for key,(name,cls) in topics.items()}
        self.wait(lambda: all(pub.get_num_connections()>0 for pub in pubs.values()))
        system=rosgraph.Master(rospy.get_name()).getSystemState()
        publishers=system[0]
        self.assertEqual([topic for topic,nodes in publishers if '/capture_observer' in nodes], ['/rosout'])
        subscribed={topic for topic,nodes in system[1] if '/capture_observer' in nodes}
        self.assertNotIn('/capture_fixture/jog/command',subscribed,
                         'Observer must not occupy the single execution subscriber slot')
        self.assertIn('/trajectory_tracking/command_telemetry',subscribed)
        output=Path(rospy.get_param('/experiment_capture_test/output'))
        run_ids=[]
        for index in range(2):
            stamp=rospy.Time.now(); run_id=str(stamp.to_nsec()); run_ids.append(run_id)
            metadata={'schema_version':1,'run_start_ns':run_id,
                      'parameters':dict(shape='circle',plane='xy',radius=.12,duration=2.,
                                        kp=2.,ki=0.,kd=0.,acceleration_ff_time=.1),
                      'origin_m':[0.,0.,0.], 'start_orientation_xyzw':[0.,0.,0.,1.]}
            pubs['metadata'].publish(String(json.dumps(metadata)))
            pubs['start'].publish(Header(stamp=stamp))
            pubs['state'].publish(String('RUNNING: synthetic telemetry'))
            first=time.monotonic()
            limit=2.05 if index==0 else .25
            while time.monotonic()-first<limit:
                t=(rospy.Time.now()-stamp).to_sec()
                phase=int(t>=2.)
                error=.002 if phase else .001
                pubs['tcp'].publish(Float64MultiArray(data=[t,0,0,0,error,0,0,error,
                                        .01,.01,.01,.01,.005,phase,0,error]))
                pubs['joints'].publish(Float64MultiArray(data=[t]+[0.]*33))
                pubs['servo'].publish(Float64MultiArray(data=[t]+[0.]*11+[1.,2.]))
                time.sleep(.02)
            state='COMPLETED: fixture' if index==0 else 'FAULT: injected fixture'
            pubs['state'].publish(String(state))
            summary_file=self.wait(lambda: next(output.glob('session_*/run_'+run_id+'/summary.json'),None))
            summary=json.loads(summary_file.read_text())
            self.assertEqual(summary['parameters']['kp'],2.,'Must use frozen metadata, not server kp=9')
            self.assertAlmostEqual(summary['tcp']['running']['total_mm']['rms'],1.,places=8)
            self.assertEqual(summary['eligible_for_aggregate'],index==0,summary['quality'])
        report=self.wait(lambda: next((p for p in output.glob('session_*/comparison/comparison.json')
                                      if run_ids[0] in p.read_text()),None))
        comparison=json.loads(report.read_text())
        self.assertEqual(len(comparison['runs']),2)
        self.assertEqual(comparison['aggregates'][0]['repeats'],1)
        self.wait(lambda: (report.parent/'report.md').exists())
        self.assertTrue(list(report.parent.glob('comparison_*.png')))


if __name__=='__main__':
    rospy.init_node('experiment_capture_test')
    rostest.rosrun('aubo_trajectory_tracking','experiment_capture',CaptureTransportTest)
