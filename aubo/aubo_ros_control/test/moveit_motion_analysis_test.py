#!/usr/bin/env python3
"""Regression for fault/source reporting, using offline bags and no ROS master."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import rosbag
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String
from moveit_msgs.msg import MoveGroupActionGoal

spec = importlib.util.spec_from_file_location(
    'motion_analysis', Path(__file__).resolve().parents[1] / 'scripts/analyze_moveit_motion.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


class MotionAnalysisTest(unittest.TestCase):
    def test_runtime_scaling_changes_are_preserved_per_request(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'scaling.bag'
            with rosbag.Bag(str(path), 'w') as bag:
                for index, velocity in enumerate([.5, .7]):
                    message = MoveGroupActionGoal()
                    message.goal_id.id = 'goal_' + str(index)
                    message.goal.request.group_name = 'aubo_i5'
                    message.goal.request.max_velocity_scaling_factor = velocity
                    message.goal.request.max_acceleration_scaling_factor = .4
                    bag.write(analysis.MOVE_GROUP_GOAL_TOPIC, message, rospy.Time(100 + index))
            rows = analysis.analyze(path, .15)['motion_entrypoints']['move_group_requests']
            self.assertEqual([r['requested_velocity_scaling'] for r in rows], [.5, .7])
            self.assertEqual([r['requested_acceleration_scaling'] for r in rows], [.4, .4])
            self.assertEqual([r['goal_id'] for r in rows], ['goal_0', 'goal_1'])
            self.assertEqual([r['time_ros_s'] for r in rows], [100., 101.])

    def test_source_takeover_and_latched_faults_are_reported(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'faults.bag'
            with rosbag.Bag(str(path), 'w') as bag:
                for index, source in enumerate(['push', 'query_latched']):
                    message = DiagnosticArray()
                    status = DiagnosticStatus(name='Aubo hardware motion flow')
                    values = dict(feedback_source=source,
                                  feedback_ignored_push_samples_total=str(index * 7),
                                  feedback_sample_age_ms=str(30 + index * 190),
                                  feedback_timeout_latched='true' if index else 'false',
                                  command_queue_limit_exceeded='true' if index else 'false',
                                  jtc_queue_target_points='40',
                                  jtc_backpressure_waits_total=str(index * 12),
                                  queue_oldest_command_age_ms=str(index * 1001),
                                  mac_slow_query_samples_over_50ms=str(index))
                    status.values = [KeyValue(k, v) for k, v in values.items()]
                    message.status = [status]
                    bag.write(analysis.FLOW_TOPIC, message, rospy.Time(100 + index))
            report = analysis.analyze(path, .15)
            flow = report['hardware_flow']
            self.assertEqual(flow['feedback_sources'], ['push', 'query_latched'])
            self.assertEqual(flow['feedback_ignored_push_samples_total'], 7)
            self.assertEqual(flow['feedback_max_sample_age_ms'], 220)
            self.assertTrue(flow['feedback_timeout_latched'])
            self.assertTrue(flow['command_queue_limit_exceeded'])
            self.assertEqual(flow['max_oldest_queued_command_age_ms'], 1001)
            self.assertEqual(flow['slow_mac_query_samples_over_50ms'], 1)
            self.assertEqual(flow['jtc_queue_target_points'], 40)
            self.assertEqual(flow['jtc_backpressure_waits_total'], 12)
            self.assertTrue(any('独立路点查询已接管' in s for s in report['findings']))

    def test_bags_without_new_fields_or_flow_remain_readable(self):
        for with_flow in [False, True]:
            with tempfile.TemporaryDirectory() as root:
                path = Path(root) / 'old.bag'
                with rosbag.Bag(str(path), 'w') as bag:
                    if with_flow:
                        message = DiagnosticArray()
                        message.status = [DiagnosticStatus(name='Aubo hardware motion flow')]
                        bag.write(analysis.FLOW_TOPIC, message, rospy.Time(100))
                    else:
                        bag.write('/unrelated', String(data='no flow'), rospy.Time(100))
                report = analysis.analyze(path, .15)
                if with_flow:
                    self.assertEqual(report['hardware_flow']['feedback_sources'], [])
                    self.assertFalse(report['hardware_flow']['feedback_timeout_latched'])
                else:
                    self.assertIsNone(report['hardware_flow'])


if __name__ == '__main__':
    unittest.main()
