#!/usr/bin/env python3
"""Audit a saved RobotTrajectory JSON against a recorded scene or the live scene."""
import argparse
import json
from pathlib import Path
import genpy
import rospy
from aubo_thesis_experiments.srv import AuditTrajectory, AuditTrajectoryRequest
from aubo_thesis_experiments.ros_codec import to_dict

def main():
    p = argparse.ArgumentParser()
    p.add_argument('input', help='JSON with start_state, trajectory, optional scene/path_constraints/goal_constraints')
    p.add_argument('--group', help='default: recorded group_name, otherwise aubo_i5')
    p.add_argument('--step', type=float, help='default: recorded max_joint_step, otherwise 0.02 rad')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    record = json.loads(Path(a.input).read_text(encoding='utf-8'))
    req = AuditTrajectoryRequest()
    genpy.message.fill_message_args(req.start_state, [record['start_state']])
    genpy.message.fill_message_args(req.trajectory, [record['trajectory']])
    if 'scene' in record:
        genpy.message.fill_message_args(req.scene,[record['scene']]); req.use_provided_scene = True
    if 'path_constraints' in record:
        genpy.message.fill_message_args(req.path_constraints,[record['path_constraints']])
    if 'goal_constraints' in record:
        from moveit_msgs.msg import Constraints
        for row in record['goal_constraints']:
            constraint = Constraints(); genpy.message.fill_message_args(constraint,[row]); req.goal_constraints.append(constraint)
    req.group_name = a.group or record.get('group_name','aubo_i5')
    req.max_joint_step = a.step if a.step is not None else record.get('max_joint_step',.02)
    rospy.init_node('audit_saved_trajectory',anonymous=True)
    rospy.wait_for_service('/thesis_experiment/audit_trajectory',timeout=30)
    result = rospy.ServiceProxy('/thesis_experiment/audit_trajectory',AuditTrajectory)(req)
    Path(a.output).write_text(json.dumps(to_dict(result),indent=2),encoding='utf-8')
    print(result.reason)
    return 0 if result.valid else 2

if __name__ == '__main__':
    raise SystemExit(main())
