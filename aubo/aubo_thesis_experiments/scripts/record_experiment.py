#!/usr/bin/env python3
"""Passive ROS logger: never publishes motion commands or calls start services."""
import hashlib
import json
import os
import platform
import subprocess
import threading
import time
from pathlib import Path
import rospy
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from moveit_msgs.msg import DisplayTrajectory
from aubo_sorting_core.msg import ExperimentAudit
from aubo_perception.msg import DetectedObjectArray
from sensor_msgs.msg import CameraInfo
from aubo_thesis_experiments.ros_codec import to_dict

def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()

class Recorder:
    def __init__(self):
        output = Path(os.path.expanduser(rospy.get_param('~output_dir')))
        # Exclusive creation prevents accidental mixing with an earlier batch.
        output.mkdir(parents=True, exist_ok=False)
        self.lock, self.output = threading.Lock(), output
        self.file = (output/'events.jsonl').open('x', encoding='utf-8')
        self.trajectory_file = (output/'trajectories.jsonl').open('x', encoding='utf-8')
        self.audit_file = (output/'audits.jsonl').open('x', encoding='utf-8')
        manifest = {'schema_version':1, 'started_wall':time.time(), 'python':platform.python_version(),
                    'metadata':rospy.get_param('~metadata', {}),
                    'task_parameters':rospy.get_param('/color_sorting_task', {}),
                    'use_sim_time':rospy.get_param('/use_sim_time', False),
                    'sha256':{}, 'git':{}}
        for key in ('robot_description','robot_description_semantic','robot_description_planning'):
            value = rospy.get_param('/'+key, None)
            (output/(key+'.json')).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
        for path in rospy.get_param('~hash_files', []):
            manifest['sha256'][path] = sha256_file(os.path.expanduser(path))
        for repo in rospy.get_param('~repositories', []):
            revision = subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'], universal_newlines=True).strip()
            dirty = subprocess.check_output(['git','-C',repo,'status','--porcelain','--untracked-files=no'], universal_newlines=True).strip()
            manifest['git'][repo] = {'revision':revision, 'tracked_changes':dirty}
        (output/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
        self.camera_saved = False
        self.subscribers = [
            rospy.Subscriber('/sorting/experiment_events', String, self.event, queue_size=200),
            rospy.Subscriber('/sorting/experiment_trajectory', DisplayTrajectory, self.trajectory, queue_size=20),
            rospy.Subscriber('/sorting/experiment_audit', ExperimentAudit, self.audit, queue_size=5),
            rospy.Subscriber(rospy.get_param('~detections_topic','/sorting/detections'), DetectedObjectArray, self.detections, queue_size=20),
            rospy.Subscriber(rospy.get_param('~feedback_topic','/aubo_i5/aubo_i5_controller/follow_joint_trajectory/feedback'),
                             FollowJointTrajectoryActionFeedback, self.feedback, queue_size=200),
            rospy.Subscriber(rospy.get_param('~camera_info_topic','/workspace_camera/color/camera_info'), CameraInfo, self.camera_info, queue_size=1)]
        rospy.on_shutdown(self.close)
        rospy.loginfo('Passive experiment recording to %s', output)

    def write(self, record, target=None):
        record['received_wall'] = time.time()
        with self.lock:
            f = target or self.file
            if not f.closed:
                f.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n'); f.flush()

    def event(self, msg):
        try: self.write(json.loads(msg.data))
        except (ValueError, TypeError) as error: rospy.logerr('Invalid experiment event: %s', error)

    def trajectory(self, msg):
        self.write({'kind':'planned_trajectory', 'ros_stamp':rospy.Time.now().to_sec(),
                    'message':to_dict(msg)}, self.trajectory_file)

    def detections(self, msg):
        self.write({'kind':'detections', 'source_stamp':msg.header.stamp.to_sec(),
                    'observation_valid':msg.observation_valid, 'frame_id':msg.header.frame_id,
                    'objects':[{'class_name':o.class_name or o.color, 'confidence':o.confidence,
                                'xyz':[o.pose.position.x,o.pose.position.y,o.pose.position.z],
                                'yaw_rad':o.grasp_angle, 'width_m':o.grasp_width,
                                'depth_valid':o.depth_valid, 'geometry_valid':o.grasp_geometry_valid} for o in msg.objects]})

    def audit(self, msg):
        record = to_dict(msg); record['kind'] = 'trajectory_audit'
        self.write(record, self.audit_file)

    def feedback(self, msg):
        if len(msg.feedback.joint_names) != len(msg.feedback.error.positions):
            rospy.logwarn_throttle(5,'Skipping malformed/missing joint error feedback')
            return
        self.write({'kind':'joint_feedback','goal_id':msg.status.goal_id.id,
                    'source_stamp':msg.feedback.header.stamp.to_sec(),
                    'joint_names':list(msg.feedback.joint_names),'error_positions':list(msg.feedback.error.positions)})

    def camera_info(self, msg):
        with self.lock:
            if self.camera_saved: return
            (self.output/'camera_info.json').write_text(json.dumps(
                to_dict(msg), indent=2), encoding='utf-8')
            self.camera_saved = True

    def close(self):
        with self.lock:
            self.file.close(); self.trajectory_file.close()
            self.audit_file.close()

if __name__ == '__main__':
    rospy.init_node('thesis_experiment_recorder'); Recorder(); rospy.spin()
