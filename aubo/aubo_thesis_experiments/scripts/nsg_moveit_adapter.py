#!/usr/bin/env python3
"""Adapt the real motion_planning/GetNeuralPlan service to MoveIt GetMotionPlan.

This node plans and audits only. The sorting task retains responsibility for
retiming, a second fresh-scene audit, operator start and execution.
"""
import copy
import json
import math
import threading
import time
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanResponse, GetPositionIK, GetPositionIKRequest
from motion_planning.srv import GetNeuralPlan, GetNeuralPlanRequest
from aubo_thesis_experiments.srv import SnapshotScene, AuditTrajectory, AuditTrajectoryRequest
from aubo_thesis_experiments.ros_codec import to_dict

class Adapter:
    def __init__(self):
        self.lock = threading.Lock()
        self.base_frame = rospy.get_param('~base_frame','base_link')
        self.output = Path(rospy.get_param('~output_dir')).expanduser()
        self.output.mkdir(parents=True, exist_ok=False)
        self.neural = rospy.ServiceProxy('/compute_neural_path', GetNeuralPlan)
        self.snapshot = rospy.ServiceProxy('/thesis_experiment/snapshot_scene', SnapshotScene)
        self.auditor = rospy.ServiceProxy('/thesis_experiment/audit_trajectory', AuditTrajectory)
        self.ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        for service in ('/compute_neural_path','/thesis_experiment/snapshot_scene','/thesis_experiment/audit_trajectory','/compute_ik'):
            rospy.wait_for_service(service, timeout=rospy.get_param('~startup_timeout',120.0))
        self.metadata = rospy.get_param('/compute_neural_path/metadata')
        if not self.metadata.get('strict_runtime') or not self.metadata.get('checkpoint_sha256') or self.metadata.get('collision_engine') == 'NullCollisionEngine':
            raise RuntimeError('real trained NSG and active collision engine are required')
        self.names = self.metadata['joint_names']
        if len(self.names) != 6 or len(set(self.names)) != 6:
            raise RuntimeError('expected six unique NSG joint names')
        self.server = rospy.Service('/thesis_experiment/nsg_plan', GetMotionPlan, self.plan)

    def goal(self, req, start):
        for constraints in req.goal_constraints:
            joints = {c.joint_name:c.position for c in constraints.joint_constraints}
            if set(joints) == set(self.names):
                return [joints[n] for n in self.names]
            if len(constraints.position_constraints) != 1 or len(constraints.orientation_constraints) != 1:
                continue
            position, orientation = constraints.position_constraints[0], constraints.orientation_constraints[0]
            poses = position.constraint_region.primitive_poses
            offset = position.target_point_offset
            if len(poses) != 1 or any(abs(v)>1e-9 for v in (offset.x,offset.y,offset.z)) or position.link_name != orientation.link_name or position.header.frame_id != orientation.header.frame_id:
                continue
            ik_request = GetPositionIKRequest()
            ik_request.ik_request.group_name = req.group_name
            ik_request.ik_request.ik_link_name = position.link_name
            ik_request.ik_request.robot_state = start
            ik_request.ik_request.avoid_collisions = True
            ik_request.ik_request.constraints = req.path_constraints
            pose = PoseStamped(); pose.header = position.header
            pose.pose.position = poses[0].position; pose.pose.orientation = orientation.orientation
            ik_request.ik_request.pose_stamped = pose
            ik_request.ik_request.timeout = rospy.Duration(1.0)
            result = self.ik(ik_request)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                values = dict(zip(result.solution.joint_state.name,result.solution.joint_state.position))
                return [values[n] for n in self.names]
        raise ValueError('no supported reachable joint/pose goal; goal constraints are never silently omitted')

    def plan(self, request):
        with self.lock:
            return self.plan_serial(request)

    def plan_serial(self, request):
        started = time.monotonic()
        result = GetMotionPlanResponse()
        result.motion_plan_response.error_code.val = MoveItErrorCodes.PLANNING_FAILED
        record = {'request':to_dict(request), 'neural_metadata':self.metadata, 'started_wall':time.time()}
        try:
            if rospy.get_param('/compute_neural_path/metadata') != self.metadata:
                raise ValueError('neural backend metadata changed; restart this experiment adapter')
            req = request.motion_plan_request
            if req.trajectory_constraints.constraints:
                raise ValueError('trajectory constraints unsupported by NSG adapter')
            frozen = self.snapshot(rospy.get_param('~max_obstacles',10000))
            if not frozen.success: raise ValueError(frozen.reason)
            if frozen.frame_id.lstrip('/') != self.base_frame.lstrip('/'):
                raise ValueError('planning frame differs from NSG FK base frame')
            record['scene'] = to_dict(frozen.scene)
            start = copy.deepcopy(frozen.scene.robot_state)
            js = req.start_state.joint_state
            if len(js.name) != len(js.position) or len(set(js.name)) != len(js.name):
                raise ValueError('malformed request start joints')
            values = dict(zip(start.joint_state.name,start.joint_state.position))
            values.update(zip(js.name,js.position))
            start.joint_state.name = list(values); start.joint_state.position = [values[n] for n in values]
            start.joint_state.velocity = []; start.joint_state.effort = []
            start.is_diff = False
            # Attached bodies come from the frozen scene, never a stale empty request.
            neural_req = GetNeuralPlanRequest()
            neural_req.start_joint = [values[n] for n in self.names]
            neural_req.goal_joint = self.goal(req,start)
            for q in (neural_req.start_joint,neural_req.goal_joint):
                if not all(math.isfinite(v) for v in q): raise ValueError('nonfinite joint target')
                if any(v < lo or v > hi for v,lo,hi in zip(q,self.metadata['joint_limits_low'],self.metadata['joint_limits_high'])):
                    raise ValueError('joint target exceeds live robot limits')
            # World AABBs include occupied OctoMap leaves. Do not also supply
            # collision objects in planning_scene, which would duplicate them.
            neural_req.obs_flattened = frozen.obs_flattened
            planned = self.neural(neural_req)
            if not planned.success: raise ValueError('NSG did not find a valid candidate')
            if set(planned.trajectory.joint_names) != set(self.names):
                raise ValueError('NSG trajectory joint names differ from robot')
            audit_req = AuditTrajectoryRequest()
            audit_req.start_state = start; audit_req.use_provided_scene = True; audit_req.scene = frozen.scene
            audit_req.trajectory.joint_trajectory = planned.trajectory
            audit_req.group_name = req.group_name
            audit_req.path_constraints = req.path_constraints; audit_req.goal_constraints = req.goal_constraints
            audit_req.max_joint_step = rospy.get_param('~max_joint_step',.02)
            # Keep the complete audit input at the top level so this file can
            # be replayed by audit_trajectory.py without reconstructing state.
            record.update(to_dict(audit_req))
            audited = self.auditor(audit_req)
            record['audit'] = to_dict(audited); record['trajectory'] = to_dict(audit_req.trajectory)
            if not audited.valid: raise ValueError('full MoveIt audit rejected candidate: '+audited.reason)
            result.motion_plan_response.trajectory_start = start
            result.motion_plan_response.group_name = req.group_name
            result.motion_plan_response.trajectory = audit_req.trajectory
            result.motion_plan_response.error_code.val = MoveItErrorCodes.SUCCESS
        except Exception as error:
            record['error'] = str(error)
            rospy.logerr('NSG adapter refused plan: %s',error)
        result.motion_plan_response.planning_time = time.monotonic()-started
        record['response'] = to_dict(result)
        path = self.output/('request_%s.json'%int(time.time()*1e9))
        path.write_text(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        return result

if __name__ == '__main__':
    rospy.init_node('thesis_nsg_adapter'); Adapter(); rospy.spin()
