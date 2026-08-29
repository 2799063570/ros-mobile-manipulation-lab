#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import copy
import math
import sys
import threading
import time
import xml.etree.ElementTree as ElementTree

import actionlib
import moveit_commander
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTolerance,
)
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint

from aubo_perception.msg import DetectedObjectArray


class ColorSortingTask(object):
    """Service-controlled camera observation and color sorting state machine."""

    def __init__(self):
        self.group_name = rospy.get_param("~planning_group", "aubo_i5")
        self.end_effector_link = rospy.get_param("~end_effector_link", "tcp_link")
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.detections_topic = rospy.get_param(
            "~detections_topic", "/sorting/detections"
        )
        self.gripper_action_name = rospy.get_param(
            "~gripper_action", "/gripper_controller/follow_joint_trajectory"
        )

        self.table_z = float(rospy.get_param("~table_z", 0.14))
        self.table_frame = rospy.get_param("~table_frame", self.target_frame)
        self.table_center = rospy.get_param("~table_center", [0.80, 0.0, -0.06])
        self.table_size = rospy.get_param("~table_size", [0.80, 1.20, 0.40])
        self.table_collision_margin = max(
            0.0, float(rospy.get_param("~table_collision_margin", 0.0))
        )
        self.object_height = float(rospy.get_param("~object_height", 0.04))
        self.grasp_height_offset = float(
            rospy.get_param("~grasp_height_offset", 0.01)
        )
        self.grasp_rpy = rospy.get_param("~grasp_rpy", [math.pi, 0.0, 0.0])
        self.observation_pose = rospy.get_param(
            "~observation_pose", [0.58, 0.0, 0.62]
        )
        self.observation_named_target = rospy.get_param(
            "~observation_named_target", ""
        )
        self.work_ready_named_target = rospy.get_param(
            "~work_ready_named_target", "work_ready"
        )
        self.pregrasp_height = float(rospy.get_param("~pregrasp_height", 0.25))
        self.lift_height = float(rospy.get_param("~lift_height", 0.30))
        self.place_clearance = float(rospy.get_param("~place_clearance", 0.02))
        self.cartesian_step = float(rospy.get_param("~cartesian_step", 0.01))
        self.minimum_cartesian_fraction = float(
            rospy.get_param("~minimum_cartesian_fraction", 0.90)
        )

        self.gripper_open = float(rospy.get_param("~gripper_open", 0.0))
        self.gripper_closed = float(rospy.get_param("~gripper_closed", 0.28))
        self.gripper_motion_time = float(
            rospy.get_param("~gripper_motion_time", 2.5)
        )
        self.gripper_contact_tolerance = float(
            rospy.get_param("~gripper_contact_tolerance", 0.30)
        )
        self.use_grasp_attachment = bool(
            rospy.get_param("~use_grasp_attachment", True)
        )
        self.grasp_attach_topic = rospy.get_param(
            "~grasp_attach_topic", "/sorting/grasp/attach"
        )
        self.grasp_detach_topic = rospy.get_param(
            "~grasp_detach_topic", "/sorting/grasp/detach"
        )
        self.grasp_status_topic = rospy.get_param(
            "~grasp_status_topic", "/sorting/grasp/status"
        )
        self.grasp_attachment_timeout = float(
            rospy.get_param("~grasp_attachment_timeout", 3.0)
        )
        self.sort_colors = rospy.get_param("~sort_colors", ["red", "green", "blue"])
        self.place_frame = rospy.get_param("~place_frame", self.target_frame)
        self.place_targets = rospy.get_param("~place_targets")
        self.detection_timeout = float(rospy.get_param("~detection_timeout", 15.0))
        self.detection_samples = max(
            1, int(rospy.get_param("~detection_samples", 8))
        )
        self.detection_settle_time = float(
            rospy.get_param("~detection_settle_time", 1.0)
        )
        self.verify_observation_detections = bool(
            rospy.get_param("~verify_observation_detections", False)
        )
        self.observation_verification_timeout = float(
            rospy.get_param("~observation_verification_timeout", 4.0)
        )
        self.grasp_offset_x = float(rospy.get_param("~grasp_offset_x", 0.0))
        self.grasp_offset_y = float(rospy.get_param("~grasp_offset_y", 0.0))
        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", 0.15))
        self.acceleration_scaling = float(
            rospy.get_param("~acceleration_scaling", 0.15)
        )
        self.gripper_server_timeout = float(
            rospy.get_param("~gripper_server_timeout", 30.0)
        )
        self.finish_named_target = rospy.get_param("~finish_named_target", "down")
        self.scene_update_timeout = float(
            rospy.get_param("~scene_update_timeout", 10.0)
        )
        self.auto_move_to_observation = bool(
            rospy.get_param("~auto_move_to_observation", True)
        )
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.require_octomap = bool(rospy.get_param("~require_octomap", False))
        self.point_cloud_topic = rospy.get_param(
            "~point_cloud_topic", "/workspace_camera/depth/color/points"
        )
        self.planning_scene_topic = rospy.get_param(
            "~planning_scene_topic", "/move_group/monitored_planning_scene"
        )
        self.clear_octomap_service = rospy.get_param(
            "~clear_octomap_service", "/clear_octomap"
        )
        self.octomap_wait_timeout = float(
            rospy.get_param("~octomap_wait_timeout", 30.0)
        )
        self.base_lock_topic = rospy.get_param(
            "~base_lock_topic", "/sorting/base_locked"
        )
        self._robot_limits_valid = self._verify_loaded_upper_arm_limit()

        self._detections = None
        self._detections_wall_time = 0.0
        self._grasp_status = ""
        self._grasp_status_sequence = 0
        self._attached_model = ""
        self._data_lock = threading.Lock()
        self._operation_lock = threading.Lock()
        self._busy = True
        self._initialized = False
        self._observation_ready = False
        self._stop_requested = threading.Event()
        self._last_cloud_wall_time = 0.0
        self._cloud_points = 0
        self._octomap_sequence = 0

        self._state_publisher = rospy.Publisher(
            "/sorting/state", String, queue_size=1, latch=True
        )
        self._detection_summary_publisher = rospy.Publisher(
            "/sorting/detection_summary", String, queue_size=1, latch=True
        )
        self._base_lock_publisher = rospy.Publisher(
            self.base_lock_topic, Bool, queue_size=1, latch=True
        )
        self._base_lock_publisher.publish(Bool(data=False))
        self._subscriber = rospy.Subscriber(
            self.detections_topic,
            DetectedObjectArray,
            self._detection_callback,
            queue_size=2,
        )
        self._grasp_attach_publisher = rospy.Publisher(
            self.grasp_attach_topic, String, queue_size=1
        )
        self._grasp_detach_publisher = rospy.Publisher(
            self.grasp_detach_topic, String, queue_size=1
        )
        self._grasp_status_subscriber = rospy.Subscriber(
            self.grasp_status_topic,
            String,
            self._grasp_status_callback,
            queue_size=5,
        )
        self._cloud_subscriber = rospy.Subscriber(
            self.point_cloud_topic, PointCloud2, self._cloud_callback, queue_size=1
        )
        self._planning_scene_subscriber = rospy.Subscriber(
            self.planning_scene_topic,
            PlanningScene,
            self._planning_scene_callback,
            queue_size=5,
        )
        self._clear_octomap = rospy.ServiceProxy(self.clear_octomap_service, Empty)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander(self.group_name)
        self.arm.set_end_effector_link(self.end_effector_link)
        self.arm.set_pose_reference_frame(self.target_frame)
        self.planning_frame = self.arm.get_planning_frame()
        self.tf_listener = tf.TransformListener()
        self.arm.set_planning_time(float(rospy.get_param("~planning_time", 12.0)))
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.arm.set_max_acceleration_scaling_factor(self.acceleration_scaling)

        self.gripper_client = actionlib.SimpleActionClient(
            self.gripper_action_name, FollowJointTrajectoryAction
        )

        self._services = [
            rospy.Service("/sorting/move_to_observation", Trigger, self._observe_service),
            rospy.Service("/sorting/start", Trigger, self._start_service),
            rospy.Service("/sorting/stop", Trigger, self._stop_service),
            rospy.Service("/sorting/open_gripper", Trigger, self._open_service),
            rospy.Service("/sorting/prepare_work", Trigger, self._prepare_work_service),
            rospy.Service("/sorting/home", Trigger, self._home_service),
        ]
        self._publish_state("INITIALIZING", "waiting for Gazebo controllers")

    def start(self):
        worker = threading.Thread(target=self._initialize)
        worker.daemon = True
        worker.start()

    def _publish_state(self, state, detail=""):
        message = state if not detail else "{} | {}".format(state, detail)
        self._state_publisher.publish(String(data=message))
        rospy.loginfo("Sorting state: %s", message)

    def _detection_callback(self, message):
        with self._data_lock:
            self._detections = message
            self._detections_wall_time = time.time()
        counts = []
        for color in self.sort_colors:
            count = sum(1 for item in message.objects if item.color == color)
            counts.append("{}:{}".format(color, count))
        self._detection_summary_publisher.publish(String(data="  ".join(counts)))

    def _grasp_status_callback(self, message):
        with self._data_lock:
            self._grasp_status = message.data
            self._grasp_status_sequence += 1

    def _cloud_callback(self, message):
        points = int(message.width) * int(message.height)
        if points <= 0:
            return
        with self._data_lock:
            self._cloud_points = points
            self._last_cloud_wall_time = time.time()

    def _planning_scene_callback(self, message):
        if not message.world.octomap.octomap.data:
            return
        with self._data_lock:
            self._octomap_sequence += 1

    def _refresh_octomap(self):
        """Clear stale chassis-frame geometry and require a fresh sensor map."""
        if not self.require_octomap:
            return True
        deadline = time.time() + self.octomap_wait_timeout
        cloud_not_before = time.time()
        while not rospy.is_shutdown() and time.time() < deadline:
            with self._data_lock:
                cloud_time = self._last_cloud_wall_time
                cloud_points = self._cloud_points
            if cloud_time > cloud_not_before and cloud_points > 0:
                break
            if self._stop_requested.is_set():
                return False
            rospy.sleep(0.05)
        else:
            rospy.logerr("No fresh RGB-D cloud received on %s", self.point_cloud_topic)
            return False

        remaining = max(0.1, deadline - time.time())
        try:
            rospy.wait_for_service(self.clear_octomap_service, timeout=remaining)
            with self._data_lock:
                previous_sequence = self._octomap_sequence
            self._clear_octomap()
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr("Cannot clear MoveIt OctoMap: %s", str(error))
            return False

        while not rospy.is_shutdown() and time.time() < deadline:
            with self._data_lock:
                refreshed = self._octomap_sequence > previous_sequence
            if refreshed:
                rospy.loginfo(
                    "Fresh MoveIt OctoMap confirmed from %s (%d points/cloud)",
                    self.point_cloud_topic,
                    cloud_points,
                )
                return True
            if self._stop_requested.is_set():
                return False
            rospy.sleep(0.05)
        rospy.logerr("MoveIt did not publish a fresh non-empty OctoMap")
        return False

    def _wait_for_grasp_plugin(self):
        if not self.use_grasp_attachment:
            return True
        deadline = time.time() + self.grasp_attachment_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self._data_lock:
                status = self._grasp_status
            if status == "ready" or status.startswith("detached:"):
                rospy.loginfo("Gazebo grasp attachment plugin is ready")
                return True
            if status.startswith("error:"):
                rospy.logerr("Gazebo grasp plugin reported: %s", status)
                return False
            rospy.sleep(0.05)
        rospy.logerr(
            "No status received from Gazebo grasp plugin on %s",
            self.grasp_status_topic,
        )
        return False

    def _set_grasp_attachment(self, model_name, attach):
        if not self.use_grasp_attachment:
            return True
        expected = ("attached:" if attach else "detached:") + model_name
        publisher = (
            self._grasp_attach_publisher if attach else self._grasp_detach_publisher
        )
        with self._data_lock:
            initial_sequence = self._grasp_status_sequence
        publisher.publish(String(data=model_name))
        deadline = time.time() + self.grasp_attachment_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self._data_lock:
                status = self._grasp_status
                sequence = self._grasp_status_sequence
            if sequence > initial_sequence and status == expected:
                self._attached_model = model_name if attach else ""
                rospy.loginfo("Gazebo grasp status: %s", status)
                return True
            if sequence > initial_sequence and status.startswith("error:"):
                rospy.logerr("Gazebo grasp plugin reported: %s", status)
                return False
            rospy.sleep(0.02)
        rospy.logerr("Timed out waiting for Gazebo grasp status '%s'", expected)
        return False

    def _release_attached_object_no_wait(self):
        if self.use_grasp_attachment and self._attached_model:
            self._grasp_detach_publisher.publish(String(data=self._attached_model))
            self._attached_model = ""

    def _verify_loaded_upper_arm_limit(self):
        """Confirm that Gazebo and MoveIt received the second-axis limit."""
        expected_lower = -1.0471976
        expected_upper = 1.0471976
        tolerance = 1.0e-5
        try:
            description = rospy.get_param("/robot_description")
            root = ElementTree.fromstring(description)
            upper_arm = next(
                joint
                for joint in root.findall("joint")
                if joint.get("name") == "upperArm_joint"
            )
            limit = upper_arm.find("limit")
            urdf_lower = float(limit.get("lower"))
            urdf_upper = float(limit.get("upper"))
            planning_ns = (
                "/robot_description_planning/joint_limits/upperArm_joint/"
            )
            moveit_lower = float(rospy.get_param(planning_ns + "min_position"))
            moveit_upper = float(rospy.get_param(planning_ns + "max_position"))
        except (KeyError, StopIteration, TypeError, ValueError, ElementTree.ParseError) as error:
            rospy.logerr("Unable to verify upperArm_joint limits: %s", str(error))
            return False

        rospy.loginfo(
            "Loaded upperArm_joint limits: URDF [%.6f, %.6f], MoveIt [%.6f, %.6f] rad",
            urdf_lower,
            urdf_upper,
            moveit_lower,
            moveit_upper,
        )
        values = (urdf_lower, urdf_upper, moveit_lower, moveit_upper)
        expected = (expected_lower, expected_upper, expected_lower, expected_upper)
        if any(abs(value - target) > tolerance for value, target in zip(values, expected)):
            rospy.logerr(
                "Stale robot model detected; expected upperArm_joint limits "
                "[-1.047198, 1.047198] rad (-60 ... 60 deg)"
            )
            return False
        return True

    def _initialize(self):
        if not self._robot_limits_valid:
            self._busy = False
            self._publish_state("ERROR", "loaded upperArm_joint limit is not +/-60 deg")
            return
        if not self._wait_for_grasp_plugin():
            self._busy = False
            self._publish_state("ERROR", "Gazebo grasp plugin unavailable")
            return
        self._publish_state("INITIALIZING", "waiting for gripper action")
        if not self.gripper_client.wait_for_server(
            rospy.Duration(self.gripper_server_timeout)
        ):
            self._busy = False
            self._publish_state("ERROR", "gripper action server unavailable")
            return

        if not self._add_table_collision():
            self._busy = False
            self._publish_state("ERROR", "sorting table missing from planning scene")
            return
        if not self._refresh_octomap():
            self._busy = False
            self._publish_state("ERROR", "RGB-D cloud or MoveIt OctoMap unavailable")
            return
        self._initialized = True
        self._busy = False
        self._publish_state("IDLE", "controllers ready")

        if self.auto_move_to_observation:
            self._start_operation("OBSERVING", self._initial_observation_operation)
        elif self.auto_start:
            self._start_operation("SORTING", self._sorting_operation)

    def _start_operation(self, state, operation):
        with self._operation_lock:
            if self._busy:
                return False, "another operation is running"
            if not self._initialized:
                return False, "sorting node is not initialized"
            self._busy = True
            self._stop_requested.clear()
            self._base_lock_publisher.publish(Bool(data=True))

        def worker():
            success = False
            self._publish_state(state)
            try:
                success = bool(operation())
            except Exception as error:
                rospy.logerr("Sorting operation failed: %s", str(error))
                self._publish_state("ERROR", str(error))
            finally:
                if not success:
                    self._release_attached_object_no_wait()
                with self._operation_lock:
                    self._busy = False
                self._base_lock_publisher.publish(Bool(data=False))
                if self._stop_requested.is_set():
                    self._observation_ready = False
                    self._publish_state("STOPPED", "operation cancelled")
                elif success:
                    self._publish_state("READY", "waiting for panel command")
                else:
                    self._publish_state("ERROR", "operation failed")

        thread = threading.Thread(target=worker)
        thread.daemon = True
        thread.start()
        return True, "command accepted"

    def _observe_service(self, _request):
        success, message = self._start_operation(
            "OBSERVING", self._observation_operation
        )
        return TriggerResponse(success=success, message=message)

    def _start_service(self, _request):
        if not self._observation_ready:
            return TriggerResponse(
                success=False,
                message="move to observation pose and confirm detections first",
            )
        success, message = self._start_operation("SORTING", self._sorting_operation)
        return TriggerResponse(success=success, message=message)

    def _stop_service(self, _request):
        self._stop_requested.set()
        self.gripper_client.cancel_all_goals()
        self.arm.stop()
        self._release_attached_object_no_wait()
        return TriggerResponse(success=True, message="stop requested")

    def _open_service(self, _request):
        success, message = self._start_operation("OPENING", self._open_operation)
        return TriggerResponse(success=success, message=message)

    def _prepare_work_service(self, _request):
        success, message = self._start_operation(
            "PREPARING", self._prepare_work_operation
        )
        return TriggerResponse(success=success, message=message)

    def _home_service(self, _request):
        success, message = self._start_operation("HOMING", self._home_operation)
        return TriggerResponse(success=success, message=message)

    def _pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        quaternion = quaternion_from_euler(
            float(self.grasp_rpy[0]),
            float(self.grasp_rpy[1]),
            float(self.grasp_rpy[2]),
        )
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def _xy_in_target_frame(self, source_frame, xy):
        if source_frame == self.target_frame:
            return float(xy[0]), float(xy[1])
        source_pose = PoseStamped()
        source_pose.header.frame_id = source_frame
        source_pose.header.stamp = rospy.Time(0)
        source_pose.pose.position.x = float(xy[0])
        source_pose.pose.position.y = float(xy[1])
        source_pose.pose.orientation.w = 1.0
        try:
            self.tf_listener.waitForTransform(
                self.target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(self.scene_update_timeout),
            )
            target_pose = self.tf_listener.transformPose(
                self.target_frame, source_pose
            )
            return target_pose.pose.position.x, target_pose.pose.position.y
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as error:
            rospy.logerr(
                "Cannot transform place target from %s to %s: %s",
                source_frame,
                self.target_frame,
                str(error),
            )
            return None

    def _move_to_pose(self, pose, description):
        if self._stop_requested.is_set():
            return False
        rospy.loginfo("Planning arm to %s", description)
        self.arm.set_pose_target(pose, self.end_effector_link)
        success = bool(self.arm.go(wait=True))
        self.arm.stop()
        self.arm.clear_pose_targets()
        if not success:
            rospy.logerr("MoveIt failed to reach %s", description)
        return success and not self._stop_requested.is_set()

    def _move_named(self, target):
        if not target:
            return True
        if self._stop_requested.is_set():
            return False
        if target not in self.arm.get_named_targets():
            rospy.logerr("Unknown arm named target '%s'", target)
            return False
        rospy.loginfo("Moving arm to named target %s", target)
        self.arm.set_named_target(target)
        success = bool(self.arm.go(wait=True))
        self.arm.stop()
        return success and not self._stop_requested.is_set()

    def _cartesian_to(self, target_pose, description):
        if self._stop_requested.is_set():
            return False
        waypoint = copy.deepcopy(target_pose.pose)
        plan, fraction = self.arm.compute_cartesian_path(
            [waypoint], self.cartesian_step, 0.0, True
        )
        rospy.loginfo("Cartesian path to %s: %.1f%%", description, 100.0 * fraction)
        if fraction < self.minimum_cartesian_fraction:
            rospy.logwarn("Cartesian fraction too low; falling back to pose planning")
            return self._move_to_pose(target_pose, description)
        # Older MoveIt releases can return Cartesian trajectories whose timing
        # does not honour the scaling factors set on MoveGroupCommander.  The
        # resulting fast descent may reach the final waypoint but fail the
        # ros_control goal tolerance before the joints have settled.  Retime
        # explicitly so Cartesian grasp/lift motions use the configured speed.
        try:
            plan = self.arm.retime_trajectory(
                self.robot.get_current_state(),
                plan,
                self.velocity_scaling,
                self.acceleration_scaling,
            )
        except Exception as error:
            rospy.logwarn(
                "Could not retime Cartesian path with acceleration scaling: %s; "
                "retrying with velocity scaling only",
                str(error),
            )
            try:
                plan = self.arm.retime_trajectory(
                    self.robot.get_current_state(), plan, self.velocity_scaling
                )
            except Exception as fallback_error:
                rospy.logerr(
                    "Could not safely retime Cartesian path to %s: %s",
                    description,
                    str(fallback_error),
                )
                return False
        if not plan.joint_trajectory.points:
            rospy.logerr("Retimed Cartesian path to %s is empty", description)
            return False
        rospy.loginfo(
            "Retimed Cartesian path to %s: %.2f s at velocity scale %.2f",
            description,
            plan.joint_trajectory.points[-1].time_from_start.to_sec(),
            self.velocity_scaling,
        )
        success = bool(self.arm.execute(plan, wait=True))
        self.arm.stop()
        if not success:
            rospy.logerr("Failed to execute Cartesian path to %s", description)
        return success and not self._stop_requested.is_set()

    def _command_gripper(self, position):
        if self._stop_requested.is_set():
            return False
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["joint1", "joint2"]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = rospy.Duration(self.gripper_motion_time)
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        # Closing on a rigid object is expected to leave a position error: the
        # fingers must stop at contact instead of numerically reaching the
        # preload angle.  Send the tolerance with every goal so this also works
        # when a stale controller YAML is still present on the parameter server.
        closing = position > self.gripper_open + 1.0e-4
        position_tolerance = self.gripper_contact_tolerance if closing else 0.05
        for joint_name in goal.trajectory.joint_names:
            path_tolerance = JointTolerance()
            path_tolerance.name = joint_name
            # A contact-closing motion cannot follow a free-space trajectory
            # after the fingers touch the object.  In control_msgs, -1 erases
            # the controller's default tolerance for this goal.
            path_tolerance.position = -1.0 if closing else position_tolerance
            path_tolerance.velocity = -1.0 if closing else 0.0
            path_tolerance.acceleration = -1.0 if closing else 0.0
            goal.path_tolerance.append(path_tolerance)
            goal_tolerance = JointTolerance()
            goal_tolerance.name = joint_name
            goal_tolerance.position = position_tolerance
            goal_tolerance.velocity = -1.0 if closing else 0.0
            goal_tolerance.acceleration = -1.0 if closing else 0.0
            goal.goal_tolerance.append(goal_tolerance)
        goal.goal_time_tolerance = rospy.Duration(3.0)

        self.gripper_client.send_goal(goal)
        if not self.gripper_client.wait_for_result(
            rospy.Duration(self.gripper_motion_time + 3.0)
        ):
            self.gripper_client.cancel_goal()
            rospy.logerr("Gripper command timed out")
            return False
        state = self.gripper_client.get_state()
        result = self.gripper_client.get_result()
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr(
                "Gripper action failed: state=%d, error_code=%s, error_string='%s'",
                state,
                getattr(result, "error_code", "unavailable"),
                getattr(result, "error_string", ""),
            )
            return False
        return not self._stop_requested.is_set()

    def _add_table_collision(self):
        object_name = "sorting_table"
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.table_frame
        table_pose.header.stamp = rospy.Time(0)
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = float(self.table_center[0])
        table_pose.pose.position.y = float(self.table_center[1])
        table_pose.pose.position.z = float(self.table_center[2])

        try:
            if self.table_frame != self.planning_frame:
                self.tf_listener.waitForTransform(
                    self.planning_frame,
                    self.table_frame,
                    rospy.Time(0),
                    rospy.Duration(self.scene_update_timeout),
                )
                table_pose = self.tf_listener.transformPose(
                    self.planning_frame, table_pose
                )
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as error:
            rospy.logerr(
                "Cannot transform sorting table from %s to %s: %s",
                self.table_frame,
                self.planning_frame,
                str(error),
            )
            return False

        # Recreate the object so an existing entry cannot make the update wait
        # return before the new pose has reached move_group.
        if object_name in self.scene.get_known_object_names():
            self.scene.remove_world_object(object_name)
            removal_deadline = time.time() + self.scene_update_timeout
            while not rospy.is_shutdown() and time.time() < removal_deadline:
                if object_name not in self.scene.get_known_object_names():
                    break
                rospy.sleep(0.05)
            if object_name in self.scene.get_known_object_names():
                rospy.logerr("MoveIt did not remove stale sorting table")
                return False

        padded_size = [float(value) for value in self.table_size]
        padded_size[0] += 2.0 * self.table_collision_margin
        padded_size[1] += 2.0 * self.table_collision_margin
        self.scene.add_box(
            object_name,
            table_pose,
            size=tuple(padded_size),
        )
        deadline = time.time() + self.scene_update_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if object_name in self.scene.get_known_object_names():
                self.arm.set_support_surface_name(object_name)
                rospy.loginfo(
                    "Sorting table confirmed in MoveIt planning scene: frame=%s, "
                    "position=[%.3f, %.3f, %.3f], size=[%.3f, %.3f, %.3f]",
                    table_pose.header.frame_id,
                    table_pose.pose.position.x,
                    table_pose.pose.position.y,
                    table_pose.pose.position.z,
                    padded_size[0],
                    padded_size[1],
                    padded_size[2],
                )
                return True
            rospy.sleep(0.1)
        rospy.logerr(
            "MoveIt planning scene did not acknowledge '%s' within %.1f seconds",
            object_name,
            self.scene_update_timeout,
        )
        return False

    def _wait_for_object(self, color, not_before):
        deadline = time.time() + self.detection_timeout
        samples = []
        last_receipt_time = not_before
        while not rospy.is_shutdown() and time.time() < deadline:
            if self._stop_requested.is_set():
                return None
            with self._data_lock:
                detections = self._detections
                receipt_time = self._detections_wall_time
            if detections is not None and receipt_time > last_receipt_time:
                last_receipt_time = receipt_time
                candidates = [item for item in detections.objects if item.color == color]
                if candidates:
                    samples.append(max(candidates, key=lambda item: item.contour_area))
                    if len(samples) >= self.detection_samples:
                        detected = copy.deepcopy(samples[-1])
                        detected.pose.position.x = sum(
                            item.pose.position.x for item in samples
                        ) / float(len(samples))
                        detected.pose.position.y = sum(
                            item.pose.position.y for item in samples
                        ) / float(len(samples))
                        detected.pose.position.z = sum(
                            item.pose.position.z for item in samples
                        ) / float(len(samples))
                        rospy.loginfo(
                            "Averaged %d '%s' detections at [%.3f, %.3f]",
                            len(samples),
                            color,
                            detected.pose.position.x,
                            detected.pose.position.y,
                        )
                        return detected
            rospy.sleep(0.1)
        rospy.logerr(
            "No fresh '%s' object detected within %.1f seconds",
            color,
            self.detection_timeout,
        )
        return None

    def _observation(self):
        self._observation_ready = False
        if not self._refresh_octomap():
            return False
        # The mobile base has moved since initialization. Refresh the static
        # table in the planning frame before planning out of transport pose.
        if not self._add_table_collision():
            return False
        if self.observation_named_target:
            success = self._move_named(self.observation_named_target)
        else:
            success = self._move_to_pose(
                self._pose(*self.observation_pose), "camera observation pose"
            )
        if not success:
            return False
        rospy.sleep(self.detection_settle_time)
        self._observation_ready = True
        return True

    def _observation_operation(self):
        if not self._observation():
            return False
        if self.verify_observation_detections:
            if not self._verify_visible_colors():
                self._observation_ready = False
                return False
        return True

    def _initial_observation_operation(self):
        if not self._observation_operation():
            return False
        if self.auto_start:
            return self._sorting_operation()
        return True

    def _open_operation(self):
        if not self._command_gripper(self.gripper_open):
            return False
        if self._attached_model:
            return self._set_grasp_attachment(self._attached_model, False)
        return True

    def _home_operation(self):
        self._observation_ready = False
        return self._move_named(self.finish_named_target)

    def _prepare_work_operation(self):
        self._observation_ready = False
        if not self._refresh_octomap():
            return False
        # Refresh the map-fixed table before unfolding near the workstation.
        if not self._add_table_collision():
            return False
        return self._move_named(self.work_ready_named_target)

    def _verify_visible_colors(self):
        required = set(str(color) for color in self.sort_colors)
        not_before = time.time()
        deadline = not_before + self.observation_verification_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self._stop_requested.is_set():
                return False
            with self._data_lock:
                detections = self._detections
                receipt_time = self._detections_wall_time
            if detections is not None and receipt_time > not_before:
                visible = set(str(item.color) for item in detections.objects)
                if required.issubset(visible):
                    rospy.loginfo(
                        "Observation verified all colors: %s",
                        ", ".join(sorted(required)),
                    )
                    return True
            rospy.sleep(0.1)
        rospy.logwarn(
            "Observation pose did not show all required colors within %.1f seconds",
            self.observation_verification_timeout,
        )
        return False

    def _pick_and_place(self, detected):
        color = detected.color
        object_x = detected.pose.position.x + self.grasp_offset_x
        object_y = detected.pose.position.y + self.grasp_offset_y
        grasp_z = (
            self.table_z
            + 0.5 * self.object_height
            + self.grasp_height_offset
        )
        travel_z = self.table_z + self.lift_height

        rospy.loginfo(
            "Picking %s at [%.3f, %.3f, %.3f]", color, object_x, object_y, grasp_z
        )
        if not self._command_gripper(self.gripper_open):
            return False
        if not self._move_to_pose(
            self._pose(object_x, object_y, self.table_z + self.pregrasp_height),
            color + " pre-grasp",
        ):
            return False
        if not self._cartesian_to(
            self._pose(object_x, object_y, grasp_z), color + " grasp"
        ):
            return False
        if not self._command_gripper(self.gripper_closed):
            return False
        object_model_name = "{}_block".format(color)
        if not self._set_grasp_attachment(object_model_name, True):
            return False
        rospy.sleep(0.5)
        if not self._cartesian_to(
            self._pose(object_x, object_y, travel_z), color + " lift"
        ):
            return False

        if color not in self.place_targets:
            rospy.logerr("No place target configured for color '%s'", color)
            return False
        place_xy = self._xy_in_target_frame(
            self.place_frame, self.place_targets[color]
        )
        if place_xy is None:
            return False
        place_x, place_y = place_xy
        if not self._move_to_pose(
            self._pose(place_x, place_y, travel_z), color + " pre-place"
        ):
            return False
        if not self._cartesian_to(
            self._pose(place_x, place_y, grasp_z + self.place_clearance),
            color + " place",
        ):
            return False
        if not self._command_gripper(self.gripper_open):
            return False
        if not self._set_grasp_attachment(object_model_name, False):
            return False
        rospy.sleep(0.5)
        return self._cartesian_to(
            self._pose(place_x, place_y, travel_z), color + " retreat"
        )

    def _sorting_operation(self):
        for index, color in enumerate(self.sort_colors):
            if self._stop_requested.is_set():
                return False
            detection_start = time.time()
            self._publish_state("DETECTING", color)
            detected = self._wait_for_object(color, detection_start)
            if detected is None:
                return False
            self._observation_ready = False
            self._publish_state("PICKING", color)
            if not self._pick_and_place(detected):
                return False
            if index < len(self.sort_colors) - 1:
                self._publish_state("OBSERVING", "next object")
                if not self._observation():
                    return False

        self._observation_ready = False
        if self.finish_named_target:
            self._publish_state("HOMING", self.finish_named_target)
            return self._move_named(self.finish_named_target)
        return True


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("color_sorting_task")
    try:
        task = ColorSortingTask()
        task.start()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("Color sorting task failed: %s", str(error))
        return 1
    finally:
        moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
