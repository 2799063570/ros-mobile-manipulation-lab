#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import copy
import math
import sys
import threading
import time

import actionlib
import moveit_commander
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint

from aubo_mobile_perception.msg import DetectedObjectArray


class ColorSortingTask(object):
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

        self.table_z = float(rospy.get_param("~table_z", 0.12))
        self.table_center = rospy.get_param("~table_center", [0.80, 0.0, -0.08])
        self.table_size = rospy.get_param("~table_size", [0.80, 1.20, 0.40])
        self.object_height = float(rospy.get_param("~object_height", 0.05))
        self.grasp_rpy = rospy.get_param("~grasp_rpy", [math.pi, 0.0, 0.0])
        self.observation_pose = rospy.get_param(
            "~observation_pose", [0.58, 0.0, 0.62]
        )
        self.pregrasp_height = float(rospy.get_param("~pregrasp_height", 0.25))
        self.lift_height = float(rospy.get_param("~lift_height", 0.30))
        self.place_clearance = float(rospy.get_param("~place_clearance", 0.025))
        self.cartesian_step = float(rospy.get_param("~cartesian_step", 0.01))
        self.minimum_cartesian_fraction = float(
            rospy.get_param("~minimum_cartesian_fraction", 0.90)
        )

        self.gripper_open = float(rospy.get_param("~gripper_open", 0.0))
        self.gripper_closed = float(rospy.get_param("~gripper_closed", 0.35))
        self.gripper_motion_time = float(
            rospy.get_param("~gripper_motion_time", 1.5)
        )
        self.sort_colors = rospy.get_param("~sort_colors", ["red", "green", "blue"])
        self.place_targets = rospy.get_param("~place_targets")
        self.detection_timeout = float(rospy.get_param("~detection_timeout", 15.0))
        self.detection_settle_time = float(
            rospy.get_param("~detection_settle_time", 1.0)
        )
        self.gripper_server_timeout = float(
            rospy.get_param("~gripper_server_timeout", 30.0)
        )
        self.finish_named_target = rospy.get_param("~finish_named_target", "down")

        self._detections = None
        self._detections_wall_time = 0.0
        self._lock = threading.Lock()
        self._subscriber = rospy.Subscriber(
            self.detections_topic,
            DetectedObjectArray,
            self._detection_callback,
            queue_size=2,
        )

        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander(self.group_name)
        self.arm.set_end_effector_link(self.end_effector_link)
        self.arm.set_pose_reference_frame(self.target_frame)
        self.arm.set_planning_time(float(rospy.get_param("~planning_time", 12.0)))
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(
            float(rospy.get_param("~velocity_scaling", 0.15))
        )
        self.arm.set_max_acceleration_scaling_factor(
            float(rospy.get_param("~acceleration_scaling", 0.15))
        )

        self.gripper_client = actionlib.SimpleActionClient(
            self.gripper_action_name, FollowJointTrajectoryAction
        )

    def _detection_callback(self, message):
        with self._lock:
            self._detections = message
            self._detections_wall_time = time.time()

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

    def _move_to_pose(self, pose, description):
        rospy.loginfo("Planning arm to %s", description)
        self.arm.set_pose_target(pose, self.end_effector_link)
        success = bool(self.arm.go(wait=True))
        self.arm.stop()
        self.arm.clear_pose_targets()
        if not success:
            rospy.logerr("MoveIt failed to reach %s", description)
        return success

    def _move_named(self, target):
        if not target:
            return True
        if target not in self.arm.get_named_targets():
            rospy.logerr("Unknown arm named target '%s'", target)
            return False
        rospy.loginfo("Moving arm to named target %s", target)
        self.arm.set_named_target(target)
        success = bool(self.arm.go(wait=True))
        self.arm.stop()
        return success

    def _cartesian_to(self, target_pose, description):
        waypoint = copy.deepcopy(target_pose.pose)
        plan, fraction = self.arm.compute_cartesian_path(
            [waypoint], self.cartesian_step, 0.0, True
        )
        rospy.loginfo("Cartesian path to %s: %.1f%%", description, 100.0 * fraction)
        if fraction < self.minimum_cartesian_fraction:
            rospy.logwarn("Cartesian fraction too low; falling back to pose planning")
            return self._move_to_pose(target_pose, description)
        success = bool(self.arm.execute(plan, wait=True))
        self.arm.stop()
        if not success:
            rospy.logerr("Failed to execute Cartesian path to %s", description)
        return success

    def _command_gripper(self, position):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["joint1", "joint2"]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = rospy.Duration(self.gripper_motion_time)
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        self.gripper_client.send_goal(goal)
        if not self.gripper_client.wait_for_result(
            rospy.Duration(self.gripper_motion_time + 3.0)
        ):
            self.gripper_client.cancel_goal()
            rospy.logerr("Gripper command timed out")
            return False
        if self.gripper_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("Gripper action failed with state %d", self.gripper_client.get_state())
            return False
        return True

    def _add_table_collision(self):
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.target_frame
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = float(self.table_center[0])
        table_pose.pose.position.y = float(self.table_center[1])
        table_pose.pose.position.z = float(self.table_center[2])
        self.scene.add_box(
            "sorting_table",
            table_pose,
            size=tuple(float(value) for value in self.table_size),
        )
        rospy.sleep(1.0)
        self.arm.set_support_surface_name("sorting_table")

    def _wait_for_object(self, color, not_before):
        deadline = time.time() + self.detection_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self._lock:
                detections = self._detections
                receipt_time = self._detections_wall_time
            if detections is not None and receipt_time >= not_before:
                candidates = [item for item in detections.objects if item.color == color]
                if candidates:
                    return max(candidates, key=lambda item: item.contour_area)
            rospy.sleep(0.1)
        rospy.logerr("No fresh '%s' object detected within %.1f seconds", color, self.detection_timeout)
        return None

    def _observation(self):
        pose = self._pose(*self.observation_pose)
        if not self._move_to_pose(pose, "camera observation pose"):
            return False
        rospy.sleep(self.detection_settle_time)
        return True

    def _pick_and_place(self, detected):
        color = detected.color
        object_x = detected.pose.position.x
        object_y = detected.pose.position.y
        grasp_z = self.table_z + 0.5 * self.object_height
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
        rospy.sleep(0.5)
        if not self._cartesian_to(
            self._pose(object_x, object_y, travel_z), color + " lift"
        ):
            return False

        if color not in self.place_targets:
            rospy.logerr("No place target configured for color '%s'", color)
            return False
        place_xy = self.place_targets[color]
        place_x = float(place_xy[0])
        place_y = float(place_xy[1])
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
        rospy.sleep(0.5)
        return self._cartesian_to(
            self._pose(place_x, place_y, travel_z), color + " retreat"
        )

    def run(self):
        rospy.loginfo("Waiting for gripper action %s", self.gripper_action_name)
        if not self.gripper_client.wait_for_server(
            rospy.Duration(self.gripper_server_timeout)
        ):
            rospy.logerr("Gripper action server is unavailable")
            return False

        self._add_table_collision()
        if not self._observation():
            return False

        for color in self.sort_colors:
            detection_start = time.time()
            detected = self._wait_for_object(color, detection_start)
            if detected is None:
                return False
            if not self._pick_and_place(detected):
                return False
            if color != self.sort_colors[-1] and not self._observation():
                return False

        if self.finish_named_target:
            return self._move_named(self.finish_named_target)
        return True


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("color_sorting_task")
    exit_code = 1
    try:
        task = ColorSortingTask()
        exit_code = 0 if task.run() else 1
    except Exception as error:
        rospy.logerr("Color sorting task failed: %s", str(error))
    finally:
        moveit_commander.roscpp_shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
