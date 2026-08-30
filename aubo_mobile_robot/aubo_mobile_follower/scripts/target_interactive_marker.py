#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Move the Gazebo follower target with standard RViz interactive markers."""

from __future__ import print_function

import copy
import threading

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


class TargetInteractiveMarker(object):
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "follower_target")
        self.marker_frame = rospy.get_param("~marker_frame", "odom")
        self.target_z = float(rospy.get_param("~target_z", 0.5))
        self.description = rospy.get_param(
            "~description", "拖动 X/Y 拉杆移动跟随目标"
        )
        self.marker_size_x = float(rospy.get_param("~marker_size_x", 0.4))
        self.marker_size_y = float(rospy.get_param("~marker_size_y", 0.4))
        self.marker_size_z = float(rospy.get_param("~marker_size_z", 1.0))
        self.update_rate = float(rospy.get_param("~update_rate", 20.0))
        self._lock = threading.Lock()
        self._pending_pose = None

        rospy.loginfo("Waiting for Gazebo model-state services")
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self._get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState
        )
        self._set_model_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState
        )

        initial_pose = self._wait_for_model()
        self.server = InteractiveMarkerServer(
            "/aubo_mobile_follower/target_control"
        )
        self._create_marker(initial_pose)
        self.server.applyChanges()
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / max(1.0, self.update_rate)), self._apply_pose
        )
        rospy.loginfo(
            "RViz target control ready for Gazebo model '%s'", self.model_name
        )

    def _wait_for_model(self):
        while not rospy.is_shutdown():
            try:
                response = self._get_model_state(self.model_name, "world")
                if response.success:
                    pose = response.pose
                    pose.position.z = self.target_z
                    return pose
                rospy.logwarn_throttle(
                    2.0, "Waiting for Gazebo model '%s'", self.model_name
                )
            except rospy.ServiceException as error:
                rospy.logwarn_throttle(2.0, "Cannot query target model: %s", str(error))
            rospy.sleep(0.2)
        raise rospy.ROSInterruptException("shutdown while waiting for target")

    @staticmethod
    def _axis_control(name, quaternion):
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.x = quaternion[0]
        control.orientation.y = quaternion[1]
        control.orientation.z = quaternion[2]
        control.orientation.w = quaternion[3]
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        return control

    def _create_marker(self, pose):
        interactive = InteractiveMarker()
        interactive.header.frame_id = self.marker_frame
        interactive.name = "follower_target"
        interactive.description = self.description
        interactive.pose = pose
        interactive.scale = 0.9

        visual = Marker()
        visual.type = Marker.CUBE
        visual.scale.x = self.marker_size_x
        visual.scale.y = self.marker_size_y
        visual.scale.z = self.marker_size_z
        visual.color.r = 0.9
        visual.color.g = 0.15
        visual.color.b = 0.15
        visual.color.a = 0.65
        visual.pose.orientation.w = 1.0

        visible = InteractiveMarkerControl()
        visible.name = "target_body"
        visible.always_visible = True
        visible.markers.append(visual)
        interactive.controls.append(visible)

        # Interactive-marker controls move along their local X axis.
        interactive.controls.append(
            self._axis_control("move_x", (0.707107, 0.0, 0.0, 0.707107))
        )
        interactive.controls.append(
            self._axis_control("move_y", (0.0, 0.0, 0.707107, 0.707107))
        )
        self.server.insert(interactive, self._feedback)

    def _feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        pose = copy.deepcopy(feedback.pose)
        pose.position.z = self.target_z
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        with self._lock:
            self._pending_pose = pose

    def _apply_pose(self, _event):
        with self._lock:
            pose = self._pending_pose
            self._pending_pose = None
        if pose is None:
            return

        state = ModelState()
        state.model_name = self.model_name
        state.pose = pose
        state.reference_frame = "world"
        try:
            response = self._set_model_state(state)
            if not response.success:
                rospy.logwarn_throttle(
                    1.0, "Gazebo rejected target pose: %s", response.status_message
                )
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(1.0, "Cannot move Gazebo target: %s", str(error))


def main():
    rospy.init_node("follower_target_control")
    TargetInteractiveMarker()
    rospy.spin()


if __name__ == "__main__":
    main()
