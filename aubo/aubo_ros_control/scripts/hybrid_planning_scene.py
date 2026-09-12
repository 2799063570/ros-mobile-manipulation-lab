#!/usr/bin/env python3
"""Keep the Gazebo sorting table in MoveIt's planning scene."""

import threading

import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool


class HybridPlanningScene:
    def __init__(self):
        self._lock = threading.Lock()
        self._ready = rospy.Publisher(
            "/hybrid/planning_scene_ready", Bool, queue_size=1, latch=True)
        self._ready.publish(False)
        self._client = rospy.ServiceProxy(
            "/apply_planning_scene", ApplyPlanningScene, persistent=True)

        self._frame = rospy.get_param("~table_frame", "base_link")
        self._center = self._vector("table_center", [0.70, 0.0, 0.0])
        self._size = self._vector("table_size", [0.70, 1.00, 0.20])
        self._xy_margin = float(rospy.get_param("~table_collision_margin", 0.02))
        self._top_margin = float(rospy.get_param("~table_top_margin", 0.01))
        if min(self._size) <= 0.0 or self._xy_margin < 0.0 or self._top_margin < 0.0:
            raise ValueError("table dimensions must be positive and margins non-negative")

        self._timer = rospy.Timer(rospy.Duration(2.0), self._refresh)
        self._refresh(None)

    @staticmethod
    def _vector(name, default):
        value = rospy.get_param("~" + name, default)
        if not isinstance(value, list) or len(value) != 3:
            raise ValueError("~{} must contain three numbers".format(name))
        return [float(item) for item in value]

    def _request(self):
        collision = CollisionObject()
        collision.header.frame_id = self._frame
        collision.id = "hybrid_sorting_table"
        collision.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self._size[0] + 2.0 * self._xy_margin,
            self._size[1] + 2.0 * self._xy_margin,
            self._size[2] + self._top_margin,
        ]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = self._center[0]
        pose.position.y = self._center[1]
        # Add vertical padding only above the physical tabletop.
        pose.position.z = self._center[2] + 0.5 * self._top_margin
        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects.append(collision)
        request = ApplyPlanningSceneRequest()
        request.scene = scene
        return request, primitive, pose

    def _refresh(self, _event):
        if not self._lock.acquire(False):
            return
        try:
            try:
                self._client.wait_for_service(timeout=1.0)
                request, primitive, pose = self._request()
                response = self._client(request)
                if not response.success:
                    raise RuntimeError("MoveIt rejected ApplyPlanningScene")
                self._ready.publish(True)
                rospy.loginfo_throttle(
                    30.0,
                    "[hybrid_scene] table active: frame=%s center=[%.3f, %.3f, %.3f] "
                    "size=[%.3f, %.3f, %.3f]",
                    self._frame, pose.position.x, pose.position.y, pose.position.z,
                    *primitive.dimensions)
            except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
                self._ready.publish(False)
                # Recreate a persistent proxy after move_group restarts.
                self._client = rospy.ServiceProxy(
                    "/apply_planning_scene", ApplyPlanningScene, persistent=True)
                rospy.logwarn_throttle(
                    2.0, "[hybrid_scene] waiting for MoveIt planning scene: %s", error)
        finally:
            self._lock.release()


def main():
    rospy.init_node("hybrid_planning_scene")
    HybridPlanningScene()
    rospy.spin()


if __name__ == "__main__":
    main()
