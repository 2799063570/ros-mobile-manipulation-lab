#!/usr/bin/env python3
"""Publish a measured camera stand as one MoveIt world collision object."""

import math

import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import (
    ApplyPlanningScene,
    ApplyPlanningSceneRequest,
    GetPlanningScene,
    GetPlanningSceneRequest,
)
from shape_msgs.msg import SolidPrimitive


def vector3(value, name, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("{} must contain exactly three numbers".format(name))
    numbers = [float(item) for item in value]
    if not all(math.isfinite(item) for item in numbers):
        raise ValueError("{} must contain finite numbers".format(name))
    if positive and not all(item > 0.0 for item in numbers):
        raise ValueError("{} dimensions must be positive".format(name))
    return numbers


def make_collision_object(frame_id, object_id, parts):
    if not frame_id or not object_id:
        raise ValueError("frame_id and object_id must be nonempty")
    if not isinstance(parts, list) or not parts:
        raise ValueError("parts must contain at least one box")

    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = object_id
    collision_object.operation = CollisionObject.ADD
    for index, part in enumerate(parts):
        if not isinstance(part, dict):
            raise ValueError("parts[{}] must be a mapping".format(index))
        size = vector3(part.get("size"), "parts[{}].size".format(index), True)
        center = vector3(part.get("center"), "parts[{}].center".format(index))
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        pose.orientation.w = 1.0
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
    return collision_object


def main():
    rospy.init_node("publish_camera_stand")
    frame_id = rospy.get_param("~frame_id")
    object_id = rospy.get_param("~object_id", "camera_stand")
    parts = rospy.get_param("~parts")
    collision_object = make_collision_object(frame_id, object_id, parts)

    rospy.wait_for_service("/apply_planning_scene", timeout=30.0)
    apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
    request = ApplyPlanningSceneRequest()
    request.scene = PlanningScene()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    request.scene.world.collision_objects.append(collision_object)
    if not apply_scene(request).success:
        raise RuntimeError("MoveIt rejected camera stand collision object")

    rospy.wait_for_service("/get_planning_scene", timeout=10.0)
    get_scene = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
    query = GetPlanningSceneRequest()
    query.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    scene = get_scene(query).scene
    if object_id not in [item.id for item in scene.world.collision_objects]:
        raise RuntimeError("camera stand was not found in MoveIt planning scene")
    rospy.loginfo(
        "Published camera stand '%s' with %d boxes in frame '%s'",
        object_id, len(parts), frame_id,
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except (KeyError, TypeError, ValueError, RuntimeError, rospy.ROSException,
            rospy.ServiceException) as error:
        rospy.logfatal("Could not publish camera stand: %s", error)
        raise
