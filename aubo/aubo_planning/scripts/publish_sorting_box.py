#!/usr/bin/env python3
"""Publish an open-top sorting box as five MoveIt collision primitives."""
import math


def vector3(value, name, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('{} must contain three numbers'.format(name))
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError('{} must be finite'.format(name))
    if positive and min(result) <= 0:
        raise ValueError('{} must be positive'.format(name))
    return result


def box_parts(outer_size, bottom_center, wall_thickness, bottom_thickness, yaw=0.):
    """bottom_center is the center of the outside BOTTOM face, not the volume."""
    length, width, height = vector3(outer_size, 'outer_size', True)
    origin = vector3(bottom_center, 'bottom_center')
    wall, bottom, yaw = float(wall_thickness), float(bottom_thickness), float(yaw)
    if not all(math.isfinite(v) for v in (wall, bottom, yaw)):
        raise ValueError('thickness and yaw must be finite')
    if not 0 < wall < min(length, width)/2 or not 0 < bottom < height:
        raise ValueError('need 0 < 2*wall_thickness < min(length,width) and 0 < bottom_thickness < height')
    wall_height, wall_z = height-bottom, (height+bottom)/2
    local = [
        ('bottom', [length, width, bottom], [0., 0., bottom/2]),
        ('wall_x_minus', [wall, width, wall_height], [-(length-wall)/2, 0., wall_z]),
        ('wall_x_plus', [wall, width, wall_height], [(length-wall)/2, 0., wall_z]),
        ('wall_y_minus', [length-2*wall, wall, wall_height], [0., -(width-wall)/2, wall_z]),
        ('wall_y_plus', [length-2*wall, wall, wall_height], [0., (width-wall)/2, wall_z]),
    ]
    cosine, sine = math.cos(yaw), math.sin(yaw)
    quaternion = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
    parts = []
    for name, size, (x, y, z) in local:
        parts.append(dict(name=name, size=size,
                          center=[origin[0]+cosine*x-sine*y,
                                  origin[1]+sine*x+cosine*y, origin[2]+z],
                          quaternion=list(quaternion)))
    return parts


def make_collision_object(frame_id, object_id, parts):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    if not isinstance(frame_id, str) or not frame_id or not isinstance(object_id, str) or not object_id:
        raise ValueError('frame_id and object_id must be nonempty strings')
    collision = CollisionObject()
    collision.header.frame_id = frame_id
    collision.id = object_id
    collision.operation = CollisionObject.ADD
    for part in parts:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = part['size']
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = part['center']
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = part['quaternion']
        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose)
    return collision


def main():
    import rospy
    from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
    from moveit_msgs.srv import (ApplyPlanningScene, ApplyPlanningSceneRequest,
                                 GetPlanningScene, GetPlanningSceneRequest)
    from std_msgs.msg import Bool

    rospy.init_node('publish_sorting_box')
    ready = rospy.Publisher('~ready', Bool, queue_size=1, latch=True)
    ready.publish(False)
    try:
        frame_id = rospy.get_param('~frame_id', 'base_link')
        object_id = rospy.get_param('~object_id', 'sorting_box')
        size = vector3(rospy.get_param('~outer_size'), 'outer_size', True)
        center = vector3(rospy.get_param('~bottom_center'), 'bottom_center')
        wall = float(rospy.get_param('~wall_thickness'))
        bottom = float(rospy.get_param('~bottom_thickness'))
        yaw = float(rospy.get_param('~yaw', 0.))
        parts = box_parts(size, center, wall, bottom, yaw)
        collision = make_collision_object(frame_id, object_id, parts)
        rospy.wait_for_service('/apply_planning_scene', timeout=30.)
        apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        request = ApplyPlanningSceneRequest()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects.append(collision)
        if not apply_scene(request).success:
            raise RuntimeError('MoveIt rejected sorting box collision object')
        rospy.wait_for_service('/get_planning_scene', timeout=10.)
        get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        query = GetPlanningSceneRequest()
        query.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        objects = get_scene(query).scene.world.collision_objects
        found = [item for item in objects if item.id == object_id]
        if len(found) != 1 or len(found[0].primitives) != 5 or len(found[0].primitive_poses) != 5:
            raise RuntimeError('open-top sorting box was not found with all five parts in MoveIt')
        ready.publish(True)
        rospy.loginfo("Published open-top box '%s' in '%s': inner size %.3f x %.3f x %.3f m; "
                      "floor z=%.3f, rim z=%.3f", object_id, frame_id,
                      size[0]-2*wall, size[1]-2*wall, size[2]-bottom,
                      center[2]+bottom, center[2]+size[2])
        rospy.spin()
    except (KeyError, TypeError, ValueError, RuntimeError, rospy.ROSException,
            rospy.ServiceException) as error:
        ready.publish(False)
        rospy.logfatal('Could not publish sorting box: %s', error)
        raise


if __name__ == '__main__':
    main()
