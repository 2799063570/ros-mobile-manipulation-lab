#!/usr/bin/env python3
"""Mirror the built-in world's static box/cylinder visuals into RViz.

The simulation uses world-referenced odometry, so Gazebo world coordinates
match odom. This reads inline SDF models only, not included or dynamic models.
"""

import xml.etree.ElementTree as ET

import rospy
from tf.transformations import euler_matrix, quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray


def pose_matrix(element):
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('Course visuals require parent-relative SDF poses')
    values = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    transform = euler_matrix(*values[3:])
    transform[:3, 3] = values[:3]
    return transform


def load_markers(world_file, frame):
    markers = MarkerArray()
    world = ET.parse(world_file).getroot().find('world')
    for model in world.findall('model'):
        if model.findtext('static', 'false').strip().lower() not in ('true', '1'):
            continue
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                box = visual.find('geometry/box')
                cylinder = visual.find('geometry/cylinder')
                if box is None and cylinder is None:
                    continue
                marker = Marker()
                marker.header.frame_id = frame
                marker.ns = 'simulation_course'
                marker.id = len(markers.markers)
                marker.action = Marker.ADD
                if box is not None:
                    marker.type = Marker.CUBE
                    size = [float(v) for v in box.findtext('size').split()]
                else:
                    marker.type = Marker.CYLINDER
                    diameter = 2 * float(cylinder.findtext('radius'))
                    size = [diameter, diameter, float(cylinder.findtext('length'))]
                marker.scale.x, marker.scale.y, marker.scale.z = size
                transform = pose_matrix(model) @ pose_matrix(link) @ pose_matrix(visual)
                p = marker.pose.position
                p.x, p.y, p.z = transform[:3, 3]
                q = marker.pose.orientation
                q.x, q.y, q.z, q.w = quaternion_from_matrix(transform)
                rgba = visual.findtext('material/diffuse', '0.5 0.5 0.5 1')
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                    float(v) for v in rgba.split())
                markers.markers.append(marker)
    return markers


def main():
    rospy.init_node('follower_course_markers')
    markers = load_markers(rospy.get_param('~world'), rospy.get_param('~frame', 'odom'))
    publisher = rospy.Publisher('/aubo_mobile_follower/course', MarkerArray,
                                queue_size=1, latch=True)
    publisher.publish(markers)
    rospy.loginfo('Published %d static course visuals', len(markers.markers))
    rospy.spin()


if __name__ == '__main__':
    main()
