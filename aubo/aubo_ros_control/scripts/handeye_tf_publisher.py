#!/usr/bin/env python3

import math

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped


def required(mapping, key, section):
    if key not in mapping:
        raise ValueError("missing '{}.{}' in calibration file".format(section, key))
    return mapping[key]


def main():
    rospy.init_node("handeye_tf_publisher")
    calibration_file = rospy.get_param("~calibration_file")

    with open(calibration_file, "r") as stream:
        calibration = yaml.safe_load(stream) or {}

    parameters = required(calibration, "parameters", "root")
    transform = required(calibration, "transformation", "root")
    eye_on_hand = bool(required(parameters, "eye_on_hand", "parameters"))
    default_parent_key = "robot_effector_frame" if eye_on_hand else "robot_base_frame"
    parent_frame = rospy.get_param(
        "~parent_frame", required(parameters, default_parent_key, "parameters")
    )
    child_frame = rospy.get_param(
        "~child_frame", required(parameters, "tracking_base_frame", "parameters")
    )

    quaternion = [
        float(required(transform, key, "transformation"))
        for key in ("qx", "qy", "qz", "qw")
    ]
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-12:
        raise ValueError("calibration quaternion has zero length")
    quaternion = [value / norm for value in quaternion]

    message = TransformStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = parent_frame
    message.child_frame_id = child_frame
    message.transform.translation.x = float(required(transform, "x", "transformation"))
    message.transform.translation.y = float(required(transform, "y", "transformation"))
    message.transform.translation.z = float(required(transform, "z", "transformation"))
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(message)
    rospy.loginfo(
        "Published hand-eye TF %s -> %s from %s",
        parent_frame,
        child_frame,
        calibration_file,
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except (IOError, OSError, ValueError, yaml.YAMLError) as error:
        rospy.logfatal("Failed to publish hand-eye calibration: %s", error)
        raise
