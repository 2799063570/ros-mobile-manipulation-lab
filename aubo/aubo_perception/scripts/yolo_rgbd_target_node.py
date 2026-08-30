#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Detector-neutral bridge from darknet_ros boxes + aligned depth to AUBO targets.

Outputs both the shared sorting contract and the visual-servo PoseStamped
contract.  MoveIt, the sorting state machine and visual control therefore do
not import darknet_ros message types directly.
"""

from __future__ import print_function

import threading

import numpy as np
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
try:
    from darknet_ros_msgs.msg import BoundingBoxes
except ImportError:
    BoundingBoxes = None
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from aubo_perception.msg import DetectedObject, DetectedObjectArray


class YoloRgbdTargetNode(object):
    def __init__(self):
        self.boxes_topic = rospy.get_param(
            "~boxes_topic", "/darknet_ros/bounding_boxes"
        )
        self.depth_topic = rospy.get_param(
            "~aligned_depth_topic",
            "/workspace_camera/aligned_depth_to_color/image_raw",
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/workspace_camera/color/camera_info"
        )
        self.detections_topic = rospy.get_param(
            "~detections_topic", "/sorting/detections"
        )
        self.target_pose_topic = rospy.get_param(
            "~target_pose_topic", "/visual_servo/target_pose"
        )
        self.target_label_topic = rospy.get_param(
            "~target_label_topic", "/visual_servo/target_label"
        )
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.selected_class = rospy.get_param("~selected_class", "")
        self.class_aliases = rospy.get_param("~class_aliases", {})
        self.minimum_probability = float(
            rospy.get_param("~minimum_probability", 0.50)
        )
        self.minimum_depth = float(rospy.get_param("~minimum_depth", 0.15))
        self.maximum_depth = float(rospy.get_param("~maximum_depth", 2.50))
        self.depth_scale_16u = float(rospy.get_param("~depth_scale_16u", 0.001))
        self.roi_scale = min(1.0, max(0.1, float(rospy.get_param("~roi_scale", 0.55))))
        self.object_center_z = float(rospy.get_param("~object_center_z", 0.16))
        self.maximum_depth_age = float(rospy.get_param("~maximum_depth_age", 0.25))

        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self._camera_info = None
        self._info_lock = threading.Lock()
        self._depth_message = None
        self._depth_lock = threading.Lock()

        self.detections_publisher = rospy.Publisher(
            self.detections_topic, DetectedObjectArray, queue_size=2
        )
        self.target_pose_publisher = rospy.Publisher(
            self.target_pose_topic, PoseStamped, queue_size=1
        )
        self.target_label_publisher = rospy.Publisher(
            self.target_label_topic, String, queue_size=1
        )
        self.info_subscriber = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1
        )
        self.depth_subscriber = rospy.Subscriber(
            self.depth_topic, Image, self._depth_callback, queue_size=1
        )
        self.boxes_subscriber = rospy.Subscriber(
            self.boxes_topic, BoundingBoxes, self._boxes_callback, queue_size=1
        )
        rospy.loginfo(
            "YOLO RGB-D bridge: %s + %s -> %s",
            self.boxes_topic,
            self.depth_topic,
            self.detections_topic,
        )

    def _camera_info_callback(self, message):
        with self._info_lock:
            self._camera_info = message

    def _depth_callback(self, message):
        with self._depth_lock:
            self._depth_message = message

    def _boxes_callback(self, message):
        with self._depth_lock:
            depth_message = self._depth_message
        if depth_message is None:
            rospy.logwarn_throttle(2.0, "Waiting for aligned RGB-D depth")
            return
        image_header = getattr(message, "image_header", message.header)
        box_stamp = image_header.stamp if image_header.stamp != rospy.Time(0) else message.header.stamp
        if (box_stamp != rospy.Time(0) and depth_message.header.stamp != rospy.Time(0) and
                abs((box_stamp - depth_message.header.stamp).to_sec()) > self.maximum_depth_age):
            rospy.logwarn_throttle(1.0, "YOLO boxes and aligned depth are too far apart")
            return
        self._callback(message, depth_message)

    def _depth_metres(self, message):
        image = np.asarray(self.bridge.imgmsg_to_cv2(message, "passthrough"))
        if image.dtype == np.uint16:
            return image.astype(np.float32) * self.depth_scale_16u
        return image.astype(np.float32)

    def _label(self, box):
        raw = str(getattr(box, "Class", getattr(box, "class_name", "")))
        return str(self.class_aliases.get(raw, raw)).lower()

    def _depth_for_box(self, depth, box):
        height, width = depth.shape[:2]
        xmin = max(0, min(width - 1, int(box.xmin)))
        xmax = max(0, min(width, int(box.xmax)))
        ymin = max(0, min(height - 1, int(box.ymin)))
        ymax = max(0, min(height, int(box.ymax)))
        if xmax <= xmin or ymax <= ymin:
            return None
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        half_w = max(1, int(0.5 * (xmax - xmin) * self.roi_scale))
        half_h = max(1, int(0.5 * (ymax - ymin) * self.roi_scale))
        x0, x1 = max(0, int(cx) - half_w), min(width, int(cx) + half_w + 1)
        y0, y1 = max(0, int(cy) - half_h), min(height, int(cy) + half_h + 1)
        samples = depth[y0:y1, x0:x1]
        finite = np.isfinite(samples)
        with np.errstate(invalid="ignore"):
            valid = finite & (samples >= self.minimum_depth) & (samples <= self.maximum_depth)
        samples = samples[valid]
        if samples.size < 5:
            return None
        return float(np.median(samples)), int(round(cx)), int(round(cy))

    @staticmethod
    def _camera_pose(info, stamp, depth, pixel_x, pixel_y):
        fx, fy = float(info.K[0]), float(info.K[4])
        cx, cy = float(info.K[2]), float(info.K[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = info.header.frame_id
        pose.pose.position.x = (pixel_x - cx) * depth / fx
        pose.pose.position.y = (pixel_y - cy) * depth / fy
        pose.pose.position.z = depth
        pose.pose.orientation.w = 1.0
        return pose

    def _to_target_frame(self, camera_pose):
        try:
            self.tf_listener.waitForTransform(
                self.target_frame,
                camera_pose.header.frame_id,
                camera_pose.header.stamp,
                rospy.Duration(0.15),
            )
            return self.tf_listener.transformPose(self.target_frame, camera_pose)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as error:
            rospy.logwarn_throttle(1.0, "YOLO target TF unavailable: %s", str(error))
            return None

    def _callback(self, boxes_message, depth_message):
        with self._info_lock:
            info = self._camera_info
        if info is None or not info.header.frame_id:
            rospy.logwarn_throttle(2.0, "Waiting for RGB camera_info and frame_id")
            return
        try:
            depth = self._depth_metres(depth_message)
        except (CvBridgeError, TypeError, ValueError) as error:
            rospy.logwarn_throttle(2.0, "Cannot decode aligned depth: %s", str(error))
            return

        output = DetectedObjectArray()
        output.header.stamp = depth_message.header.stamp
        output.header.frame_id = self.target_frame
        ranked = []
        for box in boxes_message.bounding_boxes:
            probability = float(getattr(box, "probability", 0.0))
            label = self._label(box)
            if probability < self.minimum_probability:
                continue
            if self.selected_class and label != self.selected_class.lower():
                continue
            depth_result = self._depth_for_box(depth, box)
            if depth_result is None:
                continue
            distance, pixel_x, pixel_y = depth_result
            camera_pose = self._camera_pose(
                info, depth_message.header.stamp, distance, pixel_x, pixel_y
            )
            if camera_pose is None:
                continue
            target_pose = self._to_target_frame(camera_pose)
            if target_pose is None:
                continue
            detected = DetectedObject()
            detected.color = label
            detected.pose = target_pose.pose
            # Sorting expects the known object centre height rather than a
            # noisy depth-surface Z coordinate.
            detected.pose.position.z = self.object_center_z
            detected.contour_area = float(
                max(0, int(box.xmax) - int(box.xmin))
                * max(0, int(box.ymax) - int(box.ymin))
            )
            detected.pixel_x = pixel_x
            detected.pixel_y = pixel_y
            output.objects.append(detected)
            ranked.append((probability, camera_pose, label))

        self.detections_publisher.publish(output)
        if ranked:
            _, camera_pose, label = max(ranked, key=lambda item: item[0])
            self.target_pose_publisher.publish(camera_pose)
            self.target_label_publisher.publish(String(data=label))


def main():
    rospy.init_node("yolo_rgbd_target")
    if BoundingBoxes is None:
        rospy.logfatal(
            "yolo_rgbd_target requires the optional darknet_ros_msgs package; "
            "install a maintained detector backend or migrate this adapter to its messages."
        )
        return
    YoloRgbdTargetNode()
    rospy.spin()


if __name__ == "__main__":
    main()
