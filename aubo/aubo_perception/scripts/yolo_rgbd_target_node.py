#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共享 RGB-D 几何层：Ultralytics 为默认入口，darknet 仅为可选兼容。

推理保持独立进程以隔离 Conda / cv_bridge 的 OpenCV ABI；深度缓存按 RGB
采集时间匹配，推理耗时不会使结果错误地使用刚收到的另一帧深度。
"""

from __future__ import print_function

import math
import threading
import os
from collections import deque
from types import SimpleNamespace

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
from std_msgs.msg import String, Float32
from tf.transformations import quaternion_matrix
from aubo_perception.grasp_geometry import validate_modes, center_height, ray_plane, rectangle_grasp

from aubo_perception.msg import DetectedObject, DetectedObjectArray, YoloDetectionArray


class YoloRgbdTargetNode(object):
    def __init__(self):
        self.backend = rospy.get_param("~backend", "ultralytics")
        if self.backend not in ("ultralytics", "darknet"):
            raise ValueError("backend must be ultralytics or darknet")
        if self.backend == "darknet" and BoundingBoxes is None:
            raise ValueError("darknet backend requires darknet_ros_msgs")
        detector = rospy.get_param("~detector", "yolo")
        if detector not in ("yolo", "color"):
            raise ValueError("detector must be yolo or color")
        self.workspace = rospy.get_param("~workspace", {})
        self.task_mode = rospy.get_param("~task_mode", "sorting")
        self.height_mode = rospy.get_param("~height_mode", "table")
        self.camera_mount = rospy.get_param("~camera_mount", "eye_in_hand")
        self.table_z = float(rospy.get_param("~table_z", 0.10))
        self.object_height = float(rospy.get_param("~object_height", 0.04))
        self.class_heights = rospy.get_param("~class_heights", {})
        self.height_tolerance = float(rospy.get_param("~height_tolerance", 0.02))
        for height in self.class_heights.values():
            center_height(self.table_z + float(height), self.table_z, float(height), self.height_tolerance)
        self.boxes_topic = rospy.get_param(
            "~boxes_topic", "/yolo/detections"
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
        validate_modes(self.task_mode, self.height_mode, self.camera_mount, self.selected_class)
        center_height(self.table_z + self.object_height, self.table_z,
                      self.object_height, self.height_tolerance)
        self.class_aliases = rospy.get_param("~class_aliases", {})
        self.minimum_probability = float(
            rospy.get_param("~minimum_probability", 0.50)
        )
        self.minimum_depth = float(rospy.get_param("~minimum_depth", 0.15))
        self.maximum_depth = float(rospy.get_param("~maximum_depth", 2.50))
        self.depth_scale_16u = float(rospy.get_param("~depth_scale_16u", 0.001))
        self.roi_scale = min(1.0, max(0.1, float(rospy.get_param("~roi_scale", 0.55))))
        self.maximum_depth_age = float(rospy.get_param("~maximum_depth_age", 0.25))

        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self._camera_info = None
        self._info_lock = threading.Lock()
        self._depth_cache = deque(maxlen=max(1, int(rospy.get_param("~depth_cache_size", 90))))
        self.depth_save_directory = os.path.expanduser(rospy.get_param("~depth_save_directory", ""))
        if self.depth_save_directory:
            os.makedirs(self.depth_save_directory, exist_ok=True)
        self._depth_lock = threading.Lock()

        self.detections_publisher = rospy.Publisher(
            self.detections_topic, DetectedObjectArray, queue_size=2
        )
        self.target_pose_publisher = rospy.Publisher(
            self.target_pose_topic, PoseStamped, queue_size=1
        )
        self.target_confidence_publisher = rospy.Publisher(
            rospy.get_param("~target_confidence_topic", "/visual_servo/target_confidence"), Float32, queue_size=1)
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
            self.boxes_topic, YoloDetectionArray if self.backend == "ultralytics" else BoundingBoxes,
            self._boxes_callback, queue_size=1
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
            if self._depth_cache and message.header.stamp < self._depth_cache[-1].header.stamp:
                self._depth_cache.clear()  # Simulation clock reset.
            self._depth_cache.append(message)

    def _boxes_callback(self, message):
        image_header = getattr(message, "image_header", message.header)
        box_stamp = image_header.stamp
        if box_stamp == rospy.Time(0):
            return
        depth_message = None
        # 桌高分拣可独立于深度工作；伺服与 depth 模式必须有匹配深度。
        need_depth = self.height_mode == "depth" or self.task_mode in ("servo", "both")
        if need_depth:
            with self._depth_lock:
                if self._depth_cache:
                    depth_message = min(self._depth_cache,
                        key=lambda depth: abs((depth.header.stamp - box_stamp).to_sec()))
            if depth_message is None or abs((box_stamp - depth_message.header.stamp).to_sec()) > self.maximum_depth_age:
                rospy.logwarn_throttle(1.0, "No cached depth matching the detector source image")
                self._publish_empty(image_header)
                return
        if self.backend == "ultralytics":
            boxes = []
            for item in message.detections:
                boxes.append(SimpleNamespace(
                    class_name=item.class_name, probability=item.confidence,
                    center_x=item.center_x, center_y=item.center_y,
                    width=item.width, height=item.height, angle=item.angle,
                    orientation_valid=item.orientation_valid,
                    xmin=item.center_x - item.width / 2, xmax=item.center_x + item.width / 2,
                    ymin=item.center_y - item.height / 2, ymax=item.center_y + item.height / 2))
            message = SimpleNamespace(header=image_header, bounding_boxes=boxes)
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
        # OBB 中心区域按旋转框局部坐标筛选，减少把框外桌面混入深度统计。
        if hasattr(box, "angle"):
            yy, xx = np.mgrid[y0:y1, x0:x1]
            du, dv = xx-box.center_x, yy-box.center_y
            co, si = math.cos(box.angle), math.sin(box.angle)
            inside = ((np.abs(co*du + si*dv) <= box.width*self.roi_scale/2) &
                      (np.abs(-si*du + co*dv) <= box.height*self.roi_scale/2))
            samples = samples[inside]
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

    def _publish_empty(self, header):
        output = DetectedObjectArray()
        output.header.stamp = header.stamp
        output.header.frame_id = self.target_frame
        if self.task_mode in ("sorting", "both"):
            self.detections_publisher.publish(output)
        if self.task_mode in ("servo", "both"):
            self.target_label_publisher.publish(String(data=""))
            self.target_confidence_publisher.publish(Float32(data=0.0))

    def _callback(self, boxes_message, depth_message):
        with self._info_lock:
            info = self._camera_info
        if info is None or not info.header.frame_id:
            rospy.logwarn_throttle(2.0, "Waiting for RGB camera_info and frame_id")
            return
        stamp = boxes_message.header.stamp
        depth = None
        if depth_message is not None:
            try:
                depth = self._depth_metres(depth_message)
            except (CvBridgeError, TypeError, ValueError) as error:
                rospy.logwarn_throttle(2.0, "Cannot decode aligned depth: %s", str(error))
                self._publish_empty(boxes_message.header)
                return
            if depth.shape != (info.height, info.width):
                rospy.logwarn_throttle(2.0, "Aligned depth and color calibration dimensions differ")
                self._publish_empty(boxes_message.header)
                return
            if self.depth_save_directory:
                # 仅在显式配置目录时落盘；默认只做有界内存缓存。
                try:
                    np.savez_compressed(
                        os.path.join(self.depth_save_directory, str(depth_message.header.stamp.to_nsec()) + ".npz"),
                        depth=np.asarray(self.bridge.imgmsg_to_cv2(depth_message, "passthrough")),
                        encoding=depth_message.encoding, K=np.asarray(info.K), frame_id=info.header.frame_id,
                        depth_stamp_ns=depth_message.header.stamp.to_nsec(), image_stamp_ns=stamp.to_nsec())
                except (OSError, CvBridgeError) as error:
                    rospy.logerr_throttle(2.0, "Cannot save depth: %s", str(error))
        output = DetectedObjectArray()
        output.header.stamp = stamp
        output.header.frame_id = self.target_frame
        ranked = []
        transform = None
        for box in boxes_message.bounding_boxes:
            probability = float(getattr(box, "probability", 0.0))
            label = self._label(box)
            if not math.isfinite(probability) or probability < self.minimum_probability:
                continue
            if self.task_mode == "servo" and label != self.selected_class.lower():
                continue
            u = float(getattr(box, "center_x", (box.xmin + box.xmax)/2))
            v = float(getattr(box, "center_y", (box.ymin + box.ymax)/2))
            if not (0 <= u < info.width and 0 <= v < info.height):
                continue
            dimensions = (float(getattr(box, "width", box.xmax-box.xmin)),
                          float(getattr(box, "height", box.ymax-box.ymin)),
                          float(getattr(box, "angle", 0.0)))
            if not all(math.isfinite(value) for value in dimensions) or min(dimensions[:2]) <= 0:
                continue
            camera_pose = None
            if depth is not None:
                depth_result = self._depth_for_box(depth, box)
                if depth_result is None:
                    continue
                camera_pose = self._camera_pose(info, stamp, depth_result[0], u, v)
                if camera_pose is None:
                    continue
                if self.task_mode in ("servo", "both") and label == self.selected_class.lower():
                    ranked.append((probability, camera_pose, label))
            if self.task_mode == "servo":
                continue
            try:
                # 眼在手上必须使用采集时刻的动态 TF，不能回退到最新变换。
                if transform is None:
                    self.tf_listener.waitForTransform(self.target_frame, info.header.frame_id, stamp, rospy.Duration(0.15))
                    transform = self.tf_listener.lookupTransform(self.target_frame, info.header.frame_id, stamp)
                translation, quaternion = transform
                rotation = quaternion_matrix(quaternion)[:3, :3]
                object_height = float(self.class_heights.get(label, self.object_height))
                surface_z = self.table_z + object_height
                if self.height_mode == "depth":
                    position = camera_pose.pose.position
                    point = rotation.dot([position.x, position.y, position.z]) + translation
                    surface_z = float(point[2])
                    z = center_height(surface_z, self.table_z, object_height, self.height_tolerance)
                else:
                    point = ray_plane(u, v, info.K, rotation, translation, surface_z)
                    z = self.table_z + object_height/2
                if any(not float(self.workspace.get("min_"+axis, -float("inf"))) <= float(point[i]) <=
                           float(self.workspace.get("max_"+axis, float("inf")))
                       for i, axis in enumerate(("x", "y"))):
                    continue
                width, yaw = rectangle_grasp(u, v,
                    float(getattr(box, "width", box.xmax-box.xmin)),
                    float(getattr(box, "height", box.ymax-box.ymin)), float(getattr(box, "angle", 0.0)),
                    info.K, rotation, translation, surface_z)
                detected = DetectedObject()
                detected.color = detected.class_name = label
                detected.confidence = probability
                detected.pose.position.x, detected.pose.position.y = float(point[0]), float(point[1])
                detected.pose.position.z = z
                detected.pose.orientation.w = 1.0
                detected.grasp_width, detected.grasp_angle = width, yaw
                detected.grasp_geometry_valid = bool(getattr(box, "orientation_valid", False))
                detected.depth_valid = self.height_mode == "depth"
                detected.object_height = object_height
                detected.contour_area = max(0., box.xmax-box.xmin)*max(0., box.ymax-box.ymin)
                detected.pixel_x, detected.pixel_y = int(round(u)), int(round(v))
                output.objects.append(detected)
            except (ValueError, tf.Exception) as error:
                rospy.logwarn_throttle(2.0, "Rejecting grasp target: %s", str(error))
        if self.task_mode in ("sorting", "both"):
            self.detections_publisher.publish(output)
        if self.task_mode in ("servo", "both"):
            if ranked:
                confidence, camera_pose, label = max(ranked, key=lambda item: item[0])
                self.target_confidence_publisher.publish(Float32(data=confidence))
                self.target_pose_publisher.publish(camera_pose)
                self.target_label_publisher.publish(String(data=label))
            else:
                # Pose 不重发旧值；控制器必须按源时间戳检查目标超时。
                self.target_label_publisher.publish(String(data=""))
                self.target_confidence_publisher.publish(Float32(data=0.0))


def main():
    rospy.init_node("yolo_rgbd_target")
    if rospy.get_param("~backend", "ultralytics") == "darknet" and BoundingBoxes is None:
        rospy.logfatal(
            "yolo_rgbd_target requires the optional darknet_ros_msgs package; "
            "install a maintained detector backend or migrate this adapter to its messages."
        )
        return
    YoloRgbdTargetNode()
    rospy.spin()


if __name__ == "__main__":
    main()
