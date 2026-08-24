#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import threading

import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_matrix

from aubo_mobile_perception.msg import DetectedObject, DetectedObjectArray


class ColorObjectDetector(object):
    """Detect HSV blobs and intersect their camera rays with a known table plane."""

    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/hand_camera/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/hand_camera/camera_info"
        )
        self.detections_topic = rospy.get_param(
            "~detections_topic", "/sorting/detections"
        )
        self.debug_image_topic = rospy.get_param(
            "~debug_image_topic", "/sorting/debug_image"
        )
        self.colors = rospy.get_param("~colors")
        self.min_area = float(rospy.get_param("~min_contour_area", 180.0))
        self.max_area = float(rospy.get_param("~max_contour_area", 8000.0))
        self.max_aspect_ratio = float(rospy.get_param("~max_aspect_ratio", 1.6))
        self.position_offset_x = float(rospy.get_param("~position_offset_x", 0.0))
        self.position_offset_y = float(rospy.get_param("~position_offset_y", 0.0))
        self.kernel_size = int(rospy.get_param("~morphology_kernel", 5))
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.table_z = float(rospy.get_param("~table_z", 0.14))
        self.projection_plane_z = float(
            rospy.get_param("~projection_plane_z", self.table_z + 0.05)
        )
        self.object_center_z = float(
            rospy.get_param("~object_center_z", self.table_z + 0.025)
        )
        self.min_x = float(rospy.get_param("~workspace_min_x", 0.40))
        self.max_x = float(rospy.get_param("~workspace_max_x", 0.82))
        self.min_y = float(rospy.get_param("~workspace_min_y", -0.22))
        self.max_y = float(rospy.get_param("~workspace_max_y", 0.22))

        if self.kernel_size < 1:
            self.kernel_size = 1
        if self.kernel_size % 2 == 0:
            self.kernel_size += 1

        self._bridge = CvBridge()
        self._listener = tf.TransformListener()
        self._camera_info = None
        self._lock = threading.Lock()
        self._kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        self._detections_publisher = rospy.Publisher(
            self.detections_topic, DetectedObjectArray, queue_size=2
        )
        self._debug_publisher = rospy.Publisher(
            self.debug_image_topic, Image, queue_size=1
        )
        self._camera_info_subscriber = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1
        )
        self._image_subscriber = rospy.Subscriber(
            self.image_topic, Image, self._image_callback, queue_size=1
        )

        rospy.loginfo(
            "OpenCV detector: %s -> %s, projection z=%.3f, object z=%.3f in %s",
            self.image_topic,
            self.detections_topic,
            self.projection_plane_z,
            self.object_center_z,
            self.target_frame,
        )

    def _camera_info_callback(self, message):
        with self._lock:
            self._camera_info = message

    @staticmethod
    def _find_contours(mask):
        result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return result[0] if len(result) == 2 else result[1]

    def _camera_transform(self, camera_frame, stamp):
        try:
            return self._listener.lookupTransform(self.target_frame, camera_frame, stamp)
        except tf.ExtrapolationException:
            return self._listener.lookupTransform(
                self.target_frame, camera_frame, rospy.Time(0)
            )

    def _pixel_to_table(self, pixel_x, pixel_y, camera_info, transform):
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]
        if fx <= 0.0 or fy <= 0.0:
            return None

        ray_camera = np.array(
            [(pixel_x - cx) / fx, (pixel_y - cy) / fy, 1.0], dtype=np.float64
        )
        translation, quaternion = transform
        rotation = quaternion_matrix(quaternion)[0:3, 0:3]
        ray_target = np.dot(rotation, ray_camera)
        origin = np.asarray(translation, dtype=np.float64)

        if abs(ray_target[2]) < 1.0e-6:
            return None
        scale = (self.projection_plane_z - origin[2]) / ray_target[2]
        if scale <= 0.0:
            return None

        point = origin + scale * ray_target
        point[0] += self.position_offset_x
        point[1] += self.position_offset_y
        if not (self.min_x <= point[0] <= self.max_x):
            return None
        if not (self.min_y <= point[1] <= self.max_y):
            return None
        # X/Y come from the visible top surface; publish the physical object
        # centre height so the pose has an unambiguous geometric meaning.
        point[2] = self.object_center_z
        return point

    def _mask_for_color(self, hsv_image, color_config):
        combined = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for hsv_range in color_config.get("ranges", []):
            lower = np.asarray(hsv_range["lower"], dtype=np.uint8)
            upper = np.asarray(hsv_range["upper"], dtype=np.uint8)
            combined = cv2.bitwise_or(combined, cv2.inRange(hsv_image, lower, upper))
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, self._kernel)
        return cv2.morphologyEx(combined, cv2.MORPH_CLOSE, self._kernel)

    def _image_callback(self, image_message):
        with self._lock:
            camera_info = self._camera_info
        if camera_info is None:
            rospy.logwarn_throttle(5.0, "Waiting for hand camera_info")
            return

        try:
            bgr_image = self._bridge.imgmsg_to_cv2(image_message, "bgr8")
            camera_frame = camera_info.header.frame_id or image_message.header.frame_id
            transform = self._camera_transform(camera_frame, image_message.header.stamp)
        except (CvBridgeError, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as error:
            rospy.logwarn_throttle(5.0, "Camera processing unavailable: %s", str(error))
            return

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        debug_image = bgr_image.copy()
        output = DetectedObjectArray()
        output.header.stamp = image_message.header.stamp
        output.header.frame_id = self.target_frame

        for color_name, color_config in self.colors.items():
            mask = self._mask_for_color(hsv_image, color_config)
            draw_color = tuple(int(value) for value in color_config.get("draw_bgr", [255, 255, 255]))
            for contour in self._find_contours(mask):
                area = float(cv2.contourArea(contour))
                if area < self.min_area or area > self.max_area:
                    continue
                x, y, width, height = cv2.boundingRect(contour)
                short_side = float(min(width, height))
                if short_side <= 0.0:
                    continue
                if float(max(width, height)) / short_side > self.max_aspect_ratio:
                    continue
                moments = cv2.moments(contour)
                if abs(moments["m00"]) < 1.0e-6:
                    continue
                pixel_x = int(moments["m10"] / moments["m00"])
                pixel_y = int(moments["m01"] / moments["m00"])
                point = self._pixel_to_table(
                    pixel_x, pixel_y, camera_info, transform
                )
                if point is None:
                    continue

                detected = DetectedObject()
                detected.color = color_name
                detected.pose.position.x = point[0]
                detected.pose.position.y = point[1]
                detected.pose.position.z = point[2]
                detected.pose.orientation.w = 1.0
                detected.contour_area = area
                detected.pixel_x = pixel_x
                detected.pixel_y = pixel_y
                output.objects.append(detected)

                cv2.rectangle(debug_image, (x, y), (x + width, y + height), draw_color, 2)
                label = "{} ({:.2f}, {:.2f})".format(color_name, point[0], point[1])
                cv2.putText(
                    debug_image,
                    label,
                    (x, max(18, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    draw_color,
                    1,
                    cv2.LINE_AA,
                )

        output.objects.sort(key=lambda item: (item.color, -item.contour_area))
        self._detections_publisher.publish(output)
        try:
            debug_message = self._bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_message.header = image_message.header
            self._debug_publisher.publish(debug_message)
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "Cannot publish debug image: %s", str(error))


def main():
    rospy.init_node("color_object_detector")
    ColorObjectDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
