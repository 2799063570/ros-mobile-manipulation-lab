#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mount-independent RGB-D color target frontend for AUBO visual servo.

The detector-specific part ends at a 2-D candidate (label, mask and centre).
Depth projection then publishes the stable public contract:

    /visual_servo/target_pose  geometry_msgs/PoseStamped

A future Torch/YOLO frontend should publish the same contract, or reuse the
Candidate/deprojection portion of this node.  The servo controller therefore
does not depend on OpenCV, class names, bounding-box messages or a YOLO package.
"""

from __future__ import print_function

import threading

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


class Candidate(object):
    """Detector-neutral 2-D result used by the common RGB-D projector."""

    def __init__(self, label, contour, mask, center, area, draw_bgr):
        self.label = label
        self.contour = contour
        self.mask = mask
        self.center = center
        self.area = float(area)
        self.draw_bgr = tuple(int(value) for value in draw_bgr)
        self.depth = None


class HsvColorFrontend(object):
    """Current OpenCV frontend; replaceable by a YOLO frontend later."""

    def __init__(self, colors, kernel_size, minimum_area, maximum_area,
                 maximum_aspect_ratio, minimum_fill_ratio,
                 image_border_margin, ignored_regions):
        self.colors = colors
        self.minimum_area = minimum_area
        self.maximum_area = maximum_area
        self.maximum_aspect_ratio = maximum_aspect_ratio
        self.minimum_fill_ratio = minimum_fill_ratio
        self.image_border_margin = max(0, int(image_border_margin))
        self.ignored_regions = []
        for index, region in enumerate(ignored_regions):
            try:
                x_min = float(region["x_min"])
                y_min = float(region["y_min"])
                x_max = float(region["x_max"])
                y_max = float(region["y_max"])
            except (KeyError, TypeError, ValueError):
                raise rospy.ROSInitException(
                    "ignored_regions[{}] must contain numeric x_min, y_min, "
                    "x_max and y_max".format(index)
                )
            if not (0.0 <= x_min < x_max <= 1.0 and
                    0.0 <= y_min < y_max <= 1.0):
                raise rospy.ROSInitException(
                    "ignored_regions[{}] coordinates must satisfy "
                    "0 <= min < max <= 1".format(index)
                )
            self.ignored_regions.append((x_min, y_min, x_max, y_max))
        kernel_size = max(1, int(kernel_size))
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

    @staticmethod
    def _find_contours(mask):
        result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return result[0] if len(result) == 2 else result[1]

    def ignored_pixel_regions(self, image_shape):
        height, width = image_shape[:2]
        regions = []
        for x_min, y_min, x_max, y_max in self.ignored_regions:
            left = max(0, min(width, int(round(x_min * width))))
            top = max(0, min(height, int(round(y_min * height))))
            right = max(0, min(width, int(round(x_max * width))))
            bottom = max(0, min(height, int(round(y_max * height))))
            if right > left and bottom > top:
                regions.append((left, top, right, bottom))
        return regions

    def detect(self, bgr_image):
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        image_height, image_width = hsv_image.shape[:2]
        ignored_regions = self.ignored_pixel_regions(hsv_image.shape)
        candidates = []
        for label, configuration in self.colors.items():
            color_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
            for hsv_range in configuration.get("ranges", []):
                lower = np.asarray(hsv_range["lower"], dtype=np.uint8)
                upper = np.asarray(hsv_range["upper"], dtype=np.uint8)
                color_mask = cv2.bitwise_or(
                    color_mask, cv2.inRange(hsv_image, lower, upper)
                )
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, self.kernel)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, self.kernel)
            for left, top, right, bottom in ignored_regions:
                color_mask[top:bottom, left:right] = 0

            for contour in self._find_contours(color_mask):
                area = float(cv2.contourArea(contour))
                if area < self.minimum_area or area > self.maximum_area:
                    continue
                x, y, width, height = cv2.boundingRect(contour)
                margin = self.image_border_margin
                if (x <= margin or y <= margin or
                        x + width >= image_width - margin or
                        y + height >= image_height - margin):
                    continue
                short_side = float(min(width, height))
                if short_side <= 0.0:
                    continue
                if float(max(width, height)) / short_side > self.maximum_aspect_ratio:
                    continue
                fill_ratio = area / float(width * height)
                if fill_ratio < self.minimum_fill_ratio:
                    continue
                moments = cv2.moments(contour)
                if abs(moments["m00"]) < 1.0e-9:
                    continue
                center = (
                    int(round(moments["m10"] / moments["m00"])),
                    int(round(moments["m01"] / moments["m00"])),
                )
                contour_mask = np.zeros(color_mask.shape, dtype=np.uint8)
                cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
                candidates.append(
                    Candidate(
                        label,
                        contour,
                        contour_mask,
                        center,
                        area,
                        configuration.get("draw_bgr", [255, 255, 255]),
                    )
                )
        return candidates


class RgbdVisualTargetNode(object):
    def __init__(self):
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~aligned_depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.target_pose_topic = rospy.get_param(
            "~target_pose_topic", "/visual_servo/target_pose"
        )
        self.debug_image_topic = rospy.get_param(
            "~debug_image_topic", "/visual_servo/debug_image"
        )
        self.target_label_topic = rospy.get_param(
            "~target_label_topic", "/visual_servo/target_label"
        )
        self.target_selection_topic = rospy.get_param(
            "~target_selection_topic", "/visual_servo/target_selection"
        )
        self.perception_state_topic = rospy.get_param(
            "~perception_state_topic", "/visual_servo/perception_state"
        )
        self.target_label = rospy.get_param("~target_label", "red")
        self.enabled = bool(rospy.get_param("~start_enabled", False))
        self.selection_policy = rospy.get_param("~selection_policy", "largest")
        self.minimum_depth = float(rospy.get_param("~minimum_depth", 0.10))
        self.maximum_depth = float(rospy.get_param("~maximum_depth", 2.50))
        self.depth_scale_16u = float(rospy.get_param("~depth_scale_16u", 0.001))
        self.depth_mask_erosion = int(rospy.get_param("~depth_mask_erosion", 3))
        self.maximum_projected_contour_area = float(
            rospy.get_param("~maximum_projected_contour_area", 0.0)
        )
        self.position_filter_alpha = float(
            rospy.get_param("~position_filter_alpha", 0.35)
        )
        self.position_filter_alpha = min(1.0, max(0.01, self.position_filter_alpha))
        self.filter_reset_timeout = float(rospy.get_param("~filter_reset_timeout", 0.50))
        self.sync_queue_size = int(rospy.get_param("~sync_queue_size", 8))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.08))

        if self.selection_policy not in ("largest", "image_center", "nearest"):
            raise rospy.ROSInitException(
                "selection_policy must be largest, image_center or nearest"
            )
        colors = rospy.get_param("~colors")
        self.frontend = HsvColorFrontend(
            colors,
            rospy.get_param("~morphology_kernel", 5),
            float(rospy.get_param("~minimum_contour_area", 180.0)),
            float(rospy.get_param("~maximum_contour_area", 50000.0)),
            float(rospy.get_param("~maximum_aspect_ratio", 2.5)),
            float(rospy.get_param("~minimum_fill_ratio", 0.55)),
            int(rospy.get_param("~image_border_margin", 4)),
            rospy.get_param("~ignored_regions", []),
        )

        self.bridge = CvBridge()
        self.control_lock = threading.RLock()
        self.camera_info = None
        self.filtered_position = None
        self.filtered_label = None
        self.filtered_stamp = rospy.Time(0)
        self.camera_info_lock = threading.Lock()
        self.depth_kernel = np.ones(
            (max(1, self.depth_mask_erosion), max(1, self.depth_mask_erosion)),
            dtype=np.uint8,
        )

        self.pose_publisher = rospy.Publisher(
            self.target_pose_topic, PoseStamped, queue_size=1
        )
        self.label_publisher = rospy.Publisher(
            self.target_label_topic, String, queue_size=1
        )
        self.debug_publisher = rospy.Publisher(
            self.debug_image_topic, Image, queue_size=1
        )
        self.state_publisher = rospy.Publisher(
            self.perception_state_topic, String, queue_size=1, latch=True
        )
        self.selection_subscriber = rospy.Subscriber(
            self.target_selection_topic, String, self._selection_callback, queue_size=1
        )
        self.enable_service = rospy.Service(
            "/visual_servo/perception/set_enabled", SetBool, self._set_enabled
        )
        self.reset_service = rospy.Service(
            "/visual_servo/perception/reset", Trigger, self._reset
        )
        self.info_subscriber = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1
        )
        self.color_subscriber = message_filters.Subscriber(self.color_topic, Image)
        self.depth_subscriber = message_filters.Subscriber(self.depth_topic, Image)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber],
            self.sync_queue_size,
            self.sync_slop,
        )
        self.synchronizer.registerCallback(self._images_callback)
        self._publish_state("SEARCHING" if self.enabled else "DISABLED")

        rospy.loginfo(
            "RGB-D color target: color=%s depth=%s -> %s (label='%s')",
            self.color_topic,
            self.depth_topic,
            self.target_pose_topic,
            self.target_label or "*",
        )

    def _publish_state(self, state, detail=""):
        with self.control_lock:
            label = self.target_label or "any"
        fields = [state, label]
        if detail:
            fields.append(detail)
        self.state_publisher.publish(String(data="|".join(fields)))

    def _clear_filter(self):
        with self.control_lock:
            self.filtered_position = None
            self.filtered_label = None
            self.filtered_stamp = rospy.Time(0)

    def _selection_callback(self, message):
        requested = message.data.strip().lower()
        if requested == "any":
            requested = ""
        if requested and requested not in self.frontend.colors:
            rospy.logwarn("Ignoring unknown visual target label '%s'", requested)
            return
        with self.control_lock:
            self.target_label = requested
            self._clear_filter()
            enabled = self.enabled
        self._publish_state("SEARCHING" if enabled else "DISABLED")

    def _set_enabled(self, request):
        with self.control_lock:
            self.enabled = bool(request.data)
            self._clear_filter()
            enabled = self.enabled
        self._publish_state("SEARCHING" if enabled else "DISABLED")
        return SetBoolResponse(
            success=True,
            message=("perception enabled" if enabled else "perception disabled"),
        )

    def _reset(self, _request):
        with self.control_lock:
            self._clear_filter()
            enabled = self.enabled
        self._publish_state("SEARCHING" if enabled else "DISABLED")
        return TriggerResponse(success=True, message="perception filter reset")

    def _camera_info_callback(self, message):
        with self.camera_info_lock:
            self.camera_info = message

    def _depth_metres(self, depth_message):
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        depth = np.asarray(depth)
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) * self.depth_scale_16u
        return depth.astype(np.float32)

    def _candidate_depth(self, candidate, depth_metres):
        mask = candidate.mask
        if self.depth_mask_erosion > 1:
            mask = cv2.erode(mask, self.depth_kernel, iterations=1)
        finite = np.isfinite(depth_metres)
        # Registered depth images legitimately contain NaN/Inf at holes and
        # outside the sensor range.  NumPy evaluates every '&' operand (there
        # is no short-circuit), so comparing those values can emit warnings
        # even though the finite mask later rejects them.
        with np.errstate(invalid="ignore"):
            within_range = (
                (depth_metres >= self.minimum_depth)
                & (depth_metres <= self.maximum_depth)
            )
        valid = (mask > 0) & finite & within_range
        samples = depth_metres[valid]
        if samples.size < 5:
            return None
        # A median over the object interior rejects edge pixels and isolated
        # RealSense holes much better than one centre pixel.
        return float(np.median(samples))

    def _select(self, candidates, image_shape):
        with self.control_lock:
            target_label = self.target_label
        if target_label:
            candidates = [item for item in candidates if item.label == target_label]
        candidates = [item for item in candidates if item.depth is not None]
        if not candidates:
            return None
        if self.selection_policy == "nearest":
            return min(candidates, key=lambda item: item.depth)
        if self.selection_policy == "image_center":
            center_x = 0.5 * image_shape[1]
            center_y = 0.5 * image_shape[0]
            return min(
                candidates,
                key=lambda item: (item.center[0] - center_x) ** 2
                + (item.center[1] - center_y) ** 2,
            )
        return max(candidates, key=lambda item: item.area)

    @staticmethod
    def _project(candidate, camera_info):
        fx = float(camera_info.K[0])
        fy = float(camera_info.K[4])
        cx = float(camera_info.K[2])
        cy = float(camera_info.K[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        pixel_x, pixel_y = candidate.center
        z = candidate.depth
        return ((pixel_x - cx) * z / fx, (pixel_y - cy) * z / fy, z)

    @staticmethod
    def _projected_contour_area(candidate, camera_info):
        """Approximate the contour's camera-facing physical area in m^2."""
        fx = float(camera_info.K[0])
        fy = float(camera_info.K[4])
        if fx <= 0.0 or fy <= 0.0 or candidate.depth is None:
            return None
        return candidate.area * candidate.depth * candidate.depth / (fx * fy)

    def _filter_position(self, position, label, stamp):
        with self.control_lock:
            reset = (
                self.filtered_position is None
                or label != self.filtered_label
                or (stamp - self.filtered_stamp).to_sec() > self.filter_reset_timeout
                or stamp < self.filtered_stamp
            )
            raw = np.asarray(position, dtype=np.float64)
            if reset:
                filtered = raw
            else:
                alpha = self.position_filter_alpha
                filtered = alpha * raw + (1.0 - alpha) * self.filtered_position
            self.filtered_position = filtered
            self.filtered_label = label
            self.filtered_stamp = stamp
            return filtered

    def _publish_debug(self, image, candidates, selected, header):
        for left, top, right, bottom in self.frontend.ignored_pixel_regions(image.shape):
            overlay = image.copy()
            cv2.rectangle(overlay, (left, top), (right - 1, bottom - 1),
                          (40, 40, 40), thickness=-1)
            cv2.addWeighted(overlay, 0.70, image, 0.30, 0.0, image)
            cv2.rectangle(image, (left, top), (right - 1, bottom - 1),
                          (0, 165, 255), thickness=2)
            cv2.putText(
                image,
                "ROBOT MASK",
                (left + 8, min(bottom - 8, top + 22)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )
        for candidate in candidates:
            x, y, width, height = cv2.boundingRect(candidate.contour)
            thickness = 3 if candidate is selected else 1
            cv2.rectangle(
                image, (x, y), (x + width, y + height), candidate.draw_bgr, thickness
            )
            depth_text = "?" if candidate.depth is None else "{:.3f}m".format(candidate.depth)
            cv2.putText(
                image,
                "{} {}".format(candidate.label, depth_text),
                (x, max(18, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                candidate.draw_bgr,
                1,
                cv2.LINE_AA,
            )
        try:
            debug_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            debug_message.header = header
            self.debug_publisher.publish(debug_message)
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "Cannot publish debug image: %s", str(error))

    def _images_callback(self, color_message, depth_message):
        with self.camera_info_lock:
            camera_info = self.camera_info
        if camera_info is None:
            rospy.logwarn_throttle(2.0, "Waiting for color camera_info")
            return
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
            depth_metres = self._depth_metres(depth_message)
        except (CvBridgeError, TypeError, ValueError) as error:
            rospy.logwarn_throttle(2.0, "RGB-D conversion failed: %s", str(error))
            return
        if depth_metres.shape[:2] != bgr_image.shape[:2]:
            rospy.logwarn_throttle(
                2.0, "Depth is not aligned to color: color=%s depth=%s",
                str(bgr_image.shape[:2]), str(depth_metres.shape[:2])
            )
            return
        if (camera_info.width and camera_info.height and
                (camera_info.width != bgr_image.shape[1]
                 or camera_info.height != bgr_image.shape[0])):
            rospy.logwarn_throttle(
                2.0, "camera_info resolution does not match color image"
            )
            return

        candidates = self.frontend.detect(bgr_image)
        for candidate in candidates:
            candidate.depth = self._candidate_depth(candidate, depth_metres)
        if self.maximum_projected_contour_area > 0.0:
            size_filtered_candidates = []
            for candidate in candidates:
                projected_area = self._projected_contour_area(candidate, camera_info)
                if (projected_area is not None and
                        projected_area <= self.maximum_projected_contour_area):
                    size_filtered_candidates.append(candidate)
            candidates = size_filtered_candidates
        selected = self._select(candidates, bgr_image.shape)

        with self.control_lock:
            enabled = self.enabled
        if enabled and selected is not None:
            position = self._project(selected, camera_info)
            if position is not None:
                position = self._filter_position(
                    position, selected.label, color_message.header.stamp
                )
                pose = PoseStamped()
                pose.header.stamp = color_message.header.stamp
                pose.header.frame_id = (
                    camera_info.header.frame_id
                    or color_message.header.frame_id
                    or "camera_color_optical_frame"
                )
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                pose.pose.position.z = position[2]
                pose.pose.orientation.w = 1.0
                self.pose_publisher.publish(pose)
                self.label_publisher.publish(String(data=selected.label))
                self._publish_state(
                    "DETECTED",
                    "x={:.3f},y={:.3f},z={:.3f}".format(*position),
                )
        elif enabled:
            self._publish_state("SEARCHING")
        # Deliberately publish no pose when detection/depth is invalid. The
        # servo watchdog then enters its target-loss state machine.
        self._publish_debug(bgr_image.copy(), candidates, selected, color_message.header)


def main():
    rospy.init_node("rgbd_visual_target")
    RgbdVisualTargetNode()
    rospy.spin()


if __name__ == "__main__":
    main()
