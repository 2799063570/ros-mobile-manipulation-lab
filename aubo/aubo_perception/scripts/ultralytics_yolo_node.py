#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 图像到 YOLO 检测框；避免在 Conda 推理进程中加载 cv_bridge。"""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs.msg import Image

from aubo_perception.msg import YoloDetection, YoloDetectionArray


def image_message_to_bgr(message: Image) -> np.ndarray:
    """直接解码常见 8 位图像，避免 ROS 与 Conda 的 OpenCV ABI 冲突。"""
    if message.width <= 0 or message.height <= 0:
        raise ValueError("image width and height must be positive")
    encoding = message.encoding.lower()
    channel_counts = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
        "8uc1": 1,
        "8uc3": 3,
        "8uc4": 4,
    }
    if encoding not in channel_counts:
        raise ValueError("unsupported image encoding: {}".format(message.encoding))
    channels = channel_counts[encoding]
    packed_width = message.width * channels
    if message.step < packed_width:
        raise ValueError("image step is smaller than its packed row width")

    flat = np.frombuffer(message.data, dtype=np.uint8)
    required = message.height * message.step
    if flat.size < required:
        raise ValueError("image data is shorter than height * step")
    # step 包含每行的对齐填充；先按步长分行，再剔除填充字节。
    rows = flat[:required].reshape(message.height, message.step)
    image = rows[:, :packed_width].reshape(message.height, message.width, channels)
    if channels == 1:
        image = np.repeat(image, 3, axis=2)
    elif encoding in ("rgb8", "rgba8"):
        image = image[:, :, (2, 1, 0)]
    elif channels == 4:
        image = image[:, :, :3]
    return np.ascontiguousarray(image)


def bgr_to_image_message(image: np.ndarray, source: Image) -> Image:
    output = Image()
    output.header = source.header
    output.height, output.width = image.shape[:2]
    output.encoding = "bgr8"
    output.is_bigendian = 0
    output.step = output.width * 3
    output.data = np.ascontiguousarray(image, dtype=np.uint8).tobytes()
    return output


class UltralyticsYoloNode:
    def __init__(self) -> None:
        self.project_path = Path(
            rospy.get_param(
                "~project_path", "/home/zlab/deepL/code/ultralytics-main-modify"
            )
        ).expanduser().resolve()
        self.model_path = Path(
            rospy.get_param("~model_path", str(self.project_path / "weights/GC-yolo.pt"))
        ).expanduser().resolve()
        self.input_mode = rospy.get_param("~input_mode", "topic")
        if self.input_mode not in ("topic", "image"):
            raise ValueError("input_mode must be topic or image")
        self.confidence = float(rospy.get_param("~confidence", 0.25))
        self.iou = float(rospy.get_param("~iou", 0.70))
        self.image_size = int(rospy.get_param("~image_size", 640))
        self.device = str(rospy.get_param("~device", "cpu"))
        self.max_detections = int(rospy.get_param("~max_detections", 100))
        self.publish_annotated = bool(rospy.get_param("~publish_annotated_image", True)) # 是否发布标注后的图像
        inference_rate = float(rospy.get_param("~inference_rate", 10.0))# 推理频率，单位为Hz
        self.minimum_interval = 0.0 if inference_rate <= 0 else 1.0 / inference_rate
        self.last_inference_time = 0.0
        self.inference_lock = threading.Lock()

        if not self.project_path.is_dir():
            raise RuntimeError("YOLO project directory does not exist: {}".format(self.project_path))
        if not self.model_path.is_file():
            raise RuntimeError("YOLO model does not exist: {}".format(self.model_path))
        project_string = str(self.project_path)
        if project_string not in sys.path:
            sys.path.insert(0, project_string)
        os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/ultralytics_ros")    # 设置YOLO的环境
        from ultralytics import YOLO

        rospy.loginfo("Loading YOLO model %s on %s", self.model_path, self.device)
        self.model = YOLO(str(self.model_path))

        detections_topic = rospy.get_param("~detections_topic", "/yolo/detections")
        annotated_topic = rospy.get_param("~annotated_image_topic", "/yolo/annotated_image")
        image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.detections_publisher = rospy.Publisher(
            detections_topic, YoloDetectionArray, queue_size=1, latch=self.input_mode == "image"
        )
        self.annotated_publisher = rospy.Publisher(annotated_topic, Image, queue_size=1, latch=self.input_mode == "image")
        if self.input_mode == "topic":
            self.image_subscriber = rospy.Subscriber(
                image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        else:
            # 单张本地图片仅用于 2D 离线检测；零时间戳防止与实时深度错误配对。
            import cv2
            path = Path(rospy.get_param("~image_path", "")).expanduser()
            image = cv2.imread(str(path)) if path.is_file() else None
            if image is None:
                raise ValueError("cannot read image_path: {}".format(path))
            source = Image()
            source.header.frame_id = rospy.get_param("~image_frame", "offline_image")
            self.image_callback(bgr_to_image_message(image, source))
        rospy.loginfo("Ultralytics YOLO input=%s -> %s", self.input_mode, detections_topic)

    @staticmethod
    def _name(result, class_id: int) -> str:
        if isinstance(result.names, dict):
            return str(result.names.get(class_id, class_id))
        if 0 <= class_id < len(result.names):
            return str(result.names[class_id])
        return str(class_id)

    def _new_detection(self, result, row, class_id, confidence, angle=0.0, oriented=False):
        detection = YoloDetection()
        detection.class_id = int(class_id)
        detection.class_name = self._name(result, detection.class_id)
        detection.confidence = float(confidence)
        detection.center_x, detection.center_y = float(row[0]), float(row[1])
        detection.width, detection.height = float(row[2]), float(row[3])
        detection.angle = float(angle)
        detection.orientation_valid = oriented
        return detection

    def _append_detections(self, output: YoloDetectionArray, result) -> None:
        obb = getattr(result, "obb", None)
        if obb is not None and len(obb):
            rows = obb.xywhr.detach().cpu().numpy() # [x_center, y_center, width, height, angle_rad]
            classes = obb.cls.detach().cpu().numpy().astype(np.int32)
            confidences = obb.conf.detach().cpu().numpy()
            for row, class_id, confidence in zip(rows, classes, confidences):
                output.detections.append(
                    self._new_detection(result, row, class_id, confidence, row[4], True)
                )
            return

        boxes = getattr(result, "boxes", None)
        if boxes is None or not len(boxes):
            return
        rows = boxes.xywh.detach().cpu().numpy()
        classes = boxes.cls.detach().cpu().numpy().astype(np.int32)
        confidences = boxes.conf.detach().cpu().numpy()
        for row, class_id, confidence in zip(rows, classes, confidences):
            output.detections.append(
                self._new_detection(result, row, class_id, confidence)
            )

    def image_callback(self, message: Image) -> None:
        # 用单调时钟限制推理频率；非阻塞锁避免多个相机回调并发使用同一模型。
        now = time.monotonic()
        if now - self.last_inference_time < self.minimum_interval:
            return
        if not self.inference_lock.acquire(False):
            return
        self.last_inference_time = now
        try:
            image = image_message_to_bgr(message)
            result = self.model.predict(
                source=image,
                conf=self.confidence,
                iou=self.iou,
                imgsz=self.image_size,
                device=self.device,
                max_det=self.max_detections,
                verbose=False,
            )[0]
            output = YoloDetectionArray()
            # 保留采集时间，几何节点依此查找深度缓存，不能改成推理完成时间。
            output.header = message.header
            self._append_detections(output, result) # 获取检测结果
            self.detections_publisher.publish(output)
            if self.publish_annotated and (self.input_mode == "image" or self.annotated_publisher.get_num_connections()):
                self.annotated_publisher.publish(bgr_to_image_message(result.plot(), message))
        except Exception as error:
            rospy.logerr_throttle(2.0, "YOLO inference failed: %s", str(error))
            if self.input_mode == "image":
                raise
        finally:
            self.inference_lock.release()


def main() -> None:
    rospy.init_node("ultralytics_yolo")
    try:
        UltralyticsYoloNode()
    except Exception as error:
        rospy.logfatal("Cannot start Ultralytics YOLO: %s", str(error))
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
