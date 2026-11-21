#!/usr/bin/env python3
"""Replay identical images through the ROS YOLO detectors and compare their outputs."""

import argparse
from collections import Counter
from contextlib import contextmanager
import hashlib
from itertools import combinations
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import rosgraph
import rospy
from sensor_msgs.msg import Image
import yaml

from aubo_perception.msg import YoloDetectionArray


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, type=Path, help="Image file or directory")
    parser.add_argument("--model-path", required=True, type=Path, help="GC-YOLO PyTorch checkpoint")
    parser.add_argument("--python", default="/home/zlab/anaconda3/envs/yolo/bin/python")
    parser.add_argument("--fp32-engine", type=Path)
    parser.add_argument("--fp16-engine", type=Path)
    parser.add_argument("--gpu-device", type=int, default=0, help="CUDA GPU index before visibility masking")
    parser.add_argument("--config", type=Path, help="Override ultralytics_yolo.yaml")
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=60, help="Connection and per-frame timeout in seconds")
    parser.add_argument("--min-iou", type=float, default=0.5, help="Rotated IoU for same-class matching")
    parser.add_argument("--output", type=Path, required=True, help="JSON report path")
    args = parser.parse_args(rospy.myargv()[1:])
    if not (args.fp32_engine or args.fp16_engine):
        parser.error("provide --fp32-engine and/or --fp16-engine")
    if not (args.warmup > 0 and args.repeats > 0 and args.gpu_device >= 0 and
            math.isfinite(args.timeout) and args.timeout > 0 and 0 < args.min_iou <= 1):
        parser.error("invalid warmup, repeats, GPU index, timeout or matching IoU")
    return args


def load_frames(source):
    extensions = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}
    paths = [source]
    if source.is_dir():
        paths = sorted(path for path in source.iterdir() if path.is_file() and path.suffix.lower() in extensions)
    if not paths:
        raise ValueError(f"No images found in {source}")
    frames, manifest = [], []
    for path in paths:
        encoded = path.read_bytes()
        pixels = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        if pixels is None:
            raise ValueError(f"Could not read image {path}")
        height, width = pixels.shape[:2]
        rgb = cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB)
        message = Image(height=height, width=width, encoding="rgb8", step=width * 3, data=rgb.tobytes())
        message.header.frame_id = "gc_yolo_benchmark_camera"
        frames.append(message)
        manifest.append({"path": str(path.resolve()), "name": path.name, "width": width, "height": height,
                         "sha256": hashlib.sha256(encoded).hexdigest()})
    return frames, manifest


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as model_file:
        for data in iter(lambda: model_file.read(1024 * 1024), b""):
            digest.update(data)
    return digest.hexdigest()


@contextmanager
def detector(launch, namespace, overrides, log_path, device):
    command = ["roslaunch", "aubo_perception", launch]
    command.extend(f"{key}:={value}" for key, value in overrides.items())
    env = dict(os.environ, ROS_NAMESPACE=namespace, CUDA_VISIBLE_DEVICES="-1" if device == "cpu" else device)
    if launch == "ultralytics_yolo.launch":
        env["NVIDIA_TF32_OVERRIDE"] = "0"
    with log_path.open("w") as log:
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            yield process
        finally:
            if process.poll() is None:
                # roslaunch handles SIGINT and shuts down its detector child.
                process.send_signal(signal.SIGINT)
                process.wait()


def frame_output(detections):
    return [{"class_name": box.class_name, "confidence": box.confidence,
             "x": box.center_x, "y": box.center_y, "width": box.width, "height": box.height,
             "angle": box.angle, "orientation_valid": box.orientation_valid}
            for box in detections]


def measure(launch, overrides, namespace, log_path, device, frames, args):
    responses = queue.Queue()

    def receive(message):
        responses.put((time.perf_counter_ns(), message))

    publisher = rospy.Publisher(namespace + "/image", Image, queue_size=1, tcp_nodelay=True)
    subscriber = rospy.Subscriber(namespace + "/detections", YoloDetectionArray, receive,
                                  queue_size=1, tcp_nodelay=True)
    try:
        with detector(launch, namespace, overrides, log_path, device) as process:
            deadline = time.monotonic() + args.timeout
            while not (publisher.get_num_connections() and subscriber.get_num_connections()):
                if rospy.is_shutdown() or process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError(f"Detector did not connect. See {log_path}")
                time.sleep(0.05)
            samples = []
            count = args.warmup + args.repeats * len(frames)
            for index in range(count):
                frame_index = (index - args.warmup) % len(frames)
                frame = frames[frame_index]
                stamp = rospy.Time(index + 1)
                frame.header.stamp = stamp
                start = time.perf_counter_ns()
                publisher.publish(frame)
                try:
                    completed, message = responses.get(timeout=args.timeout)
                except queue.Empty as error:
                    raise RuntimeError(f"Detector did not answer frame {frame_index}. See {log_path}") from error
                if message.header.stamp != stamp or message.header.frame_id != frame.header.frame_id:
                    raise RuntimeError(f"Detector changed the input header. See {log_path}")
                if index >= args.warmup:
                    samples.append({"frame": frame_index, "latency_ms": (completed - start) / 1e6,
                                    "boxes": frame_output(message.detections)})
            return samples
    finally:
        subscriber.unregister()
        publisher.unregister()


def statistics(values):
    if not values:
        return None
    return {"mean": float(np.mean(values)), "p50": float(np.percentile(values, 50)),
            "p95": float(np.percentile(values, 95))}


def rectangle(box):
    return ((box["x"], box["y"]), (box["width"], box["height"]), math.degrees(box["angle"]))


def rotated_iou(first, second):
    _, intersection = cv2.rotatedRectangleIntersection(rectangle(first), rectangle(second))
    if intersection is None:
        return 0.0
    area = cv2.contourArea(cv2.convexHull(intersection))
    union = first["width"] * first["height"] + second["width"] * second["height"] - area
    return min(1.0, area / union) if union > 0 else 0.0


def long_axis(box):
    if box["width"] >= box["height"]:
        return box["angle"]
    return box["angle"] + math.pi / 2


def elongated(box):
    return box["orientation_valid"] and max(box["width"], box["height"]) > 1.1 * min(box["width"], box["height"])


def agreement(reference, candidate, minimum_iou):
    ious, centers, angles, confidences = [], [], [], []
    missing, extra = 0, 0
    for baseline, accelerated in zip(reference, candidate):
        first, second = baseline["boxes"], accelerated["boxes"]
        pairs = [(rotated_iou(a, b), i, j) for i, a in enumerate(first) for j, b in enumerate(second)
                 if a["class_name"] == b["class_name"]]
        used_first, used_second = set(), set()
        for iou, i, j in sorted(pairs, reverse=True):
            if iou < minimum_iou or i in used_first or j in used_second:
                continue
            used_first.add(i)
            used_second.add(j)
            a, b = first[i], second[j]
            ious.append(iou)
            centers.append(math.hypot(a["x"] - b["x"], a["y"] - b["y"]))
            confidences.append(abs(a["confidence"] - b["confidence"]))
            if elongated(a) and elongated(b):
                angles.append(math.degrees(abs(math.remainder(long_axis(a) - long_axis(b), math.pi))))
        missing += len(first) - len(used_first)
        extra += len(second) - len(used_second)
    return {"matched": len(ious), "reference_only": missing, "candidate_only": extra,
            "rotated_iou": statistics(ious), "center_delta_px": statistics(centers),
            "orientation_count": len(angles), "long_axis_delta_deg": statistics(angles),
            "confidence_delta": statistics(confidences)}


def main():
    args = arguments()
    package = subprocess.check_output(["rospack", "find", "aubo_perception"], text=True).strip()
    config_path = args.config or Path(package) / "config" / "ultralytics_yolo.yaml"
    config = yaml.safe_load(config_path.read_text())
    if not isinstance(config, dict):
        raise ValueError(f"Expected YOLO parameters in {config_path}")
    config.setdefault("rect", False)
    config.update(image_size=640, expected_image_size=640, inference_rate=0.0, publish_annotated_image=False)
    frames, manifest = load_frames(args.images)
    runs = {
        "pytorch_cpu": ("ultralytics_yolo.launch", args.model_path, "cpu"),
        "pytorch_cuda": ("ultralytics_yolo.launch", args.model_path, str(args.gpu_device)),
    }
    for name, path in (("tensorrt_fp32", args.fp32_engine), ("tensorrt_fp16", args.fp16_engine)):
        if path:
            runs[name] = ("tensorrt_yolo.launch", path, str(args.gpu_device))
    model_hashes = {}
    for path in {model for _, model, _ in runs.values()}:
        if not path.is_file():
            raise ValueError(f"Model file does not exist: {path}")
        model_hashes[path] = file_sha256(path)
    if not rosgraph.is_master_online():
        raise RuntimeError("Start roscore before running the benchmark")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rospy.init_node("gc_yolo_benchmark", anonymous=True)
    report = {"images": manifest, "warmup": args.warmup, "repeats": args.repeats,
              "minimum_match_iou": args.min_iou, "config": config, "runs": {}, "agreement": {}}
    with tempfile.TemporaryDirectory(prefix="gc_yolo_benchmark_") as work:
        for name, (launch, model, device) in runs.items():
            print(f"Running {name} on {len(frames)} images", flush=True)
            namespace = rospy.get_name() + "/" + name
            config_path = Path(work) / f"{name}.yaml"
            config_path.write_text(yaml.safe_dump(dict(config, device=device)))
            overrides = dict(config=config_path, image_topic=namespace + "/image",
                             detections_topic=namespace + "/detections",
                             annotated_image_topic=namespace + "/annotated")
            if launch == "ultralytics_yolo.launch":
                overrides.update(input_mode="topic", model_path=model.resolve(), python=args.python)
            else:
                # The selected GPU is the only visible CUDA device for this process.
                overrides.update(engine_path=model.resolve(), gpu_device=0)
            log_path = args.output.with_suffix(f".{name}.log")
            rows = measure(launch, overrides, namespace, log_path, device, frames, args)
            latency = statistics([row["latency_ms"] for row in rows])
            classes = Counter(box["class_name"] for row in rows for box in row["boxes"])
            report["runs"][name] = {"model": str(model.resolve()), "model_sha256": model_hashes[model], "device": device,
                                    "latency_ms": latency, "serial_fps": 1000 / latency["mean"],
                                    "detections_by_class": dict(classes), "samples": rows}
            print(f"  mean {latency['mean']:.2f} ms, P50 {latency['p50']:.2f} ms, "
                  f"P95 {latency['p95']:.2f} ms", flush=True)

    for (reference_name, reference), (name, candidate) in combinations(report["runs"].items(), 2):
        pair = f"{reference_name}_vs_{name}"
        comparison = agreement(reference["samples"], candidate["samples"], args.min_iou)
        comparison["mean_speedup"] = reference["latency_ms"]["mean"] / candidate["latency_ms"]["mean"]
        comparison["p95_speedup"] = reference["latency_ms"]["p95"] / candidate["latency_ms"]["p95"]
        report["agreement"][pair] = comparison
        print(f"{pair}: {comparison['mean_speedup']:.2f}x, "
              f"{comparison['matched']} matched, {comparison['reference_only']}/{comparison['candidate_only']} unmatched")
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
