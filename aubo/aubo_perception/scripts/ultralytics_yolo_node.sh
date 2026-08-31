#!/usr/bin/env bash
set -e

PYTHON_EXECUTABLE="$(rosparam get /ultralytics_yolo/python_executable 2>/dev/null || true)"
if [ -z "$PYTHON_EXECUTABLE" ]; then
  PYTHON_EXECUTABLE="/home/zlab/anaconda3/envs/yolo/bin/python"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$PYTHON_EXECUTABLE" "$SCRIPT_DIR/ultralytics_yolo_node.py" "$@"
