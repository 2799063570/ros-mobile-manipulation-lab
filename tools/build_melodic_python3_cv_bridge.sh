#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/melodic/setup.bash ]]; then
  echo "未找到 /opt/ros/melodic/setup.bash；此脚本仅用于 Ubuntu 18.04 / ROS Melodic。" >&2
  exit 1
fi

if [[ "${ROS_DISTRO:-}" != "" && "${ROS_DISTRO}" != "melodic" ]]; then
  echo "当前终端已加载 ROS ${ROS_DISTRO}。请打开新终端后再构建 Melodic overlay。" >&2
  exit 1
fi

for command_name in git catkin_make python3; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "缺少命令：${command_name}" >&2
    exit 1
  fi
done

if ! python3 -c 'import catkin_pkg, rospkg, em, yaml, defusedxml' >/dev/null 2>&1; then
  echo "缺少 Melodic Python 3 构建模块。请先执行：" >&2
  echo "  python3 -m pip install --user 'catkin_pkg<1' 'rospkg<2' 'empy==3.3.4' defusedxml pyyaml" >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${src_dir}/.." && pwd)"
overlay_dir="${1:-${WHEELTEC_MELODIC_PY3_OVERLAY:-${workspace_dir}/melodic_py3_cv_bridge_ws}}"

mkdir -p "${overlay_dir}/src"
if [[ ! -d "${overlay_dir}/src/vision_opencv/.git" ]]; then
  if [[ -e "${overlay_dir}/src/vision_opencv" ]]; then
    echo "${overlay_dir}/src/vision_opencv 已存在但不是 Git checkout，请移走后重试。" >&2
    exit 1
  fi
  git clone --depth 1 --single-branch --branch melodic \
    https://github.com/ros-perception/vision_opencv.git \
    "${overlay_dir}/src/vision_opencv"
fi

source /opt/ros/melodic/setup.bash
cd "${overlay_dir}"
catkin_make \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_ENABLE_TESTING=OFF

echo "Melodic Python 3 cv_bridge overlay 已生成：${overlay_dir}"
echo "重新加载项目环境：source ${src_dir}/setup_ros.sh"
