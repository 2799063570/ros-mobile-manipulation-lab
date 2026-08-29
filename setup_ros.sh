#!/usr/bin/env bash
# Source this file to load ROS and the Catkin workspace containing this repo.
# It intentionally does not modify ~/.bashrc or any other user configuration.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 source 加载环境：source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

_wheeltec_src_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  _wheeltec_ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f "/opt/ros/melodic/setup.bash" ]]; then
  _wheeltec_ros_setup="/opt/ros/melodic/setup.bash"
else
  echo "未找到 ROS 环境；期望存在 /opt/ros/melodic/setup.bash。" >&2
  unset _wheeltec_src_dir
  return 1
fi

# Support both layouts:
#   <workspace>/src/setup_ros.sh  (normal Catkin workspace)
#   <workspace>/setup_ros.sh      (repository used as workspace root)
if [[ -f "${_wheeltec_src_dir}/devel/setup.bash" ]]; then
  _wheeltec_workspace_dir="${_wheeltec_src_dir}"
elif [[ -f "${_wheeltec_src_dir}/../devel/setup.bash" ]]; then
  _wheeltec_workspace_dir="$(cd "${_wheeltec_src_dir}/.." && pwd)"
else
  echo "未找到 devel/setup.bash。请先在工作空间根目录运行 catkin_make。" >&2
  echo "当前仓库目录：${_wheeltec_src_dir}" >&2
  unset _wheeltec_src_dir _wheeltec_ros_setup
  return 1
fi

source "${_wheeltec_ros_setup}"
source "${_wheeltec_workspace_dir}/devel/setup.bash"

echo "ROS 工作空间已加载：${_wheeltec_workspace_dir}"
if ! rospack find aubo_sdk >/dev/null 2>&1; then
  echo "警告：当前环境仍未发现 aubo_sdk；请确认已重新运行 catkin_make。" >&2
fi

unset _wheeltec_src_dir _wheeltec_ros_setup _wheeltec_workspace_dir
