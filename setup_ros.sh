#!/usr/bin/env bash
# Source this file to load ROS and the Catkin workspace containing this repo.
# It intentionally does not modify ~/.bashrc or any other user configuration.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 source 加载环境：source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

_wheeltec_src_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_wheeltec_requested_distro="${WHEELTEC_ROS_DISTRO:-${ROS_DISTRO:-}}"

if [[ -n "${WHEELTEC_ROS_DISTRO:-}" && -n "${ROS_DISTRO:-}" &&
      "${WHEELTEC_ROS_DISTRO}" != "${ROS_DISTRO}" ]]; then
  echo "WHEELTEC_ROS_DISTRO=${WHEELTEC_ROS_DISTRO} 与当前已加载的 ROS_DISTRO=${ROS_DISTRO} 冲突。" >&2
  echo "请打开一个未加载其他 ROS 版本的新终端后重试。" >&2
  unset _wheeltec_src_dir _wheeltec_requested_distro
  return 1
fi

if [[ -n "${_wheeltec_requested_distro}" ]]; then
  case "${_wheeltec_requested_distro}" in
    melodic|noetic) ;;
    *)
      echo "不支持 ROS ${_wheeltec_requested_distro}；仅支持 melodic 和 noetic。" >&2
      unset _wheeltec_src_dir _wheeltec_requested_distro
      return 1
      ;;
  esac
  if [[ ! -f "/opt/ros/${_wheeltec_requested_distro}/setup.bash" ]]; then
    echo "已选择 ROS ${_wheeltec_requested_distro}，但未找到 /opt/ros/${_wheeltec_requested_distro}/setup.bash。" >&2
    unset _wheeltec_src_dir _wheeltec_requested_distro
    return 1
  fi
elif [[ -f "/opt/ros/noetic/setup.bash" ]]; then
  _wheeltec_requested_distro="noetic"
elif [[ -f "/opt/ros/melodic/setup.bash" ]]; then
  _wheeltec_requested_distro="melodic"
else
  echo "未找到受支持的 ROS 环境；期望安装 ROS Melodic 或 Noetic。" >&2
  unset _wheeltec_src_dir
  return 1
fi

_wheeltec_ros_setup="/opt/ros/${_wheeltec_requested_distro}/setup.bash"

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

export WHEELTEC_ROS_DISTRO="${_wheeltec_requested_distro}"
if [[ "${_wheeltec_requested_distro}" == "noetic" ]]; then
  export WHEELTEC_PYTHON_EXECUTABLE="${WHEELTEC_PYTHON_EXECUTABLE:-python3}"
else
  export WHEELTEC_PYTHON_EXECUTABLE="${WHEELTEC_PYTHON_EXECUTABLE:-python}"
fi

echo "ROS ${_wheeltec_requested_distro} 工作空间已加载：${_wheeltec_workspace_dir}"
if ! rospack find aubo_sdk >/dev/null 2>&1; then
  echo "警告：当前环境仍未发现 aubo_sdk；请确认已重新运行 catkin_make。" >&2
fi

unset _wheeltec_src_dir _wheeltec_ros_setup _wheeltec_workspace_dir _wheeltec_requested_distro
