#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_VERSION="${WHEELTEC_ROS_DISTRO:-${ROS_DISTRO:-}}"

if [[ -z "${ROS_VERSION}" ]]; then
  if [[ -f /opt/ros/noetic/setup.bash ]]; then
    ROS_VERSION="noetic"
  else
    ROS_VERSION="melodic"
  fi
fi

gnome-terminal -- bash -c "source '/opt/ros/${ROS_VERSION}/setup.bash'; source '${WORKSPACE_DIR}/devel/setup.bash'; exec roslaunch wheeltec_multi navigation.launch"
sleep 10

wait
exit 0
