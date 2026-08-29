#! /bin/bash

current_path=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
ros_version="${WHEELTEC_ROS_DISTRO:-${ROS_DISTRO:-}}"
if [[ -z "${ros_version}" ]]; then
  if [[ -f /opt/ros/noetic/setup.bash ]]; then
    ros_version="noetic"
  else
    ros_version="melodic"
  fi
fi
#gnome-terminal -- bash -c "ssh -Y wheeltec@192.168.0.100 'source /opt/ros/melodic/setup.bash; roscore' "
"${WHEELTEC_PYTHON_EXECUTABLE:-python}" "${current_path}/initssh.py"
sleep 1
gnome-terminal -- bash -c "ssh -Y wheeltec@192.168.0.100 'source /opt/ros/${ros_version}/setup.bash; exec roscore'"
sleep 5

wait
exit 0

