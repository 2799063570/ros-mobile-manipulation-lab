#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash; source '${WORKSPACE_DIR}/devel/setup.bash'; exec roslaunch wheeltec_multi navigation.launch"
sleep 10

wait
exit 0
