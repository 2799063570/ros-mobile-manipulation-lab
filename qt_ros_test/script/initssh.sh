#! /bin/bash

current_path=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
#gnome-terminal -- bash -c "ssh -Y wheeltec@192.168.0.100 'source /opt/ros/melodic/setup.bash; roscore' "
python "${current_path}/initssh.py"
sleep 1
gnome-terminal -- bash -c "ssh -Y wheeltec@192.168.0.100 'source /opt/ros/melodic/setup.bash; roscore' "
sleep 5

wait
exit 0

