#! /bin/bash

gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/wheeltec/wheeltec_robot/devel/setup.bash;roslaunch turn_on_wheeltec_robot 3d_navigation.launch"

wait
exit 0

