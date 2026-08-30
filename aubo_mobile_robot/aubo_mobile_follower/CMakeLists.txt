cmake_minimum_required(VERSION 3.0.2)
project(aubo_mobile_follower)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/prepare_arm_pose.py
  scripts/laser_follower.py
  scripts/color_follower.py
  scripts/line_follower.py
  scripts/follower_path.py
  scripts/target_interactive_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
