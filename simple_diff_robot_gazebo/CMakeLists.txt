cmake_minimum_required(VERSION 3.0.2)
project(simple_diff_robot_gazebo)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/keyboard_teleop.py
  scripts/single_obstacle_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch maps urdf worlds rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md TROUBLESHOOTING.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
