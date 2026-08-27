cmake_minimum_required(VERSION 3.0.2)
project(aubo_sorting_core)

find_package(catkin REQUIRED COMPONENTS
  aubo_perception
  rospy
)

catkin_package(CATKIN_DEPENDS aubo_perception rospy)

catkin_install_python(PROGRAMS
  scripts/color_sorting_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
