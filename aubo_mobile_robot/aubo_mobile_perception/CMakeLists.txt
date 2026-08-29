cmake_minimum_required(VERSION 3.0.2)
project(aubo_mobile_perception)

find_package(catkin REQUIRED COMPONENTS aubo_perception)
catkin_package(CATKIN_DEPENDS aubo_perception)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
