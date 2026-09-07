cmake_minimum_required(VERSION 3.0.2)
project(aubo_sorting)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
