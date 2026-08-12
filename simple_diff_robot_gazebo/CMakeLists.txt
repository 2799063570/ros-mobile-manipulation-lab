cmake_minimum_required(VERSION 3.0.2)
project(simple_diff_robot_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch maps urdf worlds rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
