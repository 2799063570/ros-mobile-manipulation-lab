cmake_minimum_required(VERSION 2.8.3)
project(lslidar_driver)

add_compile_options(-std=c++14)
set( CMAKE_CXX_STANDARD 14)

set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  diagnostic_updater
  nodelet
  pcl_ros
  pcl_conversions
  lslidar_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lslidar_driver
  CATKIN_DEPENDS
    roscpp diagnostic_updater nodelet
    pcl_ros pcl_conversions
    lslidar_msgs
  DEPENDS
    boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# Leishen lidar driver
add_library(lslidar_driver
  src/lslidar_driver.cc
  src/input.cc
  src/lsiosr.cpp

)

target_link_libraries(lslidar_driver
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

add_dependencies(lslidar_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Leishen lidar nodelet
add_executable(lslidar_driver_node
  src/lslidar_driver_node.cc
)
 
target_link_libraries(lslidar_driver_node
  lslidar_driver
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# install options
install(TARGETS lslidar_driver_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

