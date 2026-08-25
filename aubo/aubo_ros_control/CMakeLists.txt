cmake_minimum_required(VERSION 3.5)
project(aubo_ros_control)

find_package(catkin REQUIRED COMPONENTS
  aubo_sdk
  controller_manager
  geometry_msgs
  hardware_interface
  kdl_parser
  orocos_kdl
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

set(AUBO_SDK_ROOT "${PROJECT_SOURCE_DIR}/../aubo_sdk")
set(AUBO_SDK_INCLUDE_DIR "${AUBO_SDK_ROOT}/include/aubo_driver")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(AUBO_SDK_LIB "${AUBO_SDK_ROOT}/lib/lib64/aubocontroller/libauborobotcontroller.so")
  set(AUBO_SDK_RUNTIME_DIRS
    "${AUBO_SDK_ROOT}/lib/lib64/aubocontroller"
    "${AUBO_SDK_ROOT}/lib/lib64/log4cplus"
    "${AUBO_SDK_ROOT}/lib/lib64/config")
  set(AUBO_SDK_LIBRARY_INSTALL_RPATH
    "\$ORIGIN/../share/aubo_sdk/lib/lib64/aubocontroller;\$ORIGIN/../share/aubo_sdk/lib/lib64/log4cplus;\$ORIGIN/../share/aubo_sdk/lib/lib64/config")
  set(AUBO_SDK_NODE_INSTALL_RPATH
    "\$ORIGIN/../../share/aubo_sdk/lib/lib64/aubocontroller;\$ORIGIN/../../share/aubo_sdk/lib/lib64/log4cplus;\$ORIGIN/../../share/aubo_sdk/lib/lib64/config")
else()
  set(AUBO_SDK_LIB "${AUBO_SDK_ROOT}/lib/lib32/libauborobotcontroller.so")
  set(AUBO_SDK_RUNTIME_DIRS "${AUBO_SDK_ROOT}/lib/lib32")
  set(AUBO_SDK_LIBRARY_INSTALL_RPATH "\$ORIGIN/../share/aubo_sdk/lib/lib32")
  set(AUBO_SDK_NODE_INSTALL_RPATH "\$ORIGIN/../../share/aubo_sdk/lib/lib32")
endif()

if(NOT EXISTS "${AUBO_SDK_LIB}")
  message(FATAL_ERROR "AUBO SDK library not found: ${AUBO_SDK_LIB}")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS aubo_sdk controller_manager geometry_msgs hardware_interface
    kdl_parser orocos_kdl roscpp roslib sensor_msgs std_msgs tf2_geometry_msgs tf2_ros
)

add_library(${PROJECT_NAME} src/aubo_hardware_interface.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${AUBO_SDK_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${AUBO_SDK_LIB}
  Threads::Threads)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}"
  INSTALL_RPATH "${AUBO_SDK_LIBRARY_INSTALL_RPATH}")

add_executable(aubo_hw_node src/aubo_hw_main.cpp)
target_compile_features(aubo_hw_node PRIVATE cxx_std_14)
target_link_libraries(aubo_hw_node ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(aubo_hw_node PROPERTIES
  BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}"
  INSTALL_RPATH "${AUBO_SDK_NODE_INSTALL_RPATH}")

# Unified eye-in-hand visual-servo node.  The same kinematics, limiter and
# bounded queue are used by the Gazebo and direct-SDK backends.
add_executable(eye_in_hand_visual_servo_node src/eye_in_hand_visual_servo_node.cpp)
target_compile_features(eye_in_hand_visual_servo_node PRIVATE cxx_std_14)
target_include_directories(eye_in_hand_visual_servo_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${AUBO_SDK_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(eye_in_hand_visual_servo_node
  ${catkin_LIBRARIES}
  ${AUBO_SDK_LIB}
  Threads::Threads)
set_target_properties(eye_in_hand_visual_servo_node PROPERTIES
  BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}"
  INSTALL_RPATH "${AUBO_SDK_NODE_INSTALL_RPATH}")

install(TARGETS ${PROJECT_NAME} aubo_hw_node eye_in_hand_visual_servo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES VISUAL_SERVO.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
