cmake_minimum_required(VERSION 3.5)
project(aubo_ros_control)

set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  aubo_sdk
  controller_manager
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  kdl_parser
  pluginlib
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  rviz
)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

generate_dynamic_reconfigure_options(
  cfg/VisualServo.cfg
)

# aubo_sdk exports its source/install resource root through a catkin CMake
# extra, so this package no longer depends on a particular directory layout.
if(NOT AUBO_SDK_RESOURCE_ROOT)
  message(FATAL_ERROR "aubo_sdk did not export AUBO_SDK_RESOURCE_ROOT")
endif()
set(AUBO_SDK_ROOT "${AUBO_SDK_RESOURCE_ROOT}")
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
  LIBRARIES ${PROJECT_NAME} aubo_visual_servo_panel
  CATKIN_DEPENDS aubo_sdk controller_manager dynamic_reconfigure geometry_msgs hardware_interface
    kdl_parser pluginlib roscpp rospy roslib rviz sensor_msgs
    std_msgs std_srvs tf2_geometry_msgs tf2_ros
  DEPENDS orocos_kdl
)

add_library(${PROJECT_NAME} src/aubo_hardware_interface.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${AUBO_SDK_LIB}
  Threads::Threads)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}"
  INSTALL_RPATH "${AUBO_SDK_LIBRARY_INSTALL_RPATH}"
  # libauborobotcontroller loads libconfig/log4cplus transitively.  GNU ld's
  # default DT_RUNPATH is not used for those indirect dependencies, whereas
  # DT_RPATH is.  Keep the SDK self-contained without requiring users to set
  # LD_LIBRARY_PATH in their shell startup files.
  LINK_FLAGS "-Wl,--disable-new-dtags")

add_executable(aubo_hw_node src/aubo_hw_main.cpp)
target_compile_features(aubo_hw_node PRIVATE cxx_std_14)
target_link_libraries(aubo_hw_node ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(aubo_hw_node PROPERTIES
  BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}"
  INSTALL_RPATH "${AUBO_SDK_NODE_INSTALL_RPATH}"
  LINK_FLAGS "-Wl,--disable-new-dtags")

# Unified visual-servo node.  Camera mounting mode and command backend are
# orthogonal parameters, so all four combinations share one implementation.
add_executable(aubo_visual_servo_node
  src/visual_servo.cpp
  src/visual_servo_common.cpp
  src/direct_sdk_backend.cpp)
add_dependencies(aubo_visual_servo_node ${PROJECT_NAME}_gencfg)
target_compile_features(aubo_visual_servo_node PRIVATE cxx_std_14)
target_include_directories(aubo_visual_servo_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})
target_link_libraries(aubo_visual_servo_node
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${AUBO_SDK_LIB}
  Threads::Threads)
set_target_properties(aubo_visual_servo_node PROPERTIES
  BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}"
  INSTALL_RPATH "${AUBO_SDK_NODE_INSTALL_RPATH}"
  LINK_FLAGS "-Wl,--disable-new-dtags")

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(visual_servo_common_test
    test/visual_servo_common_test.cpp
    src/visual_servo_common.cpp)
  if(TARGET visual_servo_common_test)
    target_compile_features(visual_servo_common_test PRIVATE cxx_std_14)
    target_include_directories(visual_servo_common_test PRIVATE include)
  endif()
endif()

add_library(aubo_visual_servo_panel
  include/aubo_ros_control/visual_servo_panel.h
  src/visual_servo_panel.cpp)
target_compile_features(aubo_visual_servo_panel PRIVATE cxx_std_14)
target_include_directories(aubo_visual_servo_panel PRIVATE
  include
  ${catkin_INCLUDE_DIRS})
target_link_libraries(aubo_visual_servo_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets)

install(TARGETS ${PROJECT_NAME} aubo_hw_node aubo_visual_servo_node
                aubo_visual_servo_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY cfg config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES VISUAL_SERVO.md plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
