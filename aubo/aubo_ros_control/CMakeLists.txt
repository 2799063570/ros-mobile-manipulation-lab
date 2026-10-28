cmake_minimum_required(VERSION 3.5)
project(aubo_ros_control)

set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  aubo_sdk
  controller_manager
  controller_manager_msgs
  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  kdl_parser
  moveit_msgs
  moveit_core
  moveit_ros_planning
  shape_msgs
  pluginlib
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  visualization_msgs
  rviz
)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_action_files(FILES MoveHome.action)
add_message_files(FILES VisualServoStatus.msg)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

generate_dynamic_reconfigure_options(
  cfg/VisualServo.cfg
)

catkin_install_python(PROGRAMS
  scripts/analyze_moveit_motion.py
  scripts/analyze_hardware_trace.py
  scripts/analyze_motion_timing.py
  scripts/handeye_tf_publisher.py
  scripts/hybrid_planning_scene.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/capture_cartesian_jog_sdk.sh scripts/capture_cartesian_jog_timing.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
  CATKIN_DEPENDS actionlib actionlib_msgs message_runtime aubo_sdk controller_manager controller_manager_msgs diagnostic_msgs dynamic_reconfigure geometry_msgs hardware_interface
    kdl_parser moveit_msgs moveit_core moveit_ros_planning shape_msgs pluginlib roscpp rospy roslib rviz sensor_msgs
    std_msgs std_srvs tf2_geometry_msgs tf2_ros visualization_msgs
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
  src/hybrid_control.cpp
  src/visual_servo_common.cpp
  src/direct_sdk_backend.cpp)
add_dependencies(aubo_visual_servo_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
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

# 点动复用 SDK 连接管理，通过按需生成器供点，不经过视觉伺服的软件队列。
add_executable(aubo_cartesian_jog_node
  src/cartesian_jog.cpp src/direct_sdk_backend.cpp src/visual_servo_common.cpp)
add_dependencies(aubo_cartesian_jog_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(aubo_cartesian_jog_node PRIVATE cxx_std_14)
target_include_directories(aubo_cartesian_jog_node PRIVATE
  include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(aubo_cartesian_jog_node
  ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${AUBO_SDK_LIB} Threads::Threads)
set_target_properties(aubo_cartesian_jog_node PROPERTIES
  BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}"
  INSTALL_RPATH "${AUBO_SDK_NODE_INSTALL_RPATH}"
  LINK_FLAGS "-Wl,--disable-new-dtags")

add_executable(aubo_cartesian_jog_panel src/cartesian_jog_panel.cpp)
add_dependencies(aubo_cartesian_jog_panel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(aubo_cartesian_jog_panel PRIVATE cxx_std_14)
target_include_directories(aubo_cartesian_jog_panel PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(aubo_cartesian_jog_panel ${catkin_LIBRARIES} Qt5::Widgets)

add_executable(aubo_visual_servo_visualization_node src/visual_servo_visualization.cpp)
add_dependencies(aubo_visual_servo_visualization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(aubo_visual_servo_visualization_node PRIVATE cxx_std_14)
target_include_directories(aubo_visual_servo_visualization_node PRIVATE include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(aubo_visual_servo_visualization_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_test(NAME hardware_trace_analysis_test COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/hardware_trace_analysis_test.py)
  catkin_add_gtest(visual_servo_stream_test test/visual_servo_stream_test.cpp)
  if(TARGET visual_servo_stream_test)
    target_compile_features(visual_servo_stream_test PRIVATE cxx_std_14)
    target_include_directories(visual_servo_stream_test PRIVATE include)
  endif()
  add_executable(motion_timing_trace_test test/motion_timing_trace_test.cpp)
  target_compile_features(motion_timing_trace_test PRIVATE cxx_std_14)
  target_include_directories(motion_timing_trace_test PRIVATE include)
  target_link_libraries(motion_timing_trace_test Threads::Threads)
  add_test(NAME motion_timing_trace_test COMMAND motion_timing_trace_test)
  add_test(NAME motion_timing_analysis_test COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/motion_timing_analysis_test.py)
  add_executable(sdk_jog_timing_test test/sdk_jog_timing_test.cpp)
  target_compile_features(sdk_jog_timing_test PRIVATE cxx_std_14)
  target_include_directories(sdk_jog_timing_test PRIVATE include)
  target_link_libraries(sdk_jog_timing_test Threads::Threads)
  add_test(NAME sdk_jog_timing_test COMMAND sdk_jog_timing_test)
  add_executable(sdk_io_diagnostics_test test/sdk_io_diagnostics_test.cpp)
  target_compile_features(sdk_io_diagnostics_test PRIVATE cxx_std_14)
  target_include_directories(sdk_io_diagnostics_test PRIVATE include)
  target_link_libraries(sdk_io_diagnostics_test Threads::Threads)
  add_test(NAME sdk_io_diagnostics_test COMMAND sdk_io_diagnostics_test)
  add_executable(sdk_flow_control_test test/sdk_flow_control_test.cpp)
  target_compile_features(sdk_flow_control_test PRIVATE cxx_std_14)
  target_include_directories(sdk_flow_control_test PRIVATE include)
  target_link_libraries(sdk_flow_control_test Threads::Threads)
  add_test(NAME sdk_flow_control_test COMMAND sdk_flow_control_test)
  add_executable(direct_sdk_backend_feedback_test
    test/direct_sdk_backend_feedback_test.cpp src/direct_sdk_backend.cpp src/visual_servo_common.cpp)
  target_compile_features(direct_sdk_backend_feedback_test PRIVATE cxx_std_14)
  target_include_directories(direct_sdk_backend_feedback_test PRIVATE include ${catkin_INCLUDE_DIRS})
  target_link_libraries(direct_sdk_backend_feedback_test
    ${catkin_LIBRARIES} ${AUBO_SDK_LIB} Threads::Threads)
  set_target_properties(direct_sdk_backend_feedback_test PROPERTIES
    BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}"
    LINK_FLAGS "-Wl,--disable-new-dtags")
  add_test(NAME direct_sdk_backend_feedback_test COMMAND direct_sdk_backend_feedback_test)
  add_executable(sdk_tcp_transport_test test/sdk_tcp_transport_test.cpp)
  target_compile_features(sdk_tcp_transport_test PRIVATE cxx_std_14)
  target_include_directories(sdk_tcp_transport_test PRIVATE include)
  add_test(NAME sdk_tcp_transport_test COMMAND sdk_tcp_transport_test)
  add_executable(cartesian_jog_common_test test/cartesian_jog_common_test.cpp)
  target_compile_features(cartesian_jog_common_test PRIVATE cxx_std_14)
  target_include_directories(cartesian_jog_common_test PRIVATE include)
  add_test(NAME cartesian_jog_common_test COMMAND cartesian_jog_common_test)
  find_package(rostest REQUIRED)
  add_executable(aubo_visual_servo_fake_sdk_node
    src/visual_servo.cpp src/hybrid_control.cpp src/visual_servo_common.cpp test/fake_direct_sdk_backend.cpp)
  add_dependencies(aubo_visual_servo_fake_sdk_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
  target_compile_features(aubo_visual_servo_fake_sdk_node PRIVATE cxx_std_14)
  target_include_directories(aubo_visual_servo_fake_sdk_node PRIVATE
    include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(aubo_visual_servo_fake_sdk_node PRIVATE
    ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${AUBO_SDK_LIB} Threads::Threads)
  set_target_properties(aubo_visual_servo_fake_sdk_node PROPERTIES
    BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}" LINK_FLAGS "-Wl,--disable-new-dtags")
  add_executable(aubo_cartesian_jog_fake_sdk_node
    src/cartesian_jog.cpp test/fake_direct_sdk_backend.cpp src/visual_servo_common.cpp)
  add_dependencies(aubo_cartesian_jog_fake_sdk_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_compile_features(aubo_cartesian_jog_fake_sdk_node PRIVATE cxx_std_14)
  target_include_directories(aubo_cartesian_jog_fake_sdk_node PRIVATE
    include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(aubo_cartesian_jog_fake_sdk_node PRIVATE
    ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${AUBO_SDK_LIB} Threads::Threads)
  set_target_properties(aubo_cartesian_jog_fake_sdk_node PROPERTIES
    BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}" LINK_FLAGS "-Wl,--disable-new-dtags")
  add_rostest_gtest(aubo_hardware_feedback_test
    test/aubo_hardware_feedback.test test/aubo_hardware_feedback_test.cpp)
  if(TARGET aubo_hardware_feedback_test)
    target_compile_features(aubo_hardware_feedback_test PRIVATE cxx_std_14)
    target_include_directories(aubo_hardware_feedback_test PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(aubo_hardware_feedback_test ${PROJECT_NAME} ${catkin_LIBRARIES})
    set_target_properties(aubo_hardware_feedback_test PROPERTIES
      BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}" LINK_FLAGS "-Wl,--disable-new-dtags")
  endif()
  add_rostest_gtest(aubo_hardware_streaming_test
    test/aubo_hardware_streaming.test test/aubo_hardware_streaming_test.cpp)
  if(TARGET aubo_hardware_streaming_test)
    target_compile_features(aubo_hardware_streaming_test PRIVATE cxx_std_14)
    target_include_directories(aubo_hardware_streaming_test PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(aubo_hardware_streaming_test ${PROJECT_NAME} ${catkin_LIBRARIES})
    set_target_properties(aubo_hardware_streaming_test PROPERTIES
      BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}" LINK_FLAGS "-Wl,--disable-new-dtags")
  endif()
  add_rostest_gtest(cartesian_jog_integration_test
    test/cartesian_jog.test test/cartesian_jog_integration_test.cpp)
  if(TARGET cartesian_jog_integration_test)
    target_compile_features(cartesian_jog_integration_test PRIVATE cxx_std_14)
    target_include_directories(cartesian_jog_integration_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(cartesian_jog_integration_test ${catkin_LIBRARIES})
    add_dependencies(cartesian_jog_integration_test aubo_cartesian_jog_node)
  endif()
  add_rostest_gtest(cartesian_jog_ros_control_integration_test
    test/cartesian_jog_ros_control.test test/cartesian_jog_ros_control_integration_test.cpp)
  if(TARGET cartesian_jog_ros_control_integration_test)
    target_compile_features(cartesian_jog_ros_control_integration_test PRIVATE cxx_std_14)
    target_include_directories(cartesian_jog_ros_control_integration_test PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(cartesian_jog_ros_control_integration_test ${catkin_LIBRARIES})
    add_dependencies(cartesian_jog_ros_control_integration_test aubo_cartesian_jog_node)
  endif()
  add_rostest(test/cartesian_jog_real_model.test DEPENDENCIES cartesian_jog_ros_control_integration_test)
  add_rostest(test/cartesian_jog_sdk50.test DEPENDENCIES
    aubo_cartesian_jog_fake_sdk_node cartesian_jog_ros_control_integration_test)
  catkin_install_python(PROGRAMS test/hybrid_control_test.py
    test/visual_servo_stream_test.py
    test/visual_servo_visualization_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/hybrid_control.test)
  add_rostest(test/visual_servo_stream.test DEPENDENCIES aubo_visual_servo_fake_sdk_node)
  add_rostest(test/visual_servo_visualization.test DEPENDENCIES aubo_visual_servo_visualization_node)
  add_rostest(test/initial_observation.test)
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
add_dependencies(aubo_visual_servo_panel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(aubo_visual_servo_panel PRIVATE cxx_std_14)
target_include_directories(aubo_visual_servo_panel PRIVATE
  include
  ${catkin_INCLUDE_DIRS})
target_link_libraries(aubo_visual_servo_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets)

if(CATKIN_ENABLE_TESTING)
  add_rostest_gtest(visual_servo_panel_view_test test/visual_servo_panel_view.test test/visual_servo_panel_view_test.cpp)
  if(TARGET visual_servo_panel_view_test)
    add_dependencies(visual_servo_panel_view_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_compile_features(visual_servo_panel_view_test PRIVATE cxx_std_14)
    target_include_directories(visual_servo_panel_view_test PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(visual_servo_panel_view_test aubo_visual_servo_panel ${catkin_LIBRARIES} Qt5::Widgets)
  endif()
endif()

install(TARGETS ${PROJECT_NAME} aubo_hw_node aubo_visual_servo_node
                aubo_cartesian_jog_node aubo_cartesian_jog_panel
                aubo_visual_servo_panel
                aubo_visual_servo_visualization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY action msg cfg config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES VISUAL_SERVO.md HYBRID_CONTROL.md CARTESIAN_JOG.md plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
