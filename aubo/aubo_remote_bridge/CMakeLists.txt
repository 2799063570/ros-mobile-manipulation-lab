cmake_minimum_required(VERSION 3.0.2)
project(aubo_remote_bridge LANGUAGES C CXX)
# 节点使用C++14；共享C编解码器与STM32保持同一实现。
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs message_generation
  actionlib aubo_ros_control std_srvs moveit_msgs moveit_core moveit_ros_planning gazebo_msgs rosgraph_msgs shape_msgs geometry_msgs)
# Catkin要求Python包配置先于消息生成，确保devel空间正确合并Python模块。
catkin_python_setup()
add_message_files(FILES RemoteCommand.msg CommandStatus.msg)
add_service_files(FILES NamedPose.srv)
generate_messages()
catkin_package(INCLUDE_DIRS include LIBRARIES remote_core CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
  actionlib aubo_ros_control std_srvs moveit_msgs moveit_core moveit_ros_planning gazebo_msgs rosgraph_msgs shape_msgs geometry_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(remote_core src/RemoteProtocol.c src/remote_core.cpp src/RemoteJogProtocol.c src/jog_remote_core.cpp)
add_executable(serial_bridge_node src/serial_bridge_node.cpp)
add_executable(named_pose_server src/named_pose_server.cpp)
add_executable(cartesian_serial_bridge src/cartesian_serial_bridge.cpp)
find_package(Threads REQUIRED)
target_link_libraries(cartesian_serial_bridge Threads::Threads)
foreach(target serial_bridge_node named_pose_server cartesian_serial_bridge)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target} remote_core ${catkin_LIBRARIES})
endforeach()
# Python仅保留无ROS的串口诊断工具，不再作为ROS接收节点。
catkin_install_python(PROGRAMS scripts/serial_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS remote_core serial_bridge_node named_pose_server cartesian_serial_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch worlds config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES src/RemoteProtocol.c src/RemoteJogProtocol.c DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mcu)
install(FILES include/aubo_remote_bridge/RemoteProtocol.h include/aubo_remote_bridge/RemoteJogProtocol.h
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mcu/aubo_remote_bridge)
install(FILES CARTESIAN_SERIAL_PROTOCOL.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(cartesian_remote_core_test test/cartesian_remote_core_test.cpp)
  if(TARGET cartesian_remote_core_test)
    target_link_libraries(cartesian_remote_core_test remote_core ${catkin_LIBRARIES})
  endif()
  catkin_install_python(PROGRAMS test/cartesian_serial_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/cartesian_serial.test DEPENDENCIES cartesian_serial_bridge aubo_cartesian_jog_node)
  add_rostest_gtest(named_pose_gazebo_test test/named_pose_gazebo.test test/named_pose_gazebo_test.cpp)
  if(TARGET named_pose_gazebo_test)
    add_dependencies(named_pose_gazebo_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(named_pose_gazebo_test remote_core ${catkin_LIBRARIES})
  endif()
endif()
