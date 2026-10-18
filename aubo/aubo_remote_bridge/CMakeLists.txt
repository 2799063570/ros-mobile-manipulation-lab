cmake_minimum_required(VERSION 3.0.2)
project(aubo_remote_bridge LANGUAGES C CXX)
# 节点使用C++14；共享C编解码器与STM32保持同一实现。
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs message_generation
  actionlib moveit_msgs moveit_core moveit_ros_planning gazebo_msgs rosgraph_msgs shape_msgs geometry_msgs)
add_message_files(FILES RemoteCommand.msg CommandStatus.msg)
add_service_files(FILES NamedPose.srv)
generate_messages()
catkin_python_setup()
catkin_package(INCLUDE_DIRS include LIBRARIES remote_core CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
  actionlib moveit_msgs moveit_core moveit_ros_planning gazebo_msgs rosgraph_msgs shape_msgs geometry_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(remote_core src/RemoteProtocol.c src/remote_core.cpp)
add_executable(serial_bridge_node src/serial_bridge_node.cpp)
add_executable(named_pose_server src/named_pose_server.cpp)
foreach(target serial_bridge_node named_pose_server)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target} remote_core ${catkin_LIBRARIES})
endforeach()
# Python仅保留无ROS的串口诊断工具，不再作为ROS接收节点。
catkin_install_python(PROGRAMS scripts/serial_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS remote_core serial_bridge_node named_pose_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch worlds config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(named_pose_gazebo_test test/named_pose_gazebo.test test/named_pose_gazebo_test.cpp)
  if(TARGET named_pose_gazebo_test)
    add_dependencies(named_pose_gazebo_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(named_pose_gazebo_test remote_core ${catkin_LIBRARIES})
  endif()
endif()
