cmake_minimum_required(VERSION 3.5)
project(inspire_gripper)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

find_package(catkin REQUIRED COMPONENTS
  serial
  actionlib
	control_msgs
	diagnostic_msgs
	geometry_msgs
	moveit_msgs
	moveit_ros_perception
	moveit_ros_planning_interface
	roscpp
	rospy
	sensor_msgs
	shape_msgs
	std_msgs
	std_srvs
	tf
	trajectory_msgs
	message_generation
  )

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)
                   
SET(SOURCES ${PROJECT_SOURCE_DIR}/src/gripper_control.cpp
            ${PROJECT_SOURCE_DIR}/src/gripper_control_lib.cpp)
   
SET(HEADERS ${PROJECT_SOURCE_DIR}/include/gripper_control.h)

add_service_files(FILES set_openlimit.srv
                        set_id.srv
                        move_tgt.srv
                        move_max.srv
                        move_min.srv
                        move_minhold.srv
                        get_openlimit.srv
                        get_copen.srv
                        get_state.srv
                        set_es.srv
                        set_param.srv
                        set_frsvd.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS message_runtime roscpp serial tf 
  )

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} inspire_gripper_gencpp)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES})
add_executable(gripper_node src/gripper.cpp)
target_link_libraries(gripper_node ${catkin_LIBRARIES})
add_executable(pick_sth_demo src/pick_sth_demo.cpp)
target_link_libraries(pick_sth_demo ${catkin_LIBRARIES})

add_dependencies(gripper_node ${PROJECT_NAME}_gencpp)
add_dependencies(pick_sth_demo ${PROJECT_NAME}_gencpp)

install(TARGETS inspire_gripper gripper_node pick_sth_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
