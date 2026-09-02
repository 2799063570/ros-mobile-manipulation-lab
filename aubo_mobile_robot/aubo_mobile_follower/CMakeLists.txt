cmake_minimum_required(VERSION 3.0.2)
project(aubo_mobile_follower)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/ColorFollower.cfg
  cfg/LineFollower.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

catkin_install_python(PROGRAMS
  scripts/prepare_arm_pose.py
  scripts/laser_follower.py
  scripts/color_follower.py
  scripts/line_follower.py
  scripts/follower_path.py
  scripts/target_interactive_marker.py
  scripts/semantic_drive_supervisor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_semantic_drive_core.py)
  catkin_add_nosetests(test/test_image_message.py)
  catkin_add_nosetests(test/test_pid.py)
endif()

install(DIRECTORY config launch models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
