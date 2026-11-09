cmake_minimum_required(VERSION 3.0.2)
project(aubo_huawei_bridge)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/huawei_bridge scripts/huawei_make_command scripts/huawei_dashboard scripts/huawei_tunnel
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE PATTERN "device.json" EXCLUDE
  PATTERN "*.key" EXCLUDE PATTERN "*.pem" EXCLUDE PATTERN "*.crt" EXCLUDE)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY dashboard DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE
  PATTERN "build" EXCLUDE PATTERN ".env*" EXCLUDE)
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
