cmake_minimum_required(VERSION 3.0.2)
project(aubo_tencent_bridge)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/tencent_bridge DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config wechat DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "node_modules" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "project.private.config.json" EXCLUDE
  PATTERN "device.json" EXCLUDE
  PATTERN "*.key" EXCLUDE
  PATTERN "*.pem" EXCLUDE
  PATTERN "*.crt" EXCLUDE)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
