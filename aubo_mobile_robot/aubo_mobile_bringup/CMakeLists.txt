cmake_minimum_required(VERSION 3.0.2)
project(aubo_mobile_bringup)

find_package(catkin REQUIRED)
catkin_package()

# 展示布局随包一起安装，install space 与源码工作空间使用相同入口。
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_showcase_launch.py)
endif()
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
