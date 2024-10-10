cmake_minimum_required(VERSION 3.1.3)
project(moveit_config)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/moveit_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   PATTERN "setup_assistant.launch" EXCLUDE)
# install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
