cmake_minimum_required(VERSION 3.0.2)
project(legged_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES legged_control
#  CATKIN_DEPENDS controller_interface hardware_interface pluginlib roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
