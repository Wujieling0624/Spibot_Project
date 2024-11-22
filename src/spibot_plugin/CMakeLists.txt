cmake_minimum_required(VERSION 3.0.2)
project(spibot_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  std_msgs
  sensor_msgs
)

# find_package(ignition REQUIRED)

## System dependencies are found with pkg-config  
find_package(PkgConfig REQUIRED gazebo)  

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
)

## Build the suction_cup_gazebo_plugin library  
include_directories(  
  include  
  ${catkin_INCLUDE_DIRS}  
  ${GAZEBO_ROS_INCLUDE_DIRS}
  ${GAZEBO_PLUGINS_INCLUDE_DIRS}
  # /usr/include
  /usr/local/include # Location when using 'make system_install'
  /usr/include
  /usr/include/OGRE/Paging
  /usr/include/OGRE
  /usr/include/gazebo-11
  /usr/include/ignition/math6
  /usr/include/sdformat-9.10
  /usr/include/ignition/common3
  /usr/include/ignition/fuel_tools4
  /usr/include/ignition/msgs5
)  
## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)
  
link_directories(/usr/lib/x86_64-linux-gnu)  # Adjust this path if necessary  


add_library(draw_traj SHARED src/draw_traj_plugin.cc)
target_link_libraries(draw_traj ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(draw_triangle SHARED src/draw_triangle_plugin.cc)
target_link_libraries(draw_triangle ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

