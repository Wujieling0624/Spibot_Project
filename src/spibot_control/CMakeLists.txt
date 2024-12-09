cmake_minimum_required(VERSION 3.0.2)
project(spibot_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS 
  controller_manager
  Eigen3
  genmsg
  geometry_msgs
  gazebo_ros
  gazebo_msgs
  joint_state_controller
  nav_msgs
  robot_state_publisher
  roscpp
  rospy
  std_msgs
  sensor_msgs  
  tf
  tf2
  tf2_msgs
  tf2_geometry_msgs
  tf2_ros
)
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spibot_control
#  CATKIN_DEPENDS controller_manager genmsg joint_state_controller robot_state_publisher roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/include  # 你的头文件目录  
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # /usr/include/eigen3           # Eigen 通常安装在这里  
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# 添加C库（可选，如果你打算在多个节点中使用这些函数）  
add_library(spibot_trajectory SHARED src/move_trajectory.cpp)  
target_include_directories(spibot_trajectory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)  

add_library(spibot_trajectory_with_constraint SHARED src/move_trajectory_constraints.cpp)  
target_include_directories(spibot_trajectory_with_constraint PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)  

add_library(baselink_twist SHARED src/baselink_twist.cpp)  
target_include_directories(baselink_twist PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)  

# 添加C++可执行文件（ROS节点）  
add_executable(spibot_joint_move_noetic src/joint_publisher.cpp)
add_executable(spibot_move_noetic src/joint_publisher_constrains.cpp)

# 链接C库到你的ROS节点 
target_link_libraries(spibot_joint_move_noetic ${catkin_LIBRARIES} spibot_trajectory)
target_link_libraries(spibot_move_noetic ${catkin_LIBRARIES} spibot_trajectory_with_constraint baselink_twist)

# 设置包含目录以找到C头文件  
target_include_directories(spibot_joint_move_noetic PRIVATE ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src)  
target_include_directories(spibot_move_noetic PRIVATE ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src)  

add_executable(urdf_transformer src/urdf_transformer.cpp)  
target_link_libraries(urdf_transformer ${catkin_LIBRARIES})
target_include_directories(urdf_transformer PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)  

add_executable(apply_body_wrench_force_node src/apply_body_wrench_force.cpp)
target_link_libraries(apply_body_wrench_force_node ${catkin_LIBRARIES})

add_executable(odom_pub src/odom_publisher.cpp)
target_link_libraries(odom_pub ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/teleop_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
