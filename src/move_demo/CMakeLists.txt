cmake_minimum_required(VERSION 3.0.2)
project(move_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
  gazebo_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_msgs nav_msgs tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stm32_odom_publisher src/stm32.cpp)
target_link_libraries(stm32_odom_publisher ${catkin_LIBRARIES})
add_dependencies(stm32_odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})