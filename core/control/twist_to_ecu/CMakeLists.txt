cmake_minimum_required(VERSION 3.0.2)
project(twist_to_ecu)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  can_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES twist_to_ecu
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_to_ecu src/twist_to_ecu.cpp src/twist_to_ecu_node.cpp)
target_link_libraries(twist_to_ecu ${catkin_LIBRARIES})
add_dependencies(twist_to_ecu ${catkin_EXPORTED_TARGETS})