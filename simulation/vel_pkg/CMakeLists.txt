cmake_minimum_required(VERSION 3.0.2)
project(vel_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vel_node src/vel_node.cpp)
target_link_libraries(vel_node
  ${catkin_LIBRARIES}
)