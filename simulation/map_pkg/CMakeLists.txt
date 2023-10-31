cmake_minimum_required(VERSION 3.0.2)
project(map_pkg)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_pub_node src/map_pub_node.cpp)
target_link_libraries(map_pub_node
  ${catkin_LIBRARIES}
)