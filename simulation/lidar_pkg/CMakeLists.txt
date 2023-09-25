cmake_minimum_required(VERSION 3.0.2)
project(lidar_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_node src/lidar_node.cpp)
target_link_libraries(lidar_node
  ${catkin_LIBRARIES}
)

add_executable(lidar_behavior_node src/lidar_behavior_node.cpp)
target_link_libraries(lidar_behavior_node
  ${catkin_LIBRARIES}
)
