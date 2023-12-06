cmake_minimum_required(VERSION 3.0.2)
project(htc_pcd_map_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(htc_pcd_map_manager
  src/pcd_map_manager_node.cpp
  src/pcd_map_manager.cpp
)
add_dependencies(htc_pcd_map_manager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(htc_pcd_map_manager
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

