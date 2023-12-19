# CMakeLists.txt

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(pcd_map_mix)

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
)


catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_map_mix src/pcd_map_mix.cpp)
target_link_libraries(pcd_map_mix ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcd_map_mix ${catkin_EXPORTED_TARGETS})