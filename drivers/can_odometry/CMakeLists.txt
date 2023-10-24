cmake_minimum_required(VERSION 2.8.3)
project(can_odometry)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_odometry_node
  nodes/can_odometry/can_odometry_node.cpp
  nodes/can_odometry/can_odometry_core.cpp
)

target_link_libraries(can_odometry_node
  ${catkin_LIBRARIES}
)

add_dependencies(can_odometry_node
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS can_odometry_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
