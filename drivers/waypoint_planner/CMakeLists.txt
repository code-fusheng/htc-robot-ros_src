cmake_minimum_required(VERSION 2.8.3)
project(waypoint_planner)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(
  catkin REQUIRED COMPONENTS
    astar_search
    autoware_config_msgs
    autoware_msgs
    libwaypoint_follower
    pcl_conversions
    pcl_ros
    roscpp
    roslint
    std_msgs
    tf
    smartcar_msgs
    can_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  astar_avoid
  src/astar_avoid/astar_avoid.cpp
  src/astar_avoid/astar_avoid_node.cpp
)

target_link_libraries(
  astar_avoid
  ${catkin_LIBRARIES}
)

add_dependencies(
  astar_avoid
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS astar_avoid
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
