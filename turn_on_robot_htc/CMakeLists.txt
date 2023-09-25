# 最低系统版本
cmake_minimum_required(VERSION 3.0.2)
# 项目工程名
project(turn_on_robot_htc)
# find package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  geometry_msgs
  actionlib
  actionlib_msgs
  nav_msgs
  tf
  message_generation
  roslib
  sensor_msgs
)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME})

# add_executable(hello_robot src/hello_robot.cpp)
# target_link_libraries(hello_robot ${catkin_LIBRARIES})
add_executable(htc_wheeltec_robot_node src/htc_wheeltec_robot.cpp src/Quaternion_Solution.cpp)
target_link_libraries(htc_wheeltec_robot_node ${catkin_LIBRARIES})