cmake_minimum_required(VERSION 3.0.2)
project(demo_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  Demo.msg
)

# 依赖的消息类型
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs message_generation message_runtime
)
