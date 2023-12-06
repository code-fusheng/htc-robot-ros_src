cmake_minimum_required(VERSION 3.0.2)
project(can_test_puber)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  can_module_usb2can
  can_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
#  message_generation
)




catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES can_test_puber
  CATKIN_DEPENDS can_module_usb2can can_msgs roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(test_puber
  src/test_puber.cc
)

add_dependencies(test_puber
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(test_puber
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/twist2ack.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


