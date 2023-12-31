cmake_minimum_required(VERSION 2.8.3)
project(can_module_usb2can)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  mmware_msgs
)
find_package(Threads)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES arm64_usb2can
 CATKIN_DEPENDS can_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  link_directories("${PROJECT_SOURCE_DIR}/lib/x86/linux_64")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_32")
  link_directories("${PROJECT_SOURCE_DIR}/lib/x86/linux_32")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  link_directories("${PROJECT_SOURCE_DIR}/lib/arm/64bit")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch32")
  link_directories("${PROJECT_SOURCE_DIR}/lib/arm/32bit")
endif()

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)
# message(${CMAKE_SYSTEM})  # Linux-4.15.0-50-generic

add_executable(can_module_CANalyst_node
              src/usb2can_node.cpp
              src/usb2can_core.cpp 
              src/Msg.cpp
              src/MsgBase.cpp
              src/MsgJD03.cpp
              src/MsgJD01.cpp
              src/MsgNWD.cpp
              )
add_dependencies(can_module_CANalyst_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(can_module_CANalyst_node
  ${catkin_LIBRARIES}
  controlcan  # 将自动从系统库/自定义库中查找libcontrolcan.so
  pthread
)
           