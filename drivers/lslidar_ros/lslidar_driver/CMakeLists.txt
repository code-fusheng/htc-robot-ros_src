cmake_minimum_required(VERSION 2.8.3)
project(lslidar_driver)

add_compile_options(-std=c++14)
set( CMAKE_CXX_STANDARD 14)

set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS
	${${PROJECT_NAME}_CATKIN_DEPS}
      pcl_conversions
      rospy
		  roscpp
		  pluginlib
		  lslidar_msgs
	    std_msgs
      genmsg
      message_generation
      sensor_msgs
      diagnostic_updater
      nodelet
      pcl_ros
)

find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lslidar_driver
  CATKIN_DEPENDS
    message_runtime std_msgs pluginlib roscpp diagnostic_updater nodelet
    pcl_ros pcl_conversions
    lslidar_msgs
  DEPENDS
    boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_library(
  lslidar_single_input 
  src/single_input.cc
)

add_library(
  lslidar_input 
  src/input.cpp
)

add_library(
  lslidar_single_serial
  src/single_lsiosr.cpp
)

add_library(
  lslidar_driver 
  src/lslidar_driver.cpp
)

# Leishen lidar driver
add_library(
  lslidar_single_driver
  src/lslidar_single_driver.cc
)

target_link_libraries(
  lslidar_driver
	lslidar_input
	${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

target_link_libraries(
  lslidar_single_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

target_link_libraries(
  lslidar_single_serial
  ${catkin_LIBRARIES}
)

add_library(
  lslidar_driver_nodelet 
  src/lslidar_driver_nodelet.cc 
  src/lslidar_driver.cpp
)

target_link_libraries(
  lslidar_driver_nodelet
	lslidar_input
	${catkin_LIBRARIES}
)

target_link_libraries(
  lslidar_single_driver
  lslidar_single_input
  lslidar_single_serial
  ${catkin_LIBRARIES}
)

add_dependencies(
  lslidar_single_driver
  lslidar_single_input
  lslidar_single_serial
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  lslidar_driver_node 
  src/lslidar_driver_node.cpp)

# Leishen lidar nodelet
add_executable(
  lslidar_single_driver_node
  src/lslidar_single_driver_node.cc
)
target_link_libraries(lslidar_single_driver_node
  lslidar_single_driver
  lslidar_single_input
  lslidar_single_serial
  ${catkin_LIBRARIES}
)

if(catkin_EXPORTED_TARGETS)
	add_dependencies(lslidar_input ${catkin_EXPORTED_TARGETS})
endif()

add_dependencies(lslidar_single_driver_node
  lslidar_single_input
  lslidar_single_serial
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    lslidar_driver_node
		lslidar_driver
		lslidar_input
		${catkin_LIBRARIES}
		${libpcap_LIBRARIES}
)

# install options
install(TARGETS lslidar_input lslidar_driver lslidar_driver_nodelet lslidar_driver_node
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

install(TARGETS lslidar_single_input lslidar_single_driver_node lslidar_single_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
		nodelet_lslidar.xml
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)

