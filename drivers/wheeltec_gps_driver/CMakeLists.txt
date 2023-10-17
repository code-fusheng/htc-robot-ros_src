cmake_minimum_required(VERSION 2.8.3)
project(wheeltec_gps_driver)

find_package(catkin REQUIRED
	nav_msgs
	roscpp
	rospy
	sensor_msgs
	rosmsg
	geometry_msgs
	)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS roscpp rosmsg rospy nav_msgs geometry_msgs sensor_msgs)

install(PROGRAMS
   scripts/nmea_serial_driver
   scripts/nmea_socket_driver
   scripts/nmea_topic_driver
   scripts/nmea_topic_serial_reader
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

add_executable(gps_path src/gps_path.cpp) 
target_link_libraries(gps_path ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})
if (CATKIN_ENABLE_TESTING)
  find_package(roslint)
  roslint_python()
  roslint_add_test()
endif()
