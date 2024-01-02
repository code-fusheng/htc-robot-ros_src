cmake_minimum_required(VERSION 3.0.2)
project(ndt_localizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  pcl_conversions
  pcl_ros
  autoware_msgs 
  ndt_cpu
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS 
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  pcl_conversions
  pcl_ros
  autoware_msgs
  ndt_cpu
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# add_executable(htc_ndt_localizer src/ndt_localizer.cpp  src/ndt_localizer_node.cpp)
add_executable(htc_ndt_localizer src/yunle_ndt_localizer.cpp)
target_link_libraries(htc_ndt_localizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(htc_ndt_localizer
  ndt_cpu pcl_omp_registration
)
add_dependencies(htc_ndt_localizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)