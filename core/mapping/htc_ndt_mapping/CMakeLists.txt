cmake_minimum_required(VERSION 3.0.2)
project(htc_ndt_mapping)

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
  std_msgs
  nav_msgs
  tf
  pcl_ros
  sensor_msgs
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS std_msgs  
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(yunle_ndt_mapping src/yunle_ndt_mapping.cpp)
target_link_libraries(yunle_ndt_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(yunle_ndt_mapping ndt_cpu pcl_omp_registration)
add_dependencies(yunle_ndt_mapping ${catkin_EXPORTED_TARGETS})

add_executable(htc_ndt_mapping src/ndt_mapping_node.cpp src/ndt_mapping.cpp)
target_link_libraries(htc_ndt_mapping ${catkin_LIBRARIES})
add_dependencies(htc_ndt_mapping ${catkin_EXPORTED_TARGETS})


