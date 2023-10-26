cmake_minimum_required(VERSION 2.8.3)
project(ndt_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)
find_package(CUDA)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

if (CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
    list(APPEND PCL_OPENMP_PACKAGES ndt_gpu)
endif ()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  rosbag
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  can_msgs
  automotive_msgs
  # ${PCL_OPENMP_PACKAGES}
  # pcl_omp_registration
  # ndt_cpu
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs autoware_msgs tf ${PCL_OPENMP_PACKAGES}
 DEPENDS system_lib PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

string(REPLACE ":" ";" LIBRARY_DIRS $ENV{LD_LIBRARY_PATH})
link_directories(${LIBRARY_DIRS})  # 可以将libgpu_ndt.so加到这个路径里，然后就可以链接到了
link_directories(/usr/lib/ndt_lib)

include_directories(include)
# link_directories(lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ../../common/user_protocol/include
  ../../common/utils/include
)

if (CUDA_FOUND)
  include_directories(
    ${CUDA_INCLUDE_DIRS}
  )
endif ()

add_library(ndt_localization src/ndt_localization.cpp)

target_link_libraries(ndt_localization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_cpu
  pcl_omp_registration
)

if (CUDA_FOUND)
    target_link_libraries(ndt_localization ${CUDA_LIBRARIES} ndt_gpu)
    target_include_directories(ndt_localization PRIVATE ${CUDA_INCLUDE_DIRS})
endif ()


if (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    set_target_properties(ndt_localization PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
endif (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

# general ndt localization
add_executable(ndt_localization_node src/ndt_localization.cpp)

add_dependencies(ndt_localization_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ndt_localization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_localization
)

# sasaki ndt localization

