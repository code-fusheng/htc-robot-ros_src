cmake_minimum_required(VERSION 2.8.3)
project(can_adapter)

add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
        tf
        pcl_ros
        roscpp
        std_msgs
        geometry_msgs
        smartcar_msgs
        nav_msgs
        can_msgs
        rtk_cloud_msgs
        htcbot_msgs
        )

catkin_package(
        CATKIN_DEPENDS roscpp
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        geometry_msgs
        smartcar_msgs
        nav_msgs
        pcl_ros
        can_msgs
        rtk_cloud_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(can_adapter_node 
               src/can_adapter.cpp
               src/main.cpp
               )
target_link_libraries(can_adapter_node
                     ${catkin_LIBRARIES})
target_include_directories(can_adapter_node PRIVATE
                           ${catkin_INCLUDE_DIRS})
add_dependencies(can_adapter_node ${catkin_EXPORTED_TARGETS})

