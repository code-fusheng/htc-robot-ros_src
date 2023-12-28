cmake_minimum_required(VERSION 2.8.3)

project(htcbot_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        std_srvs
        geometry_msgs
        sensor_msgs
        message_generation
        # jsk_recognition_msgs
        )

add_message_files(
        FILES
        MapPathConf.msg
        ModeSwitch.msg
        GridMapBuild.msg
        MappingConf.msg
)

add_service_files(
        FILES
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
        # jsk_recognition_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        # sensor_msgs
        # jsk_recognition_msgs
)



