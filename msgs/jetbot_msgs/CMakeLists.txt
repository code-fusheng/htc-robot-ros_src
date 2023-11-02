cmake_minimum_required(VERSION 2.8.3)

project(jetbot_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        # geometry_msgs
        # sensor_msgs
        message_generation
        # jsk_recognition_msgs
        )

add_message_files(
        FILES
        BoundingBox.msg
        BoundingBoxes.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        # sensor_msgs
        # geometry_msgs
        # jsk_recognition_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        # geometry_msgs
        # sensor_msgs
        # jsk_recognition_msgs
)



