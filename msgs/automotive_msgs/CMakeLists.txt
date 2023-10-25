cmake_minimum_required(VERSION 2.8.3)

project(automotive_msgs)

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
        SaveMap.msg
        NDTStat.msg
        ShowGraph.msg
        SavePath.msg
        SaveStation.msg
        SetTarget.msg
        UserCmd.msg
        CmdPoseRecord.msg
        SubwayInfo.msg
        ControlInfo.msg
        CSHSStatus.msg
        CmdSetSubwayinfo.msg
        CmdSetTarget.msg
        CmdVehicleControl.msg
        ErrorStatus.msg
        NDTMappingReq.msg
        NDTMappingRes.msg
        SimpleObstacleDist.msg
        ConfigSubway.msg
        ConfigMapManager.msg
        ConfigTrajSet.msg
        PathRecordingStatus.msg
        GridMapCreateConfig.msg
        GridMapCreateStatus.msg
        PPTWayPointStatus.msg
        CanAdapterStatus.msg
        VideoStreamControl.msg
)

add_service_files(
        FILES
        OnlineMapping.srv
        PPTSet.srv
        SmartcarRunType.srv
        AvoidSwitch.srv
        FunctionSwitch.srv
        SmartcarSolution.srv
        TransPoints.srv
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



