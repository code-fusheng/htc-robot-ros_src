#ifndef CAN_ADAPTER_H
#define CAN_ADAPTER_H

#include <can_msgs/ecu.h>
#include <can_msgs/vehicle_status.h>
#include <iostream>
#include <ros/ros.h>
#include <smartcar_msgs/CrossLock.h>
#include <smartcar_msgs/State.h>
#include <automotive_msgs/SimpleObstacleDist.h>
#include <automotive_msgs/PPTWayPointStatus.h>
#include <std_msgs/Int32.h>
// #include <automotive_msgs/SmartcarRunTypeRequest.h>
// #include <automotive_msgs/SmartcarRunTypeResponse.h>
#include <automotive_msgs/SmartcarRunType.h>
#include <automotive_msgs/CanAdapterStatus.h>
#include <ros/service.h>
#include <rtk_cloud_msgs/RTKTaskSet.h>
#include <rtk_cloud_msgs/RTKTaskSetRequest.h>
#include <rtk_cloud_msgs/RTKTaskSetResponse.h>

#include <rtk_cloud_msgs/RTKTaskControl.h>
#include <rtk_cloud_msgs/RTKTaskControlRequest.h>
#include <rtk_cloud_msgs/RTKTaskControlResponse.h>

#include <htcbot_msgs/ModeSwitch.h>

#include <mutex>

namespace can_adapter
{

    class CanAdapterNode
    {
    private:
        // handle
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher pub_ctl;

        ros::Subscriber sub_ecu, sub_status, sub_remote_control, sub_vehicle_status, sub_simple_obstacle_detection;
        ros::Subscriber running_auto_sub;
        ros::Subscriber sub_ppt_waypoints_status;

        double bias, steer_left_param, steer_right_param;
        double lane_speed_limit, cross_speed_limit;
	
        std::string sub_topic;
        std::string pub_topic, sub_status_topic;

	std::string sub_laser_urgent_obs;

        smartcar_msgs::State car_status;
        double cur_speed, cur_steer;
        ros::Time vehicle_status_update_time;

        float front_obstacle_dist, back_obstacle_dist;
        ros::Time front_obstacle_stop_time, back_obstacle_stop_time;
        ros::Time front_obstacle_update_time, back_obstacle_update_time;

        // 斜坡检测
        double lean_obs_dist_rear, lean_obs_dist_remote;
        double th_lean_obs_dist_rear, th_lean_obs_dist_remote;
        ros::Time lean_obs_update_time, lean_obs_stop_time;

        double ratio_fix_speed;
        double front_dist_fix_speed, back_dist_fix_speed;
        double front_dist_stop, back_dist_stop;
        float _obstacle_lock_time = 1.0;  // 由于检测到障碍物所触发的刹车后重启时间

        uint8_t smartcar_run_type;
        ros::ServiceServer request_automotive_type_service, set_automotive_type_service;

        float min_speed_limit = 1.0;         // km/h
        float max_speed_limit = 4.0;         // km/h
        float dist_slow_speed_at_end = 4.0;
        float max_detection_distance = 6.0;  // 单位m 障碍物在该范围内将触发减速

        std::mutex _waypoint_status_lock;
        bool is_current_waypoint_status_valid = false;
        automotive_msgs::PPTWayPointStatus current_waypoint_status;

        void initForROS();

        void publish_brake_switch_and_reason(bool is_running, int reason);

        int brake_check(const can_msgs::ecu &source_msg);

        void callbackFromSelfDriving(const can_msgs::ecu &msg);

        void callbackFromRemoteControl(const can_msgs::ecu &msg);

        void callbackFromStatus(const smartcar_msgs::State &status_msg);

        void callbackFromVehicleStatus(const can_msgs::vehicle_statusPtr &msg);

        void callbackSimpleObsDetect(const automotive_msgs::SimpleObstacleDistConstPtr &msg);

        void callbackUserLaserRange(const automotive_msgs::SimpleObstacleDistConstPtr &msg);

        void callbackFromPPTWaypointStatus(const automotive_msgs::PPTWayPointStatus &msg);

        void handleRunningAutoSwitch(const htcbot_msgs::ModeSwitchConstPtr &msg);

        bool onServiceRequestSmartcarType(automotive_msgs::SmartcarRunType::Request &req, automotive_msgs::SmartcarRunType::Response &res);

        bool onServiceSetSmartcarType(automotive_msgs::SmartcarRunType::Request &req, automotive_msgs::SmartcarRunType::Response &res);

    public:
        CanAdapterNode();

        ~CanAdapterNode();

        // callbacks

        // initializer
    };

} // namespace can_adapter

#endif /* CAN_ADAPTER_H */
