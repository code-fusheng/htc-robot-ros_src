#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void LidarCallback(const sensor_msgs::LaserScan msg)
{
    // ROS_INFO("测距 ranges[0] = %f 米", msg.ranges[0]);
    // ROS_INFO("测距 ranges[90] = %f 米", msg.ranges[90]);
    ROS_INFO("测距 ranges[180] = %f 米", msg.ranges[180]);
    // ROS_INFO("测距 ranges[270] = %f 米", msg.ranges[270]);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &LidarCallback);
    ros::spin();
    return 0;
}