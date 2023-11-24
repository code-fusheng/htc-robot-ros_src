#include "htc_wheeltec_robot.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define PI 3.1415926f // PI //圆周率

int main(int argc, char **argv)
{
  ros::init(argc, argv, "htc_yunle_robot"); // ROS initializes and sets the node name // ROS初始化 并设置节点名称

  // 创建节点句柄
  ros::NodeHandle nh;

  // 创建tf广播器对象
  tf2_ros::TransformBroadcaster tf_broadcaster;

  ros::Rate rate(10.0); // 设置发布频率为10Hz

  while (nh.ok())
  {
    // 创建TransformStamped消息
    geometry_msgs::TransformStamped transform_stamped;

    // 填充消息字段
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_footprint";

    // 设置变换矩阵
    transform_stamped.transform.translation.x = 1.0; // 设置平移量
    transform_stamped.transform.rotation.w = 1.0;    // 设置旋转四元数

    // 发布TF变换
    tf_broadcaster.sendTransform(transform_stamped);

    // 等待
    rate.sleep();
  }

  return 0;
}