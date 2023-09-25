/*
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2023-09-25 15:33:58
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2023-09-25 15:35:56
 * @FilePath: /src/simulation/imu_pkg/src/imu_node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"

// IMU 回调函数
void IMUCallback(const sensor_msgs::Imu msg)
{
    // 检测消息包中四元数数据是否存在
    if(msg.orientation_covariance[0] < 0)
        return;
    // 四元数转成欧拉角
    tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // 弧度换算成角度
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = yaw*180/M_PI;
    ROS_INFO("滚转= %.0f 俯仰= %.0f 朝向= %.0f", roll, pitch, yaw);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc,argv, "demo_imu_data"); 

    ros::NodeHandle n;
    // 订阅 IMU 的数据话题
    ros::Subscriber sub = n.subscribe("imu/data", 100, IMUCallback);
    ros::spin();

    return 0;
}
