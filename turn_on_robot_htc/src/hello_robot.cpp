/*
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2023-09-21 16:49:13
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2023-09-27 14:31:52
 * @FilePath: /src/turn_on_robot_htc/src/hello_robot.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>
#include <std_msgs/Time.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hello_robot");
    ros::NodeHandle nh;
    
    ros::Publisher hello_pub = nh.advertise<std_msgs::Time>("hello_pub", 10);
    printf("hello world! hello robot\n");
    // printf("Current time: %f\n", ros::Time::now().toSec());
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    hello_pub.publish(msg);
    ros::spinOnce();
    return 0;
}
