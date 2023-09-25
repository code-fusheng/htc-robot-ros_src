#include <ros/ros.h>
#include <std_msgs/Time.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hello_robot");
    ros::NodeHandle nh;
    ros::Publisher hello_pub = nh.advertise<std_msgs::Time>("hello_pub", 10);
    printf("hello world! hello robot\n");
    while (ros::ok())
    {
        // printf("Current time: %f\n", ros::Time::now().toSec());
        std_msgs::Time msg;
        msg.data = ros::Time::now();
        hello_pub.publish(msg);
        ros::spinOnce();
    }
    return 0;
}
