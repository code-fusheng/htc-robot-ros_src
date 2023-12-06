#include <can_msgs/ecu.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char *argv[]){
    ros::init(argc,argv,"can_puber");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd",5);

    geometry_msgs::TwistStamped tMsg;
    tMsg.twist.linear.x = 0.5;
    tMsg.twist.angular.z = 0.1;

    geometry_msgs::TwistStamped tMsg2;
    tMsg2.twist.linear.x = 0.0;
    tMsg2.twist.angular.z = 0.0;

    	ROS_INFO("Init Success.");
    ros::Duration du(1);
	ROS_INFO("Init Success.");
    du.sleep();
	ROS_INFO("Init Success.");
    pub.publish(tMsg);
    ROS_INFO("Published tMsg.");
    du.sleep();
    pub.publish(tMsg2);
    ROS_INFO("Publisher tMsg2.");
    du.sleep();
    return 0;

}
