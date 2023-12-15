#include "twist_to_ecu.h"

twist_to_ecu::twist_to_ecu(
    ros::NodeHandle &nh, ros::NodeHandle &private_nh
) {}

twist_to_ecu::~twist_to_ecu() {};

void twist_to_ecu::run()
{
    
    // private_nh.getParam("source_topic", source_topic, "twist_raw");
    // ROS_INFO("source_topic: %s", source_topic.c_str());

    // private_nh.getParam("target_topic", target_topic, "ecu");
    // ROS_INFO("target_topic: %s", target_topic.c_str());

    twist_sub = nh.subscribe("twist_raw", 1, &twist_to_ecu::twist_callback, this);
    ecu_pub = nh.advertise<can_msgs::ecu>("ecu", 1);
    ros::spin();

}

void twist_to_ecu::twist_callback(const geometry_msgs::TwistStamped::Ptr& input) 
{
    can_msgs::ecu ecu_msg;


    if(input -> twist.linear.x>0.0001)
    {
        if(input -> twist.linear.x>2)
        {
            ecu_msg.shift = 1;
            ecu_msg.motor = 2;
        }
        else
        {
            ecu_msg.shift = 1;
            ecu_msg.motor = input -> twist.linear.x;
        }
        
    }
    else if(input -> twist.linear.x<-0.0001)
    {
        if(input -> twist.linear.x<-2)
        {
            ecu_msg.shift = 3;
            ecu_msg.motor = -2*(-1);
        }
        else
        {
            ecu_msg.shift = 3;
            ecu_msg.motor = input -> twist.linear.x*(-1);
        }
    }
    else
    {
        ecu_msg.shift = 2;
        ecu_msg.motor =0;
    }

    ecu_msg.steer += (input -> twist.angular.z * 180 / PI) * (-1);
    if (ecu_msg.steer > 120)
        ecu_msg.steer = 30;
    if (ecu_msg.steer < -120)
        ecu_msg.steer = -30;


    
    ecu_pub.publish(ecu_msg);
}