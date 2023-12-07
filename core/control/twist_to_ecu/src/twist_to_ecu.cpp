#include "twist_to_ecu.h"

twist_to_ecu::twist_to_ecu(
    ros::NodeHandle &nh, ros::NodeHandle &private_nh
) {}

twist_to_ecu::~twist_to_ecu() {};

void twist_to_ecu::run()
{

    twist_sub = nh.subscribe("/twist_raw", 1, &twist_to_ecu::twist_callback, this);
    ecu_pub = nh.advertise<can_msgs::ecu>("ecu", 1);
    ros::spin();

}

void twist_to_ecu::twist_callback(const geometry_msgs::TwistStamped::Ptr& input) 
{
    can_msgs::ecu ecu_msg;

    ecu_msg.motor = input -> twist.linear.x;
    ecu_msg.steer += (input -> twist.angular.z * 180 / PI) * 1;
    if (ecu_msg.steer > 120)
        ecu_msg.steer = 30;
    if (ecu_msg.steer < -120)
        ecu_msg.steer = -30;
    
    ecu_pub.publish(ecu_msg);
}