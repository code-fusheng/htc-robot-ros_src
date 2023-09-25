#ifndef __HTC_ROBOT_H_
#define __HTC_ROBOT_H_

#include "ros/ros.h"
#include "iostream"

# 


class turn_on_robot_htc
{
    public:
        turn_on_robot_htc();
        ~turn_on_robot_htc();
    private:
        ros::Time _Now, _Last_time;
};

#endif