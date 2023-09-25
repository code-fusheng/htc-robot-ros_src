#include "ros/ros.h"

class turn_on_robot_htc
{
    public:
        turn_on_robot_htc();
        ~turn_on_robot_htc();
    private:
        ros::Time _Now, _Last_time;
};