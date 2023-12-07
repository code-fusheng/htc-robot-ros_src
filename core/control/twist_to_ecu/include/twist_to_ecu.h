
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <can_msgs/ecu.h>

class twist_to_ecu
{

public:

    twist_to_ecu(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~twist_to_ecu();

    void run();

private:

    ros::NodeHandle nh, private_nh;

    ros::Publisher ecu_pub;
    ros::Subscriber twist_sub;

    double PI = 3.141592654;

    void twist_callback(const geometry_msgs::TwistStamped::Ptr& input);

};