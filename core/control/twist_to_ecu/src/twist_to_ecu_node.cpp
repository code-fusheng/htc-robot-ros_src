#include "twist_to_ecu.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_to_ecu");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    twist_to_ecu node(nh, private_nh);
    node.run();

    return 0;
}