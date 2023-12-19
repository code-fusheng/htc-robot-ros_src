#include "ndt_mapping.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "htc_ndt_mapping");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ndt_mapping node(nh, private_nh);
    node.run();
    return 0;
}