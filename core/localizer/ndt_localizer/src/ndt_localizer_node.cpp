#include "ndt_localizer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "htc_ndt_localizer");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ndt_localizer node(nh, private_nh);
    node.run();
    return 0;
}