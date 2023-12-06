#include "pcd_map_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "htc_pcd_map_manager");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    pcd_map_manager node(nh, private_nh);
    node.run();
    return 0;
}
