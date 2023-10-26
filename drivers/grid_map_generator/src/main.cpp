#include <ros/ros.h>
#include "pcd_grid_divider.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grid_map_generator");
    MAP_TOOLS::pcd_grid_divider app;
    app.run();
    // signal(SIGINT, USB2CAN::ExitHandler);
    return 0;
}