#include <ros/ros.h>
#include "laserscan_combine_detect/laserscan_combine_detect.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "laserscan_combine_detect_node");
    LaserScanCombineDetector::LaserCombineDetectorApp app;
    app.run(); 
    return 0;
}
