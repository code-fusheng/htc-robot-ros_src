#include <ros/ros.h>
#include <local_trajectory_generator/local_trajectory_generator.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "local_trajectory_generator_node");
    LocalTrajectoryGeneratorNS::LocalTrajectoryGenerator app;
    app.run(); 
    return 0;
}

