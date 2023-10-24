#include <ros/ros.h>

#include "can_odometry_core.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "can_odometry");
  can_odometry::CanOdometryNode n;
  n.run();

  return 0;
}
