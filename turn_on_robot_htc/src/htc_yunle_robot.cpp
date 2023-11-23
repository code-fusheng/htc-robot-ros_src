#include "htc_wheeltec_robot.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "htc_yunle_robot"); // ROS initializes and sets the node name // ROS初始化 并设置节点名称 
  ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  return 0;  
} 