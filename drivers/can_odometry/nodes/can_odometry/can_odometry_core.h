#ifndef CAN_ODOMETRY_CORE_H
#define CAN_ODOMETRY_CORE_H

// ROS includes
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

// User Defined Includes
#include "can_msgs/vehicle_status.h"
#include "can_vehicle_info.h"

namespace can_odometry
{

//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;


struct Odometry
{
  double x;
  double y;
  double th;
  ros::Time stamp;

  Odometry(const ros::Time& time)
  {
    x = 0.0;
    y = 0.0;
    th = 0.0;
    stamp = time;
  }

  // 阿克曼车型
  void updateOdometry(const double vx, const double vth, const ros::Time& cur_time)
  {
    if (stamp.sec == 0 && stamp.nsec == 0)
    {
      stamp = cur_time;
    }
    double dt = (cur_time - stamp).toSec();
    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;
    double delta_th = vth * dt;

    // ROS_INFO("dt : %f delta (x y th) : (%f %f %f %f)", dt, delta_x, delta_y, delta_th);

    x += delta_x;
    y += delta_y;
    th += delta_th;
    stamp = cur_time;
  }
};

class CanOdometryNode
{
public:
  CanOdometryNode();
  ~CanOdometryNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_;

  // subscriber
  ros::Subscriber sub1_;

  // variables
  VehicleInfo v_info_;
  Odometry odom_;

  // callbacks
  void callbackFromVehicleStatus(const can_msgs::vehicle_statusConstPtr& msg);

  // initializer
  void initForROS();

  // functions
  void publishOdometry(const can_msgs::vehicle_statusConstPtr& msg);
};
}
#endif  // CAN_ODOMETRY_CORE_H
