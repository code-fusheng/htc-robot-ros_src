#include "can_odometry_core.h"
#include <can_msgs/ecu.h>

namespace can_odometry
{
// Constructor
CanOdometryNode::CanOdometryNode() : private_nh_("~"), v_info_(), odom_(ros::Time::now())
{
  initForROS();
}

// Destructor
CanOdometryNode::~CanOdometryNode()
{
}

void CanOdometryNode::initForROS()
{
  // ros parameter settings
  // if (!nh_.hasParam("/vehicle_info/wheel_base"))
  // {
  //   v_info_.is_stored = false;
  //   ROS_INFO("vehicle_info is not set");
  // }
  // else
  // {
  //   private_nh_.getParam("/vehicle_info/wheel_base", v_info_.wheel_base);
  //   // ROS_INFO_STREAM("wheel_base : " << wheel_base);

  //   v_info_.is_stored = true;
  // }
  private_nh_.getParam("wheel_base",         v_info_.wheel_base);
  private_nh_.getParam("minimum_turning_radius",           v_info_.minimum_turning_radius);
  private_nh_.getParam("maximum_steering_wheel_angle_deg", v_info_.maximum_steering_wheel_angle_deg);
  private_nh_.getParam("is_akm",                           v_info_.is_akm);
  v_info_.is_stored = true;
  ROS_INFO("wheel_base=%.2f, turn_radius=%.2f, max_angle=%.2f, is_akm=%d", v_info_.wheel_base, v_info_.minimum_turning_radius, v_info_.maximum_steering_wheel_angle_deg, v_info_.is_akm);

  // setup subscriber
  sub1_ = nh_.subscribe("/vehicle_status", 10, &CanOdometryNode::callbackFromVehicleStatus, this);

  // setup publisher
  pub1_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
}

void CanOdometryNode::run()
{
  ros::spin();
}

void CanOdometryNode::publishOdometry(const can_msgs::vehicle_statusConstPtr& msg)
{
  // double vx = kmph2mps(msg->speed);
  double vx = msg->cur_speed;
  // if (msg->shift_level == can_msgs::ecu::SHIFT_R)
  //   vx = -vx;
  // else if (msg->shift_level == can_msgs::ecu::SHIFT_N)
  //   vx = 0;

  double angle_rad = -deg2rad(msg->cur_steer);
  double vth;

  if (v_info_.is_akm) {
    // double angle_rad = msg->wheel_direction == 0 ? deg2rad(-1*msg->cur_steer) : deg2rad(msg->cur_steer);
    if (msg->shift_level == can_msgs::ecu::SHIFT_D)
      vx = vx;
    else if (msg->shift_level == can_msgs::ecu::SHIFT_R)
      vx = -vx;
    else
      vx = 0;
      
    vth = v_info_.convertSteeringAngleToAngularVelocity(vx, angle_rad);
    odom_.updateOdometry(vx, vth, msg->Header.stamp);

    // ROS_INFO("akm vth=%.2f", vth);
  }else{
    vth = (msg->right_wheel_speed - msg->left_wheel_speed) / v_info_.wheel_base;
    odom_.updateOdometry((msg->left_wheel_speed+msg->right_wheel_speed)/2.0, vth, msg->Header.stamp);
    // ROS_INFO("no akm vth=%.2f", vth);
  }


  // ROS_INFO("vx: %.2f, angle_rad: %.2f, vth: %.2f", vx, angle_rad, vth);


  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_.th);

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = msg->Header.stamp;
  odom.header.frame_id = "odom_combined";

  // set the position
  odom.pose.pose.position.x = odom_.x;
  odom.pose.pose.position.y = odom_.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_footprint";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = vth;

  //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
  if(msg->cur_speed <= 0.05 && fabs(msg->cur_steer) < 0.05)
    //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
    //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
    memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
  else
    //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
    //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
    memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));  

  // publish the message
  pub1_.publish(odom);

  // create and send the TF transform
  // static tf::TransformBroadcaster br;
  // tf::Transform transform;
  // transform.setOrigin(tf::Vector3(odom_.x, odom_.y, 0.0));
  // tf::Quaternion q;
  // q.setRPY(0, 0, odom_.th);
  // transform.setRotation(q);
  // br.sendTransform(tf::StampedTransform(transform, msg->Header.stamp, "odom", "base_footprint"));

}

void CanOdometryNode::callbackFromVehicleStatus(const can_msgs::vehicle_statusConstPtr& msg)
{
  publishOdometry(msg);
}

}  // autoware_connector
