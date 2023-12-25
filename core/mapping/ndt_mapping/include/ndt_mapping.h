#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>

#include <time.h>
#include <thread>

#include <automotive_msgs/SaveMap.h>

#include <htcbot_msgs/ModeSwitch.h>
#include <htcbot_msgs/MapPathConf.h>

class ndt_mapping
{

public:
  ndt_mapping(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  ~ndt_mapping();

  void run();

private:
  ros::NodeHandle nh, private_nh;
  
  ros::Subscriber points_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber output_sub;

  ros::SubscribeOptions map_ops;


  // 
  enum class MethodType
  {
    PCL_GENERIC = 0,
    PCL_ANH = 1,
    PCL_ANH_GPU = 2,
    PCL_OPENMP = 3,
  };

  MethodType method_type = MethodType::PCL_GENERIC;

  struct pose
  {
    double x;     // x
    double y;     // y
    double z;     // z
    double roll;  // | x
    double pitch; // | y
    double yaw;   // | z
  };

  struct pose current_pose, current_pose_imu, current_pose_odom, current_pose_imu_odom;
  struct pose previous_pose;
  struct pose ndt_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom;
  struct pose added_pose, localizer_pose;

  ros::Time current_scan_time;
  ros::Time previous_scan_time;
  ros::Duration scan_duration;

  bool use_imu;
  bool use_odom;

  // Topic
  std::string _odom_topic;
  std::string _lidar_topic;
  std::string _imu_topic;

  double diff = 0.0;
  double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose - previous_pose
  
  // offset
  double offset_imu_x = 0.0;
  double offset_imu_y = 0.0;
  double offset_imu_z = 0.0;
  double offset_imu_roll = 0.0;
  double offset_imu_pitch = 0.0;
  double offset_imu_yaw = 0.0;

  double offset_odom_x = 0.0;
  double offset_odom_y = 0.0;
  double offset_odom_z = 0.0;
  double offset_odom_roll = 0.0;
  double offset_odom_pitch = 0.0;
  double offset_odom_yaw = 0.0;

  double offset_imu_odom_x = 0.0;
  double offset_imu_odom_y = 0.0;
  double offset_imu_odom_z = 0.0;
  double offset_imu_odom_roll = 0.0;
  double offset_imu_odom_pitch = 0.0;
  double offset_imu_odom_yaw = 0.0;
  
  // current_velocity

  double current_velocity_x = 0.0;
  double current_velocity_y = 0.0;
  double current_velocity_z = 0.0;

  double current_velocity_imu_x = 0.0;
  double current_velocity_imu_y = 0.0;
  double current_velocity_imu_z = 0.0;

  pcl::PointCloud<pcl::PointXYZI> map;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt;

  // Default values
  int max_iter;        // Maximum iterations
  float ndt_res;      // Resolution
  double step_size;   // Step size
  double trans_eps;  // Transformation epsilon

  // Leaf size of VoxelGrid filter.
  double voxel_leaf_size;

  double scan_rate;
  double min_scan_range;
  double max_scan_range;
  double min_add_scan_shift;
  int initial_scan_loaded;

  double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
  Eigen::Matrix4f tf_btol, tf_ltob;

  std::string map_path;

  ros::Publisher ndt_map_pub, current_pose_pub;
  ros::Publisher history_trajectory_pub;

  ros::Subscriber switch_sub;
  ros::Subscriber map_path_conf_sub;

  geometry_msgs::PoseStamped current_pose_msg;

  sensor_msgs::Imu imu;
  nav_msgs::Odometry odom;
  nav_msgs::Path history_trajectory;

  tf::TransformBroadcaster br;

  bool incremental_voxel_update;

  double fitness_score;
  bool has_converged;
  int final_num_iteration;
  double transformation_probability;

  bool is_first_map;

  std::ofstream ofs;
  std::string filename;

  double PI = 3.141592654;

  bool mode_switch;

  void init_param();
  void param_callback();
  void swtich_callback(const htcbot_msgs::ModeSwitch::ConstPtr& input);
  void path_conf_callback(const htcbot_msgs::MapPathConf::ConstPtr& input);
  void output_callback(const automotive_msgs::SaveMap::ConstPtr& input);

  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void imu_callback(const sensor_msgs::Imu::Ptr& input);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& input);

  void imu_calc(ros::Time current_time);
  void odom_calc(ros::Time current_time);
  void imu_odom_calc(ros::Time current_time);

  double wrapToPm(double a_num, const double a_max);
  double wrapToPmPi(double a_angle_rad);
  double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
  double getAbsoluteAngleDiff(const double &current_yaw, const double &added_yaw);

};