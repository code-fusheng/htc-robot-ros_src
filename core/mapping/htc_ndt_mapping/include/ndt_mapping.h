#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

#include <time.h>

class ndt_mapping
{

public:
  ndt_mapping(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  ~ndt_mapping();

private:
  ros::NodeHandle nh, private_nh;
  
  ros::Subscriber points_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber imu_sub;

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

  struct pose current_pose, current_pose_imu, current_pose_odom;
  struct pose previous_pose;
  struct pose ndt_pose, guess_pose, guess_pose_imu;
  struct pose added_pose, localizer_pose;

  ros::Time current_scan_time;
  ros::Time previous_scan_time;
  ros::Duration scan_duration;

  double diff = 0.0;
  double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose - previous_pose
  double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;

  double current_velocity_x = 0.0;
  double current_velocity_y = 0.0;
  double current_velocity_z = 0.0;

  double current_velocity_imu_x = 0.0;
  double current_velocity_imu_y = 0.0;
  double current_velocity_imu_z = 0.0;

  pcl::PointCloud<pcl::PointXYZI> map;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

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

  bool use_imu;
  bool use_odom;

  std::string imu_topic;

  ros::Publisher ndt_map_pub, current_pose_pub;
  ros::Publisher history_trajectory_pub;
  geometry_msgs::PoseStamped current_pose_msg;

  sensor_msgs::Imu imu;
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

  void param_callback();
  void output_callback();

  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void imu_callback(const sensor_msgs::Imu::Ptr& input);

  void imu_calc(ros::Time current_time);

  double wrapToPm(double a_num, const double a_max);
  double wrapToPmPi(double a_angle_rad);
  double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
  double getAbsoluteAngleDiff(const double &current_yaw, const double &added_yaw);

};