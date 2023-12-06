
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <sstream>
#include <string>
#include <ros/ros.h>


#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

class ndt_localizer
{

public:
  ndt_localizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  ~ndt_localizer();

private:
  ros::NodeHandle nh, private_nh;

  ros::Subscriber initial_pose_sub;
  ros::Subscriber map_points_sub;
  ros::Subscriber sensor_points_sub;

  ros::Publisher sensor_aligned_pose_pub;
  ros::Publisher ndt_pose_pub;
  ros::Publisher exe_time_pub;
  ros::Publisher transform_probability_pub;
  ros::Publisher iteration_num_pub;
  ros::Publisher diagnostics_pub;
  ros::Publisher odom_pub;

  struct pose {
      double x;
      double y;
      double z;
      double roll;
      double pitch;
      double yaw;
  };

  enum class MethodType {
      PCL_GENERIC = 0,
      PCL_ANH = 1,
      PCL_ANH_GPU = 2,
      PCL_OPENMP = 3,
  };

  MethodType method_type = MethodType::PCL_GENERIC;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;
  tf2_ros::TransformBroadcaster tf2_broadcaster;

  Eigen::Matrix4f base_to_sensor_matrix;
  Eigen::Matrix4f pre_trans, delta_trans;
  bool init_pose = false;

  std::string base_frame;
  std::string map_frame;

  // init guess for ndt
  geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg;

  std::mutex ndt_map_mtx;

  double converged_param_transform_probability;
  std::thread diagnostic_thread;
  std::map<std::string, std::string> key_value_stdmap;

  // function
  void init_params();
  void timer_diagnostic();

  bool get_transform(const std::string & target_frame, const std::string & source_frame,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr,
                       const ros::Time & time_stamp);
  bool get_transform(const std::string & target_frame, 
                       const std::string & source_frame,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr);
  void publish_tf(const std::string & frame_id, const std::string & child_frame_id,
                    const geometry_msgs::PoseStamped & pose_msg);

  void callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
  void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr);
  void callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);

};