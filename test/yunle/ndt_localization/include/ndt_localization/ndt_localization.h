#ifndef __NDT_LOCALZATION__
#define __NDT_LOCALZATION__

#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// #include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pthread.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>

#include <boost/thread/thread.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif

#include <pcl_omp_registration/ndt.h>
#ifndef USE_OMP
#define USE_OMP
#endif

#include <automotive_msgs/NDTStat.h>
#include <can_msgs/vehicle_status.h>

#include <cmath>

#include "ground_filter.hpp"
#include "user_protocol.h"
#include "utils.hpp"

namespace NDTLocalization {

#define METHOD_PCL 0
#define METHOD_CUDA 1
#define METHOD_OMP 2
#define METHOD_CPU 3

#define pi 3.141592654

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// ros::NodeHandle nh_, pnh_;  // tf也是个nodehandle

// tf::TransformBroadcaster tf_broadcaster_;  // 因为tf跟具体的ros节点相关,因此不能写在全局变量里
// tf::TransformListener tf_listener_;

pose current_pose_;
pose pre_pose_;
pose current_pose_odom_;
pose pre_pose_odom_;

pose initial_pose_;  // under map frame
ros::Subscriber sub_initial_pose_;
void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

bool pose_init_;
bool odom_init_;
bool map_init_;

pose offset_odom_;
ros::Time pre_odom_time_;
pose offset_imu_;
Eigen::Matrix4f tf_btol_;
tf::Transform current_map2odom_;

double param_min_scan_range;
double param_max_scan_range;

std::string param_odom_frame_;
std::string param_map_frame_;
std::string param_base_frame_;
std::string param_laser_frame_;

std::string param_map_topic_;
std::string param_lidar_topic_;
std::string param_odom_topic_;

double param_tf_timeout_;
double param_odom_timeout_;
bool param_use_odom_;
double param_predict_error_thresh_;

// ndt相关
double param_ndt_resolution_;
int param_ndt_max_iterations_;
double param_ndt_step_size_;
double param_ndt_epsilon_;
int param_method_type_;

#ifdef CUDA_FOUND
std::shared_ptr<gpu::GNormalDistributionsTransform> anh_gpu_ndt_ptr;
#endif
#ifdef USE_OMP
pcl_omp::NormalDistributionsTransform<PointT, PointT> omp_ndt_;
#endif
cpu::NormalDistributionsTransform<PointT, PointT> cpu_ndt_;
pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
bool has_converged_;
double fitness_score_;
double trans_probability_;
int iteration_;
double predict_pose_error_;
ros::Publisher pub_ndt_state_;
ros::Publisher history_trajectory_pub;
nav_msgs::Path history_trajectory;

pthread_mutex_t mutex;
pthread_t thread;

ros::Publisher pub_current_pose_;
geometry_msgs::PoseStamped msg_current_pose_;

ros::Publisher pub_global_map;
// globalmap message??

ros::Subscriber sub_odomimu_;
pose predict_pose_odom_;
void OdomImuCB(const geometry_msgs::PoseStamped::ConstPtr &msg);

ros::Subscriber sub_point_cloud_;
PointCloudT model_pc_;
int model_pc_num_;

PointCloudT data_pc_;  // 定义要加载的全局地图

ros::Subscriber sub_vehicle_status;
double cur_speed;
double pre_time;
void feedbackCB(const can_msgs::vehicle_status::ConstPtr &msg);

// debug use
bool param_debug_;
std::string map_file;
bool is_filter_ground;
double voxel_leaf_size;

ros::Subscriber sub_map_;
void mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

bool load_map(std::string map_file);

void pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

void init_pose_with_param();
double init_x, init_y, init_z;
double init_roll, init_pitch, init_yaw;
bool param_init_pose_with_param;

void *thread_func(void *args);

bool init(ros::NodeHandle nh, ros::NodeHandle pnh);

}  // namespace NDTLocalization
#endif
