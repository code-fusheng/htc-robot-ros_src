#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
// #include <velodyne_pointcloud/point_types.h>
// #include <velodyne_pointcloud/rawdata.h>
#include <automotive_msgs/SaveMap.h>
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif
#ifdef USE_PCL_OPENMP
#include <pcl_omp_registration/ndt.h>
#endif

#include <nav_msgs/Path.h>
#include <time.h>
#include "automotive_msgs/OnlineMapping.h"
#include <automotive_msgs/NDTMappingReq.h>
#include <automotive_msgs/NDTMappingRes.h>
#include <std_msgs/String.h>
#include <thread>
#include <mutex>

static const double PI = 3.141592654;

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
static MethodType _method_type = MethodType::PCL_GENERIC;

// global variables
static pose previous_pose, guess_pose, guess_pose_imu, guess_pose_odom,
    guess_pose_imu_odom, current_pose, current_pose_imu, current_pose_odom,
    current_pose_imu_odom, ndt_pose, added_pose, localizer_pose;

static ros::Time current_s_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0,
              diff_yaw;  // current_pose - previous_pose
static double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll,
    offset_imu_pitch, offset_imu_yaw;
static double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll,
    offset_odom_pitch, offset_odom_yaw;
static double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z,
    offset_imu_odom_roll, offset_imu_odom_pitch, offset_imu_odom_yaw;

static double current_velocity_x = 0.0;
static double current_velocity_y = 0.0;
static double current_velocity_z = 0.0;

static double current_velocity_imu_x = 0.0;
static double current_velocity_imu_y = 0.0;
static double current_velocity_imu_z = 0.0;

static pcl::PointCloud<pcl::PointXYZI> map;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
static cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> anh_ndt;
#ifdef CUDA_FOUND
static gpu::GNormalDistributionsTransform anh_gpu_ndt;
#endif
#ifdef USE_PCL_OPENMP
static pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> omp_ndt;
#endif

// Default values
static int max_iter = 30;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.1;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start,
    t2_end, t3_start, t3_end, t4_start, t4_end, t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher ndt_map_pub;
static ros::Publisher current_pose_pub;
static ros::Publisher guess_pose_linaer_pub;
static ros::Publisher history_trajectory_pub;
static ros::Publisher pub_ndtmapping_res;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double min_scan_range = 5.0;
static double max_scan_range = 200.0;
static double min_add_scan_shift = 1.0;

static Eigen::Matrix4f tf_btol, tf_ltob;

static bool _use_imu = false;
static bool _use_odom = false;
static bool _imu_upside_down = false;

static bool _incremental_voxel_update = false;

static std::string _imu_topic = "/imu_raw";
std::string lidar_topic, vehicle_odom_topic, vehicle_twist_topic;

static double fitness_score;
static bool has_converged;
static int final_num_iteration;
static double transformation_probability;

static sensor_msgs::Imu imu;
static nav_msgs::Odometry odom;

static std::ofstream ofs;
static std::string filename;
static double ndt_set_target_time = 0.0;
static double ndt_align_time = 0.0;
static double total_time = 0.0;
nav_msgs::Path history_trajectory;
static bool is_map_saving_in_progress = false;
static bool is_filter_before_add_to_map = false;
static float voxel_size_filter_before_add_to_map = 0.3;
static bool is_on_mapping = false;
ros::Subscriber points_sub, odom_sub , twist_sub , imu_sub;
ros::Subscriber output_sub, sub_mapping_req, sub_force_stop;

// 对于控制消息，使用单独的call back queue
ros::CallbackQueue control_queue;

static void thread_pub_global_map() {
    ros::Time start = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(
        new pcl::PointCloud<pcl::PointXYZI>(map));
    map_ptr->header.frame_id = "map";
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    ndt_map_pub.publish(*map_msg_ptr);
    double total_time = (ros::Time::now() - start).toSec();
}

static inline double get_absolute_angle_diff(const double &current_yaw, const double &added_yaw) {
    double diff = (current_yaw - added_yaw) * 180.0 / PI;
    diff = diff + 360*4;
    int res = int(diff) % 360;
    if ( res > 180 ) res = 360 -res;
    return res;
}

/*
 *  通过callback触发地图保存操作
 *
 */

static void _do_save_map(const std::string filename, const double &filter_res) {
    ros::Time start = ros::Time::now();
    ROS_INFO(">> start to save map");
    std::cout << "filter_res: " << filter_res << std::endl;
    std::cout << "filename: " << filename << std::endl;

    if (map.size() < 10) {
        ROS_WARN("[ndt_mapping] No enough points in map, abort save map.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    map_ptr->header.frame_id = "map";
    map_filtered->header.frame_id = "map";
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);

    // Apply voxelgrid filter
    if (filter_res < 0.01) {  // 此时不进行降采样，保存原始点云到地图
        std::cout << "Original: " << map_ptr->points.size() << " points."
                  << std::endl;
        pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    } else {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
        voxel_grid_filter.setInputCloud(map_ptr);
        voxel_grid_filter.filter(*map_filtered);
        std::cout << "Original: " << map_ptr->points.size() << " points."
                  << std::endl;
        std::cout << "Filtered: " << map_filtered->points.size() << " points."
                  << std::endl;
        pcl::toROSMsg(*map_filtered, *map_msg_ptr);
    }

    ndt_map_pub.publish(*map_msg_ptr);

    // Writing Point Cloud data to PCD file
    std::cout << "Start saving... " << std::endl;
    try {
        if (filter_res < 0.01) {
            pcl::io::savePCDFileASCII(filename, *map_ptr);
            ros::Time end = ros::Time::now();
            std::cout << "Saved " << map_ptr->points.size() << " data points to "
                    << filename << ". -> total used time: " << end - start << "s"
                    << std::endl;
        } else {
            pcl::io::savePCDFileASCII(filename, *map_filtered);
            ros::Time end = ros::Time::now();
            std::cout << "Saved " << map_filtered->points.size()
                    << " data points to " << filename << "." << std::endl;
        }
    } catch (const std::exception& e) {
        ROS_WARN("Save to %s failed!, %s", filename.c_str(), e.what());
    }
}

static void output_callback(const automotive_msgs::SaveMap::ConstPtr &input) {
    double filter_res = input->file_res;
    std::string filename = input->filename;
    _do_save_map(filename, filter_res);
}

static void imu_odom_calc(ros::Time current_time) {
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    double diff_imu_roll = imu.angular_velocity.x * diff_time;
    double diff_imu_pitch = imu.angular_velocity.y * diff_time;
    double diff_imu_yaw = imu.angular_velocity.z * diff_time;

    current_pose_imu_odom.roll += diff_imu_roll;
    current_pose_imu_odom.pitch += diff_imu_pitch;
    current_pose_imu_odom.yaw += diff_imu_yaw;

    double diff_distance = odom.twist.twist.linear.x * diff_time;
    offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) *
                         cos(current_pose_imu_odom.yaw);
    offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) *
                         sin(current_pose_imu_odom.yaw);
    offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

    offset_imu_odom_roll += diff_imu_roll;
    offset_imu_odom_pitch += diff_imu_pitch;
    offset_imu_odom_yaw += diff_imu_yaw;

    guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
    guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
    guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
    guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
    guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
    guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

    previous_time = current_time;
}

static void odom_calc(ros::Time current_time) {
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
    double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
    double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

    current_pose_odom.roll += diff_odom_roll;
    current_pose_odom.pitch += diff_odom_pitch;
    current_pose_odom.yaw += diff_odom_yaw;

    double diff_distance = odom.twist.twist.linear.x * diff_time;
    offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) *
                     cos(current_pose_odom.yaw);
    offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) *
                     sin(current_pose_odom.yaw);
    offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

    offset_odom_roll += diff_odom_roll;
    offset_odom_pitch += diff_odom_pitch;
    offset_odom_yaw += diff_odom_yaw;

    guess_pose_odom.x = previous_pose.x + offset_odom_x;
    guess_pose_odom.y = previous_pose.y + offset_odom_y;
    guess_pose_odom.z = previous_pose.z + offset_odom_z;
    guess_pose_odom.roll = previous_pose.roll + offset_odom_roll;
    guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
    guess_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

    previous_time = current_time;
}

static void imu_calc(ros::Time current_time) {
    static ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();

    double diff_imu_roll = imu.angular_velocity.x * diff_time;
    double diff_imu_pitch = imu.angular_velocity.y * diff_time;
    double diff_imu_yaw = imu.angular_velocity.z * diff_time;

    current_pose_imu.roll += diff_imu_roll;
    current_pose_imu.pitch += diff_imu_pitch;
    current_pose_imu.yaw += diff_imu_yaw;

    double accX1 = imu.linear_acceleration.x;
    double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
                   std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
    double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
                   std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

    double accX2 = std::sin(current_pose_imu.pitch) * accZ1 +
                   std::cos(current_pose_imu.pitch) * accX1;
    double accY2 = accY1;
    double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 -
                   std::sin(current_pose_imu.pitch) * accX1;

    double accX = std::cos(current_pose_imu.yaw) * accX2 -
                  std::sin(current_pose_imu.yaw) * accY2;
    double accY = std::sin(current_pose_imu.yaw) * accX2 +
                  std::cos(current_pose_imu.yaw) * accY2;
    double accZ = accZ2;

    offset_imu_x +=
        current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
    offset_imu_y +=
        current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
    offset_imu_z +=
        current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

    current_velocity_imu_x += accX * diff_time;
    current_velocity_imu_y += accY * diff_time;
    current_velocity_imu_z += accZ * diff_time;

    offset_imu_roll += diff_imu_roll;
    offset_imu_pitch += diff_imu_pitch;
    offset_imu_yaw += diff_imu_yaw;

    guess_pose_imu.x = previous_pose.x + offset_imu_x;
    guess_pose_imu.y = previous_pose.y + offset_imu_y;
    guess_pose_imu.z = previous_pose.z + offset_imu_z;
    guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
    guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
    guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

    previous_time = current_time;
}

static double wrapToPm(double a_num, const double a_max) {
    if (a_num >= a_max) {
        a_num -= 2.0 * a_max;
    }
    return a_num;
}

static double wrapToPmPi(double a_angle_rad) {
    return wrapToPm(a_angle_rad, M_PI);
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad) {
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
        diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
        diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
}
static void odom_callback(const nav_msgs::Odometry::ConstPtr &input) {
    odom = *input;
    odom_calc(input->header.stamp);
}

static void vehicle_twist_callback(
    const geometry_msgs::TwistStampedConstPtr &msg) {
    odom.header = msg->header;
    odom.twist.twist = msg->twist;
    odom_calc(odom.header.stamp);
}

static void imuUpsideDown(const sensor_msgs::Imu::Ptr input) {
    double input_roll, input_pitch, input_yaw;

    tf::Quaternion input_orientation;
    tf::quaternionMsgToTF(input->orientation, input_orientation);
    tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

    input->angular_velocity.x *= -1;
    input->angular_velocity.y *= -1;
    input->angular_velocity.z *= -1;

    input->linear_acceleration.x *= -1;
    input->linear_acceleration.y *= -1;
    input->linear_acceleration.z *= -1;

    input_roll *= -1;
    input_pitch *= -1;
    input_yaw *= -1;

    input->orientation = tf::createQuaternionMsgFromRollPitchYaw(
        input_roll, input_pitch, input_yaw);
}

static void imu_callback(const sensor_msgs::Imu::Ptr &input) {
    // std::cout << __func__ << std::endl;

    if (_imu_upside_down) imuUpsideDown(input);

    const ros::Time current_time = input->header.stamp;
    static ros::Time previous_time = current_time;
    const double diff_time = (current_time - previous_time).toSec();

    double imu_roll, imu_pitch, imu_yaw;
    tf::Quaternion imu_orientation;
    tf::quaternionMsgToTF(input->orientation, imu_orientation);
    tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

    imu_roll = wrapToPmPi(imu_roll);
    imu_pitch = wrapToPmPi(imu_pitch);
    imu_yaw = wrapToPmPi(imu_yaw);

    static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch,
                  previous_imu_yaw = imu_yaw;
    const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
    const double diff_imu_pitch =
        calcDiffForRadian(imu_pitch, previous_imu_pitch);
    const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

    imu.header = input->header;
    imu.linear_acceleration.x = input->linear_acceleration.x;
    // imu.linear_acceleration.y = input->linear_acceleration.y;
    // imu.linear_acceleration.z = input->linear_acceleration.z;
    imu.linear_acceleration.y = 0;
    imu.linear_acceleration.z = 0;

    if (diff_time != 0) {
        imu.angular_velocity.x = diff_imu_roll / diff_time;
        imu.angular_velocity.y = diff_imu_pitch / diff_time;
        imu.angular_velocity.z = diff_imu_yaw / diff_time;
    } else {
        imu.angular_velocity.x = 0;
        imu.angular_velocity.y = 0;
        imu.angular_velocity.z = 0;
    }

    imu_calc(input->header.stamp);

    previous_time = current_time;
    previous_imu_roll = imu_roll;
    previous_imu_pitch = imu_pitch;
    previous_imu_yaw = imu_yaw;
}

static pcl::PointCloud<pcl::PointXYZI> global_transformed_scan;
static bool flag_update_map = false;
std::mutex lock;

static void update_runtime_map() {
    ros::Rate rate = ros::Rate(10);
    while ( ros::isStarted() ) {
        if ( is_on_mapping && flag_update_map) {
            static ros::Time update_st = ros::Time::now();
            // fileter transformed_scan_ptr before added to global map
            // on general desktop, keeping whole pointcloud will decrease handling
            // speed when map going big much time will be cost on update global map
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_filtered(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr input_source(new pcl::PointCloud<pcl::PointXYZI>(global_transformed_scan));
            if (is_filter_before_add_to_map) {
                pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
                voxel_grid_filter.setLeafSize(voxel_size_filter_before_add_to_map,
                                            voxel_size_filter_before_add_to_map,
                                            voxel_size_filter_before_add_to_map);
                voxel_grid_filter.setInputCloud(input_source);
                voxel_grid_filter.filter(*tmp_filtered);
                map += *tmp_filtered;
                // std::cout << "filter before: " << transformed_scan_ptr->size()
                //         << " after: " << tmp_filtered->size() << std::endl;
                ROS_INFO("[ndt mapping] filter before: %d, filter after: %d", global_transformed_scan.size(), tmp_filtered->size());
            } else {
                map += global_transformed_scan;
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

            added_pose.x = current_pose.x;
            added_pose.y = current_pose.y;
            added_pose.z = current_pose.z;
            added_pose.roll = current_pose.roll;
            added_pose.pitch = current_pose.pitch;
            added_pose.yaw = current_pose.yaw;
            
            // 1. init new model
            pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> new_ndt;
            cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> new_anh_ndt;
            #ifdef CUDA_FOUND
            gpu::GNormalDistributionsTransform new_anh_gpu_ndt;
            #endif
            #ifdef USE_PCL_OPENMP
            pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> new_omp_ndt;
            #endif
            if (_method_type == MethodType::PCL_GENERIC)
                new_ndt.setInputTarget(map_ptr);
            else if (_method_type == MethodType::PCL_ANH) {
                if (_incremental_voxel_update == true)
                    new_anh_ndt.updateVoxelGrid(input_source);
                else
                    new_anh_ndt.setInputTarget(map_ptr);
            }
#ifdef CUDA_FOUND
            else if (_method_type == MethodType::PCL_ANH_GPU)
                new_anh_gpu_ndt.setInputTarget(map_ptr);
#endif
#ifdef USE_PCL_OPENMP
            else if (_method_type == MethodType::PCL_OPENMP)
                new_omp_ndt.setInputTarget(map_ptr);
#endif

            // replace
            lock.lock();
            if (_method_type == MethodType::PCL_GENERIC)
                ndt = new_ndt;
            else if (_method_type == MethodType::PCL_ANH) {
                anh_ndt = new_anh_ndt;
            }
#ifdef CUDA_FOUND
            else if (_method_type == MethodType::PCL_ANH_GPU)
                anh_gpu_ndt = new_anh_gpu_ndt;
#endif
#ifdef USE_PCL_OPENMP
            else if (_method_type == MethodType::PCL_OPENMP)
                omp_ndt = new_omp_ndt;
#endif
            lock.unlock();

            // 发布更新后的地图
            sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(map, *map_msg_ptr);
            ndt_map_pub.publish(*map_msg_ptr);
            flag_update_map = false;
            static ros::Time update_end = ros::Time::now();
            ROS_INFO("[ndt_mapping] update map used %.2f seconds", (update_end - update_st).toSec());
        }

        rate.sleep();
    }
}

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {
    // if (!is_on_mapping) return;
    static bool _is_running = false;
    // if ( !_is_running ) {
    //     _is_running = true;
    // }else{
    //     ROS_WARN("[ndt_mapping] Previous matching is on, one point cloud lost!");
    //     return;
    // }
    _is_running = true;

    ros::Time start_handle_time = ros::Time::now();
    double r;
    pcl::PointXYZI p;
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    tf::Quaternion q;

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Time current_scan_time = input->header.stamp;

    pcl::fromROSMsg(*input, tmp);

    // 过滤车身点和最远点
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin();
         item != tmp.end(); item++) {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = (double)item->intensity;

        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        // if (r < 100){
        //     if (!(-1.2 < p.x < 1.0 && -0.6 < p.y < 0.6)){
        //         scan.push_back(p);
        //     }
        // }
        if (min_scan_range < r && r < max_scan_range) {
            scan.push_back(p);
        }
    }
    // scan保存的是去除最近最远点后的原始点云

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

    // Add initial point cloud to velodyne_map
    static bool is_first_map = true;
    if (initial_scan_loaded == 0) {
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
        map += *transformed_scan_ptr;
        initial_scan_loaded = 1;
        is_first_map = true;
    }

    // Apply voxelgrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size,
                                  voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

    // ROS_INFO("[ndt mapping] points cb lock");
    lock.lock();
    if (_method_type == MethodType::PCL_GENERIC) {
        ndt.setTransformationEpsilon(trans_eps);
        ndt.setStepSize(step_size);
        ndt.setResolution(ndt_res);
        ndt.setMaximumIterations(max_iter);
        ndt.setInputSource(filtered_scan_ptr);
    } else if (_method_type == MethodType::PCL_ANH) {
        anh_ndt.setTransformationEpsilon(trans_eps);
        anh_ndt.setStepSize(step_size);
        anh_ndt.setResolution(ndt_res);
        anh_ndt.setMaximumIterations(max_iter);
        anh_ndt.setInputSource(filtered_scan_ptr);
    }
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU) {
        anh_gpu_ndt.setTransformationEpsilon(trans_eps);
        anh_gpu_ndt.setStepSize(step_size);
        anh_gpu_ndt.setResolution(ndt_res);
        anh_gpu_ndt.setMaximumIterations(max_iter);
        anh_gpu_ndt.setInputSource(filtered_scan_ptr);
    }
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP) {
        omp_ndt.setTransformationEpsilon(trans_eps);
        omp_ndt.setStepSize(step_size);
        omp_ndt.setResolution(ndt_res);
        omp_ndt.setMaximumIterations(max_iter);
        omp_ndt.setInputSource(filtered_scan_ptr);
    }
#endif

    ros::Time start_set_ndt_target = ros::Time::now();
    if (is_first_map == true) {
        if (_method_type == MethodType::PCL_GENERIC)
            ndt.setInputTarget(map_ptr);
        else if (_method_type == MethodType::PCL_ANH)
            anh_ndt.setInputTarget(map_ptr);
#ifdef CUDA_FOUND
        else if (_method_type == MethodType::PCL_ANH_GPU)
            anh_gpu_ndt.setInputTarget(map_ptr);
#endif
#ifdef USE_PCL_OPENMP
        else if (_method_type == MethodType::PCL_OPENMP)
            omp_ndt.setInputTarget(map_ptr);
#endif
        is_first_map = false;
        _is_running = false;
        lock.unlock();
        return;
    }
    ndt_set_target_time = (ros::Time::now() - start_set_ndt_target).toSec();

    guess_pose.x = previous_pose.x + diff_x;
    guess_pose.y = previous_pose.y + diff_y;
    guess_pose.z = previous_pose.z + diff_z;
    guess_pose.roll = previous_pose.roll;
    guess_pose.pitch = previous_pose.pitch;
    guess_pose.yaw = previous_pose.yaw + diff_yaw;

    // 根据imu和odom的开启情况预先计算init_guess
    if (_use_imu == true && _use_odom == true) imu_odom_calc(current_scan_time);
    if (_use_imu == true && _use_odom == false) imu_calc(current_scan_time);
    if (_use_imu == false && _use_odom == true) odom_calc(current_scan_time);

    pose guess_pose_for_ndt;
    if (_use_imu == true && _use_odom == true)
        guess_pose_for_ndt = guess_pose_imu_odom;
    else if (_use_imu == true && _use_odom == false)
        guess_pose_for_ndt = guess_pose_imu;
    else if (_use_imu == false && _use_odom == true)
        guess_pose_for_ndt = guess_pose_odom;
    else
        guess_pose_for_ndt = guess_pose;

    Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll,
                                      Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch,
                                      Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw,
                                      Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(
        guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

    Eigen::Matrix4f init_guess =
        (init_translation * init_rotation_z * init_rotation_y * init_rotation_x)
            .matrix() *
        tf_btol;

    t3_end = ros::Time::now();
    d3 = t3_end - t3_start;

    t4_start = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    ros::Time start_ndt_align = ros::Time::now();
    if (_method_type == MethodType::PCL_GENERIC) {
        ndt.align(*output_cloud, init_guess);
        fitness_score = ndt.getFitnessScore();
        t_localizer = ndt.getFinalTransformation();
        has_converged = ndt.hasConverged();
        final_num_iteration = ndt.getFinalNumIteration();
        transformation_probability = ndt.getTransformationProbability();
    } else if (_method_type == MethodType::PCL_ANH) {
        anh_ndt.align(init_guess);
        fitness_score = anh_ndt.getFitnessScore();
        t_localizer = anh_ndt.getFinalTransformation();
        has_converged = anh_ndt.hasConverged();
        final_num_iteration = anh_ndt.getFinalNumIteration();
    }
#ifdef CUDA_FOUND
    else if (_method_type == MethodType::PCL_ANH_GPU) {
        anh_gpu_ndt.align(init_guess);
        fitness_score = anh_gpu_ndt.getFitnessScore();
        t_localizer = anh_gpu_ndt.getFinalTransformation();
        has_converged = anh_gpu_ndt.hasConverged();
        final_num_iteration = anh_gpu_ndt.getFinalNumIteration();
    }
#endif
#ifdef USE_PCL_OPENMP
    else if (_method_type == MethodType::PCL_OPENMP) {
        omp_ndt.align(*output_cloud, init_guess);
        fitness_score = omp_ndt.getFitnessScore();
        t_localizer = omp_ndt.getFinalTransformation();
        has_converged = omp_ndt.hasConverged();
        final_num_iteration = omp_ndt.getFinalNumIteration();
    }
#endif
    lock.unlock();
    // ROS_INFO("[ndt mapping] points cb unlock");
    ndt_align_time = (ros::Time::now() - start_ndt_align).toSec();

    t_base_link = t_localizer * tf_ltob;
    // std::cout << t_localizer << std::endl;
    // std::cout << "----------------------" << std::endl;
    // std::cout << t_base_link << std::endl;

    // 注意scan_ptr保存的几乎是原始点云(只过滤了最近最远点)，因此用于匹配的地图会很大
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

    tf::Matrix3x3 mat_l, mat_b;

    mat_l.setValue(static_cast<double>(t_localizer(0, 0)),
                   static_cast<double>(t_localizer(0, 1)),
                   static_cast<double>(t_localizer(0, 2)),
                   static_cast<double>(t_localizer(1, 0)),
                   static_cast<double>(t_localizer(1, 1)),
                   static_cast<double>(t_localizer(1, 2)),
                   static_cast<double>(t_localizer(2, 0)),
                   static_cast<double>(t_localizer(2, 1)),
                   static_cast<double>(t_localizer(2, 2)));

    mat_b.setValue(static_cast<double>(t_base_link(0, 0)),
                   static_cast<double>(t_base_link(0, 1)),
                   static_cast<double>(t_base_link(0, 2)),
                   static_cast<double>(t_base_link(1, 0)),
                   static_cast<double>(t_base_link(1, 1)),
                   static_cast<double>(t_base_link(1, 2)),
                   static_cast<double>(t_base_link(2, 0)),
                   static_cast<double>(t_base_link(2, 1)),
                   static_cast<double>(t_base_link(2, 2)));

    // Update localizer_pose.
    localizer_pose.x = t_localizer(0, 3);
    localizer_pose.y = t_localizer(1, 3);
    localizer_pose.z = t_localizer(2, 3);
    mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw,
                 1);

    // Update ndt_pose.
    ndt_pose.x = t_base_link(0, 3);
    ndt_pose.y = t_base_link(1, 3);
    ndt_pose.z = t_base_link(2, 3);
    mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

    current_pose.x = ndt_pose.x;
    current_pose.y = ndt_pose.y;
    current_pose.z = ndt_pose.z;
    current_pose.roll = ndt_pose.roll;
    current_pose.pitch = ndt_pose.pitch;
    current_pose.yaw = ndt_pose.yaw;

    transform.setOrigin(
        tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    transform.setRotation(q);

    br.sendTransform(
        tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

    scan_duration = current_scan_time - previous_scan_time;
    double secs = scan_duration.toSec();

    // Calculate the offset (curren_pos - previous_pos)
    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    current_velocity_x = diff_x / secs;
    current_velocity_y = diff_y / secs;
    current_velocity_z = diff_z / secs;

    current_pose_imu.x = current_pose.x;
    current_pose_imu.y = current_pose.y;
    current_pose_imu.z = current_pose.z;
    current_pose_imu.roll = current_pose.roll;
    current_pose_imu.pitch = current_pose.pitch;
    current_pose_imu.yaw = current_pose.yaw;

    current_pose_odom.x = current_pose.x;
    current_pose_odom.y = current_pose.y;
    current_pose_odom.z = current_pose.z;
    current_pose_odom.roll = current_pose.roll;
    current_pose_odom.pitch = current_pose.pitch;
    current_pose_odom.yaw = current_pose.yaw;

    current_pose_imu_odom.x = current_pose.x;
    current_pose_imu_odom.y = current_pose.y;
    current_pose_imu_odom.z = current_pose.z;
    current_pose_imu_odom.roll = current_pose.roll;
    current_pose_imu_odom.pitch = current_pose.pitch;
    current_pose_imu_odom.yaw = current_pose.yaw;

    current_velocity_imu_x = current_velocity_x;
    current_velocity_imu_y = current_velocity_y;
    current_velocity_imu_z = current_velocity_z;

    // Update position and posture. current_pos -> previous_pos
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    previous_scan_time.sec = current_scan_time.sec;
    previous_scan_time.nsec = current_scan_time.nsec;

    offset_imu_x = 0.0;
    offset_imu_y = 0.0;
    offset_imu_z = 0.0;
    offset_imu_roll = 0.0;
    offset_imu_pitch = 0.0;
    offset_imu_yaw = 0.0;

    offset_odom_x = 0.0;
    offset_odom_y = 0.0;
    offset_odom_z = 0.0;
    offset_odom_roll = 0.0;
    offset_odom_pitch = 0.0;
    offset_odom_yaw = 0.0;

    offset_imu_odom_x = 0.0;
    offset_imu_odom_y = 0.0;
    offset_imu_odom_z = 0.0;
    offset_imu_odom_roll = 0.0;
    offset_imu_odom_pitch = 0.0;
    offset_imu_odom_yaw = 0.0;

    // Calculate the shift between added_pos and current_pos
    double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) +
                        pow(current_pose.y - added_pose.y, 2.0));
    double _diff_angle = get_absolute_angle_diff(current_pose.yaw, added_pose.yaw) ;  // 度, 当前朝向和上次添加点的朝向的绝对差值
    // ROS_INFO("_diff_angle = %.2lf", _diff_angle);
    if (shift >= min_add_scan_shift || _diff_angle > 7)  // 更新地图并发布
    {
        global_transformed_scan = *transformed_scan_ptr;
        flag_update_map = true;
//         static ros::Time update_st = ros::Time::now();
//         // fileter transformed_scan_ptr before added to global map
//         // on general desktop, keeping whole pointcloud will decrease handling
//         // speed when map going big much time will be cost on update global map
//         pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_filtered(
//             new pcl::PointCloud<pcl::PointXYZI>());
//         if (is_filter_before_add_to_map) {
//             pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
//             voxel_grid_filter.setLeafSize(voxel_size_filter_before_add_to_map,
//                                           voxel_size_filter_before_add_to_map,
//                                           voxel_size_filter_before_add_to_map);
//             voxel_grid_filter.setInputCloud(transformed_scan_ptr);
//             voxel_grid_filter.filter(*tmp_filtered);
//             map += *tmp_filtered;
//             std::cout << "filter before: " << transformed_scan_ptr->size()
//                       << " after: " << tmp_filtered->size() << std::endl;
//         } else {
//             map += *transformed_scan_ptr;
//         }

//         added_pose.x = current_pose.x;
//         added_pose.y = current_pose.y;
//         added_pose.z = current_pose.z;
//         added_pose.roll = current_pose.roll;
//         added_pose.pitch = current_pose.pitch;
//         added_pose.yaw = current_pose.yaw;

//         if (_method_type == MethodType::PCL_GENERIC)
//             ndt.setInputTarget(map_ptr);
//         else if (_method_type == MethodType::PCL_ANH) {
//             if (_incremental_voxel_update == true)
//                 anh_ndt.updateVoxelGrid(transformed_scan_ptr);
//             else
//                 anh_ndt.setInputTarget(map_ptr);
//         }
// #ifdef CUDA_FOUND
//         else if (_method_type == MethodType::PCL_ANH_GPU)
//             anh_gpu_ndt.setInputTarget(map_ptr);
// #endif
// #ifdef USE_PCL_OPENMP
//         else if (_method_type == MethodType::PCL_OPENMP)
//             omp_ndt.setInputTarget(map_ptr);
// #endif
//         // 发布更新后的地图
//         sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
//         pcl::toROSMsg(*map_ptr, *map_msg_ptr);
//         ndt_map_pub.publish(*map_msg_ptr);

//         static ros::Time update_end = ros::Time::now();
//         ROS_INFO("[ndt_mapping] update map used %.2f seconds", (update_end - update_st).toSec());
    }

    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = current_scan_time;
    current_pose_msg.pose.position.x = current_pose.x;
    current_pose_msg.pose.position.y = current_pose.y;
    current_pose_msg.pose.position.z = current_pose.z;
    current_pose_msg.pose.orientation.x = q.x();
    current_pose_msg.pose.orientation.y = q.y();
    current_pose_msg.pose.orientation.z = q.z();
    current_pose_msg.pose.orientation.w = q.w();

    current_pose_pub.publish(current_pose_msg);
    total_time = (ros::Time::now() - start_handle_time).toSec();

    if (shift > 0.1) {
        history_trajectory.poses.push_back(current_pose_msg);
        history_trajectory_pub.publish(history_trajectory);
    }
    _is_running = false;

    // Write log
    // if (!ofs) {
    //     std::cerr << "Could not open " << filename << "." << std::endl;
    //     exit(1);
    // }
    // ofs << input->header.seq << "," << input->header.stamp << ","
    //     << input->header.frame_id << "," << scan_ptr->size() << ","
    //     << filtered_scan_ptr->size() << "," << std::fixed
    //     << std::setprecision(5) << current_pose.x << "," << std::fixed
    //     << std::setprecision(5) << current_pose.y << "," << std::fixed
    //     << std::setprecision(5) << current_pose.z << "," << current_pose.roll
    //     << "," << current_pose.pitch << "," << current_pose.yaw << ","
    //     << final_num_iteration << "," << fitness_score << "," << ndt_res << ","
    //     << step_size << "," << trans_eps << "," << max_iter << ","
    //     << voxel_leaf_size << "," << min_scan_range << "," << max_scan_range
    //     << "," << min_add_scan_shift << "," << ndt_set_target_time << ","
    //     << ndt_align_time << "," << total_time << std::endl;

    std::cout
        << "-----------------------------------------------------------------"
        << std::endl;
    std::cout << "Sequence number: " << input->header.seq << std::endl;
    std::cout << "Number of scan points: " << scan_ptr->size() << " points."
              << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size()
              << " points." << std::endl;
    std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size()
              << " points." << std::endl;
    std::cout << "map: " << map.points.size() << " points." << std::endl;
    std::cout << "NDT has converged: " << has_converged << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "Number of iteration: " << final_num_iteration << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", "
              << current_pose.z << ", " << current_pose.roll << ", "
              << current_pose.pitch << ", " << current_pose.yaw << ")"
              << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t_localizer << std::endl;
    std::cout << "shift: " << shift << std::endl;
    std::cout << "ndt_set_target_time: " << ndt_set_target_time << std::endl;
    std::cout << "ndt_align_time: " << ndt_align_time << std::endl;
    std::cout << "total_time: " << total_time << std::endl;
    std::cout
        << "-----------------------------------------------------------------"
        << std::endl;
}

static void reset() {
    previous_pose.x = 0.0;
    previous_pose.y = 0.0;
    previous_pose.z = 0.0;
    previous_pose.roll = 0.0;
    previous_pose.pitch = 0.0;
    previous_pose.yaw = 0.0;

    ndt_pose.x = 0.0;
    ndt_pose.y = 0.0;
    ndt_pose.z = 0.0;
    ndt_pose.roll = 0.0;
    ndt_pose.pitch = 0.0;
    ndt_pose.yaw = 0.0;

    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.z = 0.0;
    current_pose.roll = 0.0;
    current_pose.pitch = 0.0;
    current_pose.yaw = 0.0;

    current_pose_imu.x = 0.0;
    current_pose_imu.y = 0.0;
    current_pose_imu.z = 0.0;
    current_pose_imu.roll = 0.0;
    current_pose_imu.pitch = 0.0;
    current_pose_imu.yaw = 0.0;

    guess_pose.x = 0.0;
    guess_pose.y = 0.0;
    guess_pose.z = 0.0;
    guess_pose.roll = 0.0;
    guess_pose.pitch = 0.0;
    guess_pose.yaw = 0.0;

    added_pose.x = 0.0;
    added_pose.y = 0.0;
    added_pose.z = 0.0;
    added_pose.roll = 0.0;
    added_pose.pitch = 0.0;
    added_pose.yaw = 0.0;

    diff_x = 0.0;
    diff_y = 0.0;
    diff_z = 0.0;
    diff_yaw = 0.0;

    offset_imu_x = 0.0;
    offset_imu_y = 0.0;
    offset_imu_z = 0.0;
    offset_imu_roll = 0.0;
    offset_imu_pitch = 0.0;
    offset_imu_yaw = 0.0;

    offset_odom_x = 0.0;
    offset_odom_y = 0.0;
    offset_odom_z = 0.0;
    offset_odom_roll = 0.0;
    offset_odom_pitch = 0.0;
    offset_odom_yaw = 0.0;

    offset_imu_odom_x = 0.0;
    offset_imu_odom_y = 0.0;
    offset_imu_odom_z = 0.0;
    offset_imu_odom_roll = 0.0;
    offset_imu_odom_pitch = 0.0;
    offset_imu_odom_yaw = 0.0;

    diff = 0.0;

    current_velocity_x = 0.0;
    current_velocity_y = 0.0;
    current_velocity_z = 0.0;

    current_velocity_imu_x = 0.0;
    current_velocity_imu_y = 0.0;
    current_velocity_imu_z = 0.0;

    map.clear();
    initial_scan_loaded = 0;
}

static void start_mapping() {
    ros::NodeHandle nh;
    points_sub = nh.subscribe(lidar_topic, 10, points_callback);
    if (_use_odom) {
        odom_sub = nh.subscribe(vehicle_odom_topic, 10, odom_callback);
        twist_sub = nh.subscribe(vehicle_twist_topic, 10, vehicle_twist_callback);
    }
    if (_use_imu) {
        imu_sub = nh.subscribe(_imu_topic, 10, imu_callback);
    }
}

static void stop_mapping() {
    points_sub.shutdown();
    if (_use_odom) {
        odom_sub.shutdown();
        twist_sub.shutdown();
    }
    if (_use_imu) {
        imu_sub.shutdown();
    }

    reset();
}

static bool onServiceOnlineMapping(automotive_msgs::OnlineMapping::Request &req, automotive_msgs::OnlineMapping::Response &res) {
    ROS_INFO("[ndt_mapping] service received: start_mapping?: %d, save_dir: %s", req.start_mapping, req.save_dir.c_str());
    
    if (req.start_mapping && !is_on_mapping){
        is_on_mapping = true;
        start_mapping();
        ROS_INFO("[ndt_mapping] Started");
    }
    
    if (!req.start_mapping && is_on_mapping){
        is_on_mapping = false;
        // automotive_msgs::SaveMap msg;
        // msg.file_res = 0.1;
        // msg.filename = req.save_dir + "/static.pcd";
        // ROS_INFO("[ndt_mapping] Saving map to %s", msg.filename.c_str());
        // output_callback(msg);
        _do_save_map(req.save_dir + "/static.pcd", 0);

        stop_mapping();
        ROS_INFO("[ndt_mapping] Stopped");
    }
    
    res.confirmed = true;
    return true;
}

static void cb_mappint_req(const automotive_msgs::NDTMappingReqConstPtr& req) {
    ROS_INFO("[ndt_mapping] topic received: start_mapping?: %d, save_dir: %s", req->start_mapping, req->save_dir.c_str());
    
    if ( req->start_mapping == req->START_MAPPING ){
        if ( is_on_mapping ) {
            automotive_msgs::NDTMappingRes msg_res;
            msg_res.is_mapping = msg_res.MAPPING_ON;
            msg_res.info = "Mapping process is running, use StopAll or Stop to stop it.";
            pub_ndtmapping_res.publish(msg_res);
            return;
        }
        start_mapping();
        is_on_mapping = true;
        automotive_msgs::NDTMappingRes msg_res;
        msg_res.is_mapping = msg_res.MAPPING_ON;
        pub_ndtmapping_res.publish(msg_res);
    }
    
    if ( req->start_mapping == req->STOP_MAPPING ){
        if ( !is_on_mapping ) {
            automotive_msgs::NDTMappingRes msg_res;
            msg_res.is_mapping = msg_res.MAPPING_OFF;
            msg_res.info = "Mapping process is not running.";
            pub_ndtmapping_res.publish(msg_res);
            return;
        }

        is_on_mapping = false;

        // automotive_msgs::SaveMap msg;
        // msg.file_res = 0.02;
        // msg.filename = req->save_dir + "/static.pcd";
        // ROS_INFO("[ndt_mapping] Saving map to %s", msg.filename.c_str());
        // output_callback(msg);
        _do_save_map(req->save_dir + "/static.pcd", 0);

        stop_mapping();

        automotive_msgs::NDTMappingRes msg_res;
        msg_res.is_mapping = msg_res.MAPPING_OFF;
        pub_ndtmapping_res.publish(msg_res); 
    }
}

static void cd_force_stop_mapping(const std_msgs::StringConstPtr& input) {
    ROS_WARN("[ndt_mapping] Received force stop mapping command, will shutdown ndt mapping without saving.");
    is_on_mapping = false;
    stop_mapping();
    automotive_msgs::NDTMappingRes msg_res;
    msg_res.is_mapping = msg_res.MAPPING_OFF;
    msg_res.info = "Mapping stopped.";
    pub_ndtmapping_res.publish(msg_res); 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ndt_mapping");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set log file name.
    char buffer[80];
    std::time_t now = std::time(NULL);
    std::tm *pnow = std::localtime(&now);
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
    filename = "ndt_mapping_" + std::string(buffer) + ".csv";
    ofs.open(filename.c_str(), std::ios::app);

    // write header for log file
    if (!ofs) {
        std::cerr << "Could not open " << filename << "." << std::endl;
        exit(1);
    }

    ofs << "input->header.seq"
        << ","
        << "input->header.stamp"
        << ","
        << "input->header.frame_id"
        << ","
        << "scan_ptr->size()"
        << ","
        << "filtered_scan_ptr->size()"
        << ","
        << "current_pose.x"
        << ","
        << "current_pose.y"
        << ","
        << "current_pose.z"
        << ","
        << "current_pose.roll"
        << ","
        << "current_pose.pitch"
        << ","
        << "current_pose.yaw"
        << ","
        << "final_num_iteration"
        << ","
        << "fitness_score"
        << ","
        << "ndt_res"
        << ","
        << "step_size"
        << ","
        << "trans_eps"
        << ","
        << "max_iter"
        << ","
        << "voxel_leaf_size"
        << ","
        << "min_scan_range"
        << ","
        << "max_scan_range"
        << ","
        << "min_add_scan_shift"
        << ","
        << "ndt_set_target_time"
        << ","
        << "ndt_align_time"
        << ","
        << "total_time"
        << "," << std::endl;

    // setting parameters
    int method_type_tmp = 0;
    private_nh.getParam("method_type", method_type_tmp);
    _method_type = static_cast<MethodType>(method_type_tmp);
    private_nh.getParam("use_odom", _use_odom);
    private_nh.getParam("use_imu", _use_imu);
    private_nh.getParam("imu_upside_down", _imu_upside_down);
    private_nh.getParam("imu_topic", _imu_topic);
    private_nh.getParam("incremental_voxel_update", _incremental_voxel_update);

    std::cout << "method_type: " << static_cast<int>(_method_type) << std::endl;
    std::cout << "use_odom: " << _use_odom << std::endl;
    std::cout << "use_imu: " << _use_imu << std::endl;
    std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
    std::cout << "imu_topic: " << _imu_topic << std::endl;
    std::cout << "incremental_voxel_update: " << _incremental_voxel_update
              << std::endl;

    nh.param("lidar_topic", lidar_topic, std::string("lidar_topic_unset!"));
    nh.param("vehicle_odom_topic", vehicle_odom_topic, std::string("vehicle_odom_topic_unset!"));
    nh.param("vehicle_twist_topic", vehicle_twist_topic, std::string("vehicle_twist_topic_unset!"));

    // _init_ndt_config();
    private_nh.getParam("ndt_res", ndt_res);
    private_nh.getParam("step_size", step_size);
    private_nh.getParam("trans_eps", trans_eps);
    private_nh.getParam("max_iter", max_iter);
    private_nh.getParam("voxel_leaf_size", voxel_leaf_size);
    private_nh.getParam("min_scan_range", min_scan_range);
    private_nh.getParam("max_scan_range", max_scan_range);
    private_nh.getParam("min_add_scan_shift", min_add_scan_shift);
    private_nh.getParam("is_filter_before_add_to_map", is_filter_before_add_to_map);
    private_nh.getParam("voxel_size_filter_before_add_to_map", voxel_size_filter_before_add_to_map);

    std::cout << "-*-*-*- NDT config -*-*-*-" << std::endl;
    std::cout << "ndt_res: " << ndt_res << std::endl;
    std::cout << "step_size: " << step_size << std::endl;
    std::cout << "trans_epsilon: " << trans_eps << std::endl;
    std::cout << "max_iter: " << max_iter << std::endl;
    std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
    std::cout << "min_scan_range: " << min_scan_range << std::endl;
    std::cout << "max_scan_range: " << max_scan_range << std::endl;
    std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
    std::cout << "is_filter_before_add_to_map: " << is_filter_before_add_to_map
              << std::endl;
    std::cout << "voxel_size_filter_before_add_to_map: "
              << voxel_size_filter_before_add_to_map << std::endl;

    std::string lidar_frame;
    nh.param("lidar_frame", lidar_frame,
                     std::string("lidar_frame_unset!"));
    tf::TransformListener tf_listener;
    tf::StampedTransform tf_baselink2primarylidar;
    bool received_tf = false;
    // 1. Try getting base_link -> lidar TF from TF tree
    while (!received_tf) {
        try {
            tf_listener.waitForTransform("base_link", lidar_frame, ros::Time(),
                                         ros::Duration(1.0));
            tf_listener.lookupTransform("base_link", lidar_frame, ros::Time(),
                                        tf_baselink2primarylidar);
        } catch (tf::TransformException &ex) {
            ROS_WARN(
                "Query base_link to primary lidar frame through TF tree "
                "failed: %s",
                ex.what());
            received_tf = false;
            sleep(0.5);
            continue;
        }
        received_tf = true;
    }

    // // 只使用一个雷达，因此必须全局定义base_link到lidar frame的tf变换关系
    // // 2. Try getting base_link -> lidar TF from tf_baselink2primarylidar
    // param if (!received_tf)
    // {
    //   std::vector<double> bl2pl_vec;
    //   if (nh.getParam("tf_baselink2primarylidar", bl2pl_vec) &&
    //   bl2pl_vec.size() == 6)
    //   {
    //     tf::Vector3 trans(bl2pl_vec[0], bl2pl_vec[1], bl2pl_vec[2]);
    //     tf::Quaternion quat;
    //     quat.setRPY(bl2pl_vec[5], bl2pl_vec[4], bl2pl_vec[3]);
    //     tf_baselink2primarylidar.setOrigin(trans);
    //     tf_baselink2primarylidar.setRotation(quat);

    //     received_tf = true;
    //   }
    //   else
    //   {
    //     ROS_WARN("Query base_link to primary lidar frame through
    //     tf_baselink2primarylidar param failed");
    //   }
    // }

    // // 3. Try getting base_link -> lidar TF from tf_* params
    // if (!received_tf)
    // {
    //   float tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
    //   if (nh.getParam("tf_x", tf_x) &&
    //       nh.getParam("tf_y", tf_y) &&
    //       nh.getParam("tf_z", tf_z) &&
    //       nh.getParam("tf_roll", tf_roll) &&
    //       nh.getParam("tf_pitch", tf_pitch) &&
    //       nh.getParam("tf_yaw", tf_yaw))
    //   {
    //     tf::Vector3 trans(tf_x, tf_y, tf_z);
    //     tf::Quaternion quat;
    //     quat.setRPY(tf_roll, tf_pitch, tf_yaw);
    //     tf_baselink2primarylidar.setOrigin(trans);
    //     tf_baselink2primarylidar.setRotation(quat);

    //     received_tf = true;
    //   }
    //   else
    //   {
    //     ROS_WARN("Query base_link to primary lidar frame through tf_* params
    //     failed");
    //   }
    // }

    if (received_tf) {
        ROS_INFO("base_link to primary lidar transform queried successfully");
    } else {
        ROS_ERROR("Failed to query base_link to primary lidar transform");
        return 1;
    }

    // 将获得的tf_baselink2primarylidar转换到tf_btol和tf_ltob
    Eigen::Translation3f tl_btol(tf_baselink2primarylidar.getOrigin().getX(),
                                 tf_baselink2primarylidar.getOrigin().getY(),
                                 tf_baselink2primarylidar.getOrigin().getZ());
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_baselink2primarylidar.getRotation())
        .getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
    tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();      //雷达到机器人的静态变换矩阵
    tf_ltob = tf_btol.inverse();  //机器人到雷达的静态变换矩

#ifndef CUDA_FOUND
    if (_method_type == MethodType::PCL_ANH_GPU) {
        std::cerr
            << "**************************************************************"
            << std::endl;
        std::cerr
            << "[ERROR]PCL_ANH_GPU is not built. Please use other method type."
            << std::endl;
        std::cerr
            << "**************************************************************"
            << std::endl;
        exit(1);
    }
#endif
#ifndef USE_PCL_OPENMP
    if (_method_type == MethodType::PCL_OPENMP) {
        std::cerr
            << "**************************************************************"
            << std::endl;
        std::cerr
            << "[ERROR]PCL_OPENMP is not built. Please use other method type."
            << std::endl;
        std::cerr
            << "**************************************************************"
            << std::endl;
        exit(1);
    }
#endif

    map.header.frame_id = "map";
    history_trajectory.header.frame_id = "map";

    ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 10);
    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    history_trajectory_pub = nh.advertise<nav_msgs::Path>("/ndt/history_trajectory", 10);
    pub_ndtmapping_res = nh.advertise<automotive_msgs::NDTMappingRes>("/ndtmapping/res", 5);

    ros::SubscribeOptions save_map_ops = ros::SubscribeOptions::create<automotive_msgs::SaveMap>("mapping/save_map", 10, output_callback, ros::VoidPtr(), &control_queue);
    output_sub = nh.subscribe(save_map_ops);

    ros::SubscribeOptions mapping_control_ops = ros::SubscribeOptions::create<automotive_msgs::NDTMappingReq>("/ndtmapping/control", // topic name
                                                                                        10, // queue length
                                                                                        cb_mappint_req, // callback
                                                                                        ros::VoidPtr(), // tracked object, we don't need one thus NULL
                                                                                        &control_queue // pointer to callback queue object
                                                                                        );
    sub_mapping_req = nh.subscribe(mapping_control_ops);

    ros::SubscribeOptions force_stop_ops = ros::SubscribeOptions::create<std_msgs::String>("/ndtmapping/force_stop", // topic name
                                                                                        10, // queue length
                                                                                        cd_force_stop_mapping, // callback
                                                                                        ros::VoidPtr(), // tracked object, we don't need one thus NULL
                                                                                        &control_queue // pointer to callback queue object
                                                                                        );
    sub_force_stop = nh.subscribe(force_stop_ops);

    ros::AsyncSpinner control_spiner(1, &control_queue);
    control_spiner.start();

    // output_sub = nh.subscribe("mapping/save_map", 10, output_callback);
    // sub_mapping_req = nh.subscribe("/ndtmapping/control", 5, cb_mappint_req);
    // sub_force_stop = nh.subscribe("/ndtmapping/force_stop", 3, cd_force_stop_mapping);

    reset();

    // ros::ServiceServer mapping_service = nh.advertiseService("online_mapping", onServiceOnlineMapping);
    is_on_mapping = true;

    points_sub = nh.subscribe(lidar_topic, 10, points_callback);

    // // 使用多线程spin, 防止points cloud队列过满，其他队列无法执行的问题
    // ros::MultiThreadedSpinner spinner(6); // 数字表示调用的线程数  
    // spinner.spin(); // spin() will not return until the node has been shutdown
    std::thread thread_update_map(update_runtime_map);

    ros::spin();
    thread_update_map.join();

    return 0;
}
