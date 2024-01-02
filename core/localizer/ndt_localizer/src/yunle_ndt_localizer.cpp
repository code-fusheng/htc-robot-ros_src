#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pthread.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/service.h>

#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <pcl_omp_registration/ndt.h>

uint8_t _auto_pilot_status = 1;

#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

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

static pose initial_pose, predict_pose, predict_pose_imu, predict_pose_odom,
    predict_pose_imu_odom, previous_pose, ndt_pose, current_pose,
    current_pose_imu, current_pose_odom, current_pose_imu_odom, localizer_pose;

static double offset_x, offset_y, offset_z,
    offset_yaw;  // current_pos - previous_pose
static double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll,
    offset_imu_pitch, offset_imu_yaw;
static double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll,
    offset_odom_pitch, offset_odom_yaw;
static double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z,
    offset_imu_odom_roll, offset_imu_odom_pitch, offset_imu_odom_yaw;

static pcl::PointCloud<pcl::PointXYZ> map;

static bool is_debug;

// If the map is loaded, map_loaded will be 1.
static int map_loaded = 0;
static int _use_gnss = 1;
static int init_pos_set = 0;

static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
static cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> anh_ndt;

// Default values
static int max_iter = 30;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.1;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon

static ros::Publisher predict_pose_pub;
static geometry_msgs::PoseStamped predict_pose_msg;

static ros::Publisher predict_pose_imu_pub;
static geometry_msgs::PoseStamped predict_pose_imu_msg;

static ros::Publisher predict_pose_odom_pub;
static geometry_msgs::PoseStamped predict_pose_odom_msg;

static ros::Publisher predict_pose_imu_odom_pub;
static geometry_msgs::PoseStamped predict_pose_imu_odom_msg;

static ros::Publisher ndt_pose_pub, ndt_pose_pub_lidarmode;
static geometry_msgs::PoseStamped ndt_pose_msg;

static ros::Publisher localizer_pose_pub;
static geometry_msgs::PoseStamped localizer_pose_msg;

static ros::Publisher estimate_twist_pub;
static geometry_msgs::TwistStamped estimate_twist_msg;

static ros::Duration scan_duration;

static double exe_time = 0.0;
static bool has_converged;
static int iteration = 0;
static double fitness_score = 0.0;
static double trans_probability = 0.0;

// reference for comparing fitness_score, default value set to 500.0
static double _gnss_reinit_fitness = 500.0;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;

static double current_velocity = 0.0, previous_velocity = 0.0,
              previous_previous_velocity = 0.0;  // [m/s]
static double current_velocity_x = 0.0, previous_velocity_x = 0.0;
static double current_velocity_y = 0.0, previous_velocity_y = 0.0;
static double current_velocity_z = 0.0, previous_velocity_z = 0.0;
static double current_velocity_smooth = 0.0;

static double current_velocity_imu_x = 0.0;
static double current_velocity_imu_y = 0.0;
static double current_velocity_imu_z = 0.0;

static double current_accel = 0.0, previous_accel = 0.0;  // [m/s^2]
static double current_accel_x = 0.0;
static double current_accel_y = 0.0;
static double current_accel_z = 0.0;

static double angular_velocity = 0.0;

static int use_predict_pose = 0;

static ros::Publisher estimated_vel_mps_pub, estimated_vel_kmph_pub,
    estimated_vel_pub;
static std_msgs::Float32 estimated_vel_mps, estimated_vel_kmph,
    previous_estimated_vel_kmph;

static std::chrono::time_point<std::chrono::system_clock> matching_start,
    matching_end;

static ros::Publisher time_ndt_matching_pub;
static std_msgs::Float32 time_ndt_matching;

static int _queue_size = 1;

static ros::Publisher ndt_stat_pub;
// static autoware_msgs::NDTStat ndt_stat_msg;

static double predict_pose_error = 0.0;

// hold transfrom from baselink to primary lidar
static Eigen::Matrix4f tf_btol;

static std::string _offset = "linear";  // linear, zero, quadratic

static ros::Publisher ndt_reliability_pub;
static std_msgs::Float32 ndt_reliability;

static bool _get_height = false;
static bool _use_local_transform = false;
static bool _use_imu = false;
static bool _use_odom = false;
static bool _imu_upside_down = false;
static bool _output_log_data = false;
static std::string _output_tf_frame_id = "base_link";

static std::string _imu_topic = "/imu_raw";

static std::ofstream ofs;
static std::string filename;

static sensor_msgs::Imu imu;
static nav_msgs::Odometry odom;

static tf2::Stamped<tf2::Transform> local_transform;

static unsigned int points_map_num = 0;

pthread_mutex_t mutex;

pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
pcl::VoxelGrid<pcl::PointXYZ> map_voxel_grid_filter;

uint8_t _FUNCTION_STATUS = 0;
bool _enable_sub_init_pose = true;

static pose convertPoseIntoRelativeCoordinate(const pose& target_pose, const pose& reference_pose) {
    tf2::Quaternion target_q;
    target_q.setRPY(target_pose.roll, target_pose.pitch, target_pose.yaw);
    tf2::Vector3 target_v(target_pose.x, target_pose.y, target_pose.z);
    tf2::Transform target_tf(target_q, target_v);

    tf2::Quaternion reference_q;
    reference_q.setRPY(reference_pose.roll, reference_pose.pitch, reference_pose.yaw);
    tf2::Vector3 reference_v(reference_pose.x, reference_pose.y, reference_pose.z);
    tf2::Transform reference_tf(reference_q, reference_v);

    tf2::Transform trans_target_tf = reference_tf.inverse() * target_tf;

    pose trans_target_pose;
    trans_target_pose.x = trans_target_tf.getOrigin().getX();
    trans_target_pose.y = trans_target_tf.getOrigin().getY();
    trans_target_pose.z = trans_target_tf.getOrigin().getZ();
    tf2::Matrix3x3 tmp_m(trans_target_tf.getRotation());
    tmp_m.getRPY(trans_target_pose.roll, trans_target_pose.pitch, trans_target_pose.yaw);

    return trans_target_pose;
}

static void map_callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    // if (map_loaded == 0)
    if (true) {
        ROS_INFO("[ndt localization]Update points_map, input points num = %d", input->width);

        points_map_num = input->width;

        // Convert the data type(from sensor_msgs to pcl).
        pcl::fromROSMsg(*input, map);

        if (_use_local_transform == true) {
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener(tf_buffer);
            geometry_msgs::TransformStamped local_transform_msg;
            try {
                local_transform_msg = tf_buffer.lookupTransform("map", "world", ros::Time::now(), ros::Duration(3.0));
            } catch (tf2::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }

            tf2::fromMsg(local_transform_msg, local_transform);
            pcl::transformPointCloud(map, map,
                                     tf2::transformToEigen(local_transform_msg)
                                         .matrix()
                                         .inverse()
                                         .cast<float>());
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

        // Setting point cloud to be aligned to.
        if (_method_type == MethodType::PCL_GENERIC) {
            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            new_ndt.setResolution(ndt_res);
            new_ndt.setInputTarget(map_ptr);
            new_ndt.setMaximumIterations(max_iter);
            new_ndt.setStepSize(step_size);
            new_ndt.setTransformationEpsilon(trans_eps);

            new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

            pthread_mutex_lock(&mutex);
            ndt = new_ndt;
            pthread_mutex_unlock(&mutex);
        } else if (_method_type == MethodType::PCL_ANH) {
            cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_anh_ndt;
            new_anh_ndt.setResolution(ndt_res);
            new_anh_ndt.setInputTarget(map_ptr);
            new_anh_ndt.setMaximumIterations(max_iter);
            new_anh_ndt.setStepSize(step_size);
            new_anh_ndt.setTransformationEpsilon(trans_eps);

            pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_scan_ptr(
                new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointXYZ dummy_point;
            dummy_scan_ptr->push_back(dummy_point);
            new_anh_ndt.setInputSource(dummy_scan_ptr);

            new_anh_ndt.align(Eigen::Matrix4f::Identity());

            pthread_mutex_lock(&mutex);
            anh_ndt = new_anh_ndt;
            pthread_mutex_unlock(&mutex);
        }
        map_loaded = 1;
    }
}

static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input) {
    tf2::Quaternion gnss_q(input->pose.orientation.x, input->pose.orientation.y,
                           input->pose.orientation.z,
                           input->pose.orientation.w);
    tf2::Matrix3x3 gnss_m(gnss_q);

    pose current_gnss_pose;
    current_gnss_pose.x = input->pose.position.x;
    current_gnss_pose.y = input->pose.position.y;
    current_gnss_pose.z = input->pose.position.z;
    gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

    static pose previous_gnss_pose = current_gnss_pose;
    ros::Time current_gnss_time = input->header.stamp;
    static ros::Time previous_gnss_time = current_gnss_time;

    if ((_use_gnss == 1 && init_pos_set == 0) ||
        fitness_score >= _gnss_reinit_fitness) {
        previous_pose.x = previous_gnss_pose.x;
        previous_pose.y = previous_gnss_pose.y;
        previous_pose.z = previous_gnss_pose.z;
        previous_pose.roll = previous_gnss_pose.roll;
        previous_pose.pitch = previous_gnss_pose.pitch;
        previous_pose.yaw = previous_gnss_pose.yaw;

        current_pose.x = current_gnss_pose.x;
        current_pose.y = current_gnss_pose.y;
        current_pose.z = current_gnss_pose.z;
        current_pose.roll = current_gnss_pose.roll;
        current_pose.pitch = current_gnss_pose.pitch;
        current_pose.yaw = current_gnss_pose.yaw;

        current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;

        diff_x = current_pose.x - previous_pose.x;
        diff_y = current_pose.y - previous_pose.y;
        diff_z = current_pose.z - previous_pose.z;
        diff_yaw = current_pose.yaw - previous_pose.yaw;
        diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

        const pose trans_current_pose =
            convertPoseIntoRelativeCoordinate(current_pose, previous_pose);

        const double diff_time =
            (current_gnss_time - previous_gnss_time).toSec();
        current_velocity = (diff_time > 0) ? (diff / diff_time) : 0;
        current_velocity = (trans_current_pose.x >= 0) ? current_velocity : -current_velocity;
        current_velocity_x = (diff_time > 0) ? (diff_x / diff_time) : 0;
        current_velocity_y = (diff_time > 0) ? (diff_y / diff_time) : 0;
        current_velocity_z = (diff_time > 0) ? (diff_z / diff_time) : 0;
        angular_velocity = (diff_time > 0) ? (diff_yaw / diff_time) : 0;

        current_accel = 0.0;
        current_accel_x = 0.0;
        current_accel_y = 0.0;
        current_accel_z = 0.0;

        init_pos_set = 1;
    }

    previous_gnss_pose.x = current_gnss_pose.x;
    previous_gnss_pose.y = current_gnss_pose.y;
    previous_gnss_pose.z = current_gnss_pose.z;
    previous_gnss_pose.roll = current_gnss_pose.roll;
    previous_gnss_pose.pitch = current_gnss_pose.pitch;
    previous_gnss_pose.yaw = current_gnss_pose.yaw;
    previous_gnss_time = current_gnss_time;
}

static void initialpose_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input) {

    if ( !_enable_sub_init_pose ) return;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped tf_msg;
    try {
        tf_msg =
            tf_buffer.lookupTransform("map", input->header.frame_id, ros::Time::now(), ros::Duration(3.0));
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
    }

    tf2::Quaternion q(
        input->pose.pose.orientation.x, input->pose.pose.orientation.y,
        input->pose.pose.orientation.z, input->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);

    if (_use_local_transform == true) {
        current_pose.x = input->pose.pose.position.x;
        current_pose.y = input->pose.pose.position.y;
        current_pose.z = input->pose.pose.position.z;
    } else {
        current_pose.x =
            input->pose.pose.position.x + tf_msg.transform.translation.x;
        current_pose.y =
            input->pose.pose.position.y + tf_msg.transform.translation.y;
        current_pose.z =
            input->pose.pose.position.z + tf_msg.transform.translation.z;
    }
    m.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);

    if (_get_height == true && map_loaded == 1) {
        double min_distance = DBL_MAX;
        double nearest_z = current_pose.z;
        for (const auto& p : map) {
            double distance = hypot(current_pose.x - p.x, current_pose.y - p.y);
            if (distance < min_distance) {
                min_distance = distance;
                nearest_z = p.z;
            }
        }
        current_pose.z = nearest_z;
    }

    current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    current_velocity = 0.0;
    current_velocity_x = 0.0;
    current_velocity_y = 0.0;
    current_velocity_z = 0.0;
    angular_velocity = 0.0;

    current_accel = 0.0;
    current_accel_x = 0.0;
    current_accel_y = 0.0;
    current_accel_z = 0.0;

    offset_x = 0.0;
    offset_y = 0.0;
    offset_z = 0.0;
    offset_yaw = 0.0;

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

    init_pos_set = 1;
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

    predict_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
    predict_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
    predict_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
    predict_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
    predict_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
    predict_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

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

    predict_pose_odom.x = previous_pose.x + offset_odom_x;
    predict_pose_odom.y = previous_pose.y + offset_odom_y;
    predict_pose_odom.z = previous_pose.z + offset_odom_z;
    predict_pose_odom.roll = previous_pose.roll + offset_odom_roll;
    predict_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
    predict_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

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

    predict_pose_imu.x = previous_pose.x + offset_imu_x;
    predict_pose_imu.y = previous_pose.y + offset_imu_y;
    predict_pose_imu.z = previous_pose.z + offset_imu_z;
    predict_pose_imu.roll = previous_pose.roll + offset_imu_roll;
    predict_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
    predict_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

    previous_time = current_time;
}

static double wrapToPm(double a_num, const double a_max) {
    if (a_num >= a_max) {
        a_num -= 2.0 * a_max;
    }
    return a_num;
}

static double wrapToPmPi(const double a_angle_rad) {
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

static void odom_callback(const nav_msgs::Odometry::ConstPtr& input) {
    odom = *input;
    odom_calc(input->header.stamp);
}

static void vehicle_twist_callback(
    const geometry_msgs::TwistStampedConstPtr& msg) {
    odom.header = msg->header;
    odom.twist.twist = msg->twist;
    odom_calc(odom.header.stamp);
}

static void imuUpsideDown(const sensor_msgs::Imu::Ptr input) {
    double input_roll, input_pitch, input_yaw;

    tf2::Quaternion input_orientation;
    tf2::fromMsg(input->orientation, input_orientation);
    tf2::Matrix3x3(input_orientation)
        .getRPY(input_roll, input_pitch, input_yaw);

    input->angular_velocity.x *= -1;
    input->angular_velocity.y *= -1;
    input->angular_velocity.z *= -1;

    input->linear_acceleration.x *= -1;
    input->linear_acceleration.y *= -1;
    input->linear_acceleration.z *= -1;

    input_roll *= -1;
    input_pitch *= -1;
    input_yaw *= -1;

    tf2::Quaternion tf_quat;
    tf_quat.setRPY(input_roll, input_pitch, input_yaw);
    input->orientation = tf2::toMsg(tf_quat);
}

static void imu_callback(const sensor_msgs::Imu::Ptr& input) {
    // std::cout << __func__ << std::endl;

    if (_imu_upside_down) imuUpsideDown(input);

    const ros::Time current_time = input->header.stamp;
    static ros::Time previous_time = current_time;
    const double diff_time = (current_time - previous_time).toSec();

    double imu_roll, imu_pitch, imu_yaw;
    tf2::Quaternion imu_orientation;
    tf2::fromMsg(input->orientation, imu_orientation);
    tf2::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

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

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input) {
    //   health_checker_ptr_->CHECK_RATE("topic_rate_filtered_points_slow", 8,
    //   5, 1, "topic filtered_points subscribe rate slow.");
    if (map_loaded == 1 && init_pos_set == 1) {
        matching_start = std::chrono::system_clock::now();

        static tf2_ros::TransformBroadcaster br;
        tf2::Transform transform;
        tf2::Quaternion predict_q, ndt_q, current_q, localizer_q;

        pcl::PointXYZ p;
        pcl::PointCloud<pcl::PointXYZ> filtered_scan;

        ros::Time current_scan_time = input->header.stamp;
        static ros::Time previous_scan_time = current_scan_time;

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*input, *scan);
    
        voxel_grid_filter.setInputCloud(scan);
        voxel_grid_filter.filter(filtered_scan);
        // ROS_WARN("filtered size: %d",scan_ptr->points.size());

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
        int scan_points_num = scan->size();

        Eigen::Matrix4f t(Eigen::Matrix4f::Identity());   // base_link
        Eigen::Matrix4f t2(Eigen::Matrix4f::Identity());  // localizer

        std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start, getFitnessScore_end;
        static double align_time, getFitnessScore_time = 0.0;

        pthread_mutex_lock(&mutex);

        if (_method_type == MethodType::PCL_GENERIC)
            ndt.setInputSource(filtered_scan_ptr);
        else if (_method_type == MethodType::PCL_ANH)
            anh_ndt.setInputSource(filtered_scan_ptr);
        // Guess the initial gross estimation of the transformation
        double diff_time = (current_scan_time - previous_scan_time).toSec();

        if (_offset == "linear") {
            offset_x = current_velocity_x * diff_time;
            offset_y = current_velocity_y * diff_time;
            offset_z = current_velocity_z * diff_time;
            offset_yaw = angular_velocity * diff_time;
        } else if (_offset == "quadratic") {
            offset_x = (current_velocity_x + current_accel_x * diff_time) * diff_time;
            offset_y = (current_velocity_y + current_accel_y * diff_time) * diff_time;
            offset_z = current_velocity_z * diff_time;
            offset_yaw = angular_velocity * diff_time;
        } else if (_offset == "zero") {
            offset_x = 0.0;
            offset_y = 0.0;
            offset_z = 0.0;
            offset_yaw = 0.0;
        }

        predict_pose.x = previous_pose.x + offset_x;
        predict_pose.y = previous_pose.y + offset_y;
        predict_pose.z = previous_pose.z + offset_z;
        predict_pose.roll = previous_pose.roll;
        predict_pose.pitch = previous_pose.pitch;
        predict_pose.yaw = previous_pose.yaw + offset_yaw;

        if (_use_imu == true && _use_odom == true)
            imu_odom_calc(current_scan_time);
        if (_use_imu == true && _use_odom == false) imu_calc(current_scan_time);
        if (_use_imu == false && _use_odom == true)
            odom_calc(current_scan_time);

        pose predict_pose_for_ndt;
        if (_use_imu == true && _use_odom == true)
            predict_pose_for_ndt = predict_pose_imu_odom;
        else if (_use_imu == true && _use_odom == false)
            predict_pose_for_ndt = predict_pose_imu;
        else if (_use_imu == false && _use_odom == true)
            predict_pose_for_ndt = predict_pose_odom;
        else
            predict_pose_for_ndt = predict_pose;

        Eigen::Translation3f init_translation(predict_pose_for_ndt.x,
                                              predict_pose_for_ndt.y,
                                              predict_pose_for_ndt.z);
        Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll,
                                          Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch,
                                          Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw,
                                          Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f init_guess = (init_translation * init_rotation_z *
                                      init_rotation_y * init_rotation_x) *
                                     tf_btol;

        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        if (_method_type == MethodType::PCL_GENERIC) {
            align_start = std::chrono::system_clock::now();
            ndt.align(*output_cloud, init_guess);
            align_end = std::chrono::system_clock::now();

            has_converged = ndt.hasConverged();

            t = ndt.getFinalTransformation();
            iteration = ndt.getFinalNumIteration();

            getFitnessScore_start = std::chrono::system_clock::now();
            fitness_score = ndt.getFitnessScore();
            getFitnessScore_end = std::chrono::system_clock::now();

            trans_probability = ndt.getTransformationProbability();
        } else if (_method_type == MethodType::PCL_ANH) {
            align_start = std::chrono::system_clock::now();
            anh_ndt.align(init_guess);
            align_end = std::chrono::system_clock::now();

            has_converged = anh_ndt.hasConverged();

            t = anh_ndt.getFinalTransformation();
            iteration = anh_ndt.getFinalNumIteration();

            getFitnessScore_start = std::chrono::system_clock::now();
            fitness_score = anh_ndt.getFitnessScore();
            getFitnessScore_end = std::chrono::system_clock::now();

            trans_probability = anh_ndt.getTransformationProbability();
        }
        align_time = std::chrono::duration_cast<std::chrono::microseconds>(
                         align_end - align_start)
                         .count() /
                     1000.0;

        t2 = t * tf_btol.inverse();
        // t2 = t;

        getFitnessScore_time =
            std::chrono::duration_cast<std::chrono::microseconds>(
                getFitnessScore_end - getFitnessScore_start)
                .count() /
            1000.0;

        pthread_mutex_unlock(&mutex);

        tf2::Matrix3x3 mat_l;  // localizer
        mat_l.setValue(
            static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)),
            static_cast<double>(t(0, 2)), static_cast<double>(t(1, 0)),
            static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
            static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)),
            static_cast<double>(t(2, 2)));

        // Update localizer_pose
        localizer_pose.x = t(0, 3);
        localizer_pose.y = t(1, 3);
        localizer_pose.z = t(2, 3);
        mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

        tf2::Matrix3x3 mat_b;  // base_link
        mat_b.setValue(
            static_cast<double>(t2(0, 0)), static_cast<double>(t2(0, 1)),
            static_cast<double>(t2(0, 2)), static_cast<double>(t2(1, 0)),
            static_cast<double>(t2(1, 1)), static_cast<double>(t2(1, 2)),
            static_cast<double>(t2(2, 0)), static_cast<double>(t2(2, 1)),
            static_cast<double>(t2(2, 2)));

        // Update ndt_pose
        ndt_pose.x = t2(0, 3);
        ndt_pose.y = t2(1, 3);
        ndt_pose.z = t2(2, 3);
        mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

        // Calculate the difference between ndt_pose and predict_pose
        predict_pose_error = sqrt((ndt_pose.x - predict_pose_for_ndt.x) *
                                      (ndt_pose.x - predict_pose_for_ndt.x) +
                                  (ndt_pose.y - predict_pose_for_ndt.y) *
                                      (ndt_pose.y - predict_pose_for_ndt.y) +
                                  (ndt_pose.z - predict_pose_for_ndt.z) *
                                      (ndt_pose.z - predict_pose_for_ndt.z));

        if (predict_pose_error <= PREDICT_POSE_THRESHOLD) {
            use_predict_pose = 0;
        } else {
            use_predict_pose = 1;
        }
        use_predict_pose = 0;

        if (use_predict_pose == 0) {
            current_pose.x = ndt_pose.x;
            current_pose.y = ndt_pose.y;
            current_pose.z = ndt_pose.z;
            current_pose.roll = ndt_pose.roll;
            current_pose.pitch = ndt_pose.pitch;
            current_pose.yaw = ndt_pose.yaw;
        } else {
            current_pose.x = predict_pose_for_ndt.x;
            current_pose.y = predict_pose_for_ndt.y;
            current_pose.z = predict_pose_for_ndt.z;
            current_pose.roll = predict_pose_for_ndt.roll;
            current_pose.pitch = predict_pose_for_ndt.pitch;
            current_pose.yaw = predict_pose_for_ndt.yaw;
        }

        // Compute the velocity and acceleration
        diff_x = current_pose.x - previous_pose.x;
        diff_y = current_pose.y - previous_pose.y;
        diff_z = current_pose.z - previous_pose.z;
        diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
        diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

        const pose trans_current_pose = convertPoseIntoRelativeCoordinate(current_pose, previous_pose);

        current_velocity = (diff_time > 0) ? (diff / diff_time) : 0;
        current_velocity = (trans_current_pose.x >= 0) ? current_velocity : -current_velocity;
        current_velocity_x = (diff_time > 0) ? (diff_x / diff_time) : 0;
        current_velocity_y = (diff_time > 0) ? (diff_y / diff_time) : 0;
        current_velocity_z = (diff_time > 0) ? (diff_z / diff_time) : 0;
        angular_velocity = (diff_time > 0) ? (diff_yaw / diff_time) : 0;

        current_pose_imu.x = current_pose.x;
        current_pose_imu.y = current_pose.y;
        current_pose_imu.z = current_pose.z;
        current_pose_imu.roll = current_pose.roll;
        current_pose_imu.pitch = current_pose.pitch;
        current_pose_imu.yaw = current_pose.yaw;

        current_velocity_imu_x = current_velocity_x;
        current_velocity_imu_y = current_velocity_y;
        current_velocity_imu_z = current_velocity_z;

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

        current_velocity_smooth = (current_velocity + previous_velocity +
                                   previous_previous_velocity) /
                                  3.0;
        if (std::fabs(current_velocity_smooth) < 0.2) {
            current_velocity_smooth = 0.0;
        }

        current_accel =
            (diff_time > 0)
                ? ((current_velocity - previous_velocity) / diff_time)
                : 0;
        current_accel_x =
            (diff_time > 0)
                ? ((current_velocity_x - previous_velocity_x) / diff_time)
                : 0;
        current_accel_y =
            (diff_time > 0)
                ? ((current_velocity_y - previous_velocity_y) / diff_time)
                : 0;
        current_accel_z =
            (diff_time > 0)
                ? ((current_velocity_z - previous_velocity_z) / diff_time)
                : 0;

        estimated_vel_mps.data = current_velocity;
        estimated_vel_kmph.data = current_velocity * 3.6;

        estimated_vel_mps_pub.publish(estimated_vel_mps);
        estimated_vel_kmph_pub.publish(estimated_vel_kmph);

        // Set values for publishing pose
        predict_q.setRPY(predict_pose.roll, predict_pose.pitch,
                         predict_pose.yaw);
        if (_use_local_transform == true) {
            tf2::Vector3 v(predict_pose.x, predict_pose.y, predict_pose.z);
            tf2::Transform transform(predict_q, v);
            predict_pose_msg.header.frame_id = "map";
            predict_pose_msg.header.stamp = current_scan_time;
            predict_pose_msg.pose.position.x =
                (local_transform * transform).getOrigin().getX();
            predict_pose_msg.pose.position.y =
                (local_transform * transform).getOrigin().getY();
            predict_pose_msg.pose.position.z =
                (local_transform * transform).getOrigin().getZ();
            predict_pose_msg.pose.orientation.x =
                (local_transform * transform).getRotation().x();
            predict_pose_msg.pose.orientation.y =
                (local_transform * transform).getRotation().y();
            predict_pose_msg.pose.orientation.z =
                (local_transform * transform).getRotation().z();
            predict_pose_msg.pose.orientation.w =
                (local_transform * transform).getRotation().w();
        } else {
            predict_pose_msg.header.frame_id = "map";
            predict_pose_msg.header.stamp = current_scan_time;
            predict_pose_msg.pose.position.x = predict_pose.x;
            predict_pose_msg.pose.position.y = predict_pose.y;
            predict_pose_msg.pose.position.z = predict_pose.z;
            predict_pose_msg.pose.orientation.x = predict_q.x();
            predict_pose_msg.pose.orientation.y = predict_q.y();
            predict_pose_msg.pose.orientation.z = predict_q.z();
            predict_pose_msg.pose.orientation.w = predict_q.w();
        }

        tf2::Quaternion predict_q_imu;
        predict_q_imu.setRPY(predict_pose_imu.roll, predict_pose_imu.pitch,
                             predict_pose_imu.yaw);
        predict_pose_imu_msg.header.frame_id = "map";
        predict_pose_imu_msg.header.stamp = input->header.stamp;
        predict_pose_imu_msg.pose.position.x = predict_pose_imu.x;
        predict_pose_imu_msg.pose.position.y = predict_pose_imu.y;
        predict_pose_imu_msg.pose.position.z = predict_pose_imu.z;
        predict_pose_imu_msg.pose.orientation.x = predict_q_imu.x();
        predict_pose_imu_msg.pose.orientation.y = predict_q_imu.y();
        predict_pose_imu_msg.pose.orientation.z = predict_q_imu.z();
        predict_pose_imu_msg.pose.orientation.w = predict_q_imu.w();
        predict_pose_imu_pub.publish(predict_pose_imu_msg);

        tf2::Quaternion predict_q_odom;
        predict_q_odom.setRPY(predict_pose_odom.roll, predict_pose_odom.pitch,
                              predict_pose_odom.yaw);
        predict_pose_odom_msg.header.frame_id = "map";
        predict_pose_odom_msg.header.stamp = input->header.stamp;
        predict_pose_odom_msg.pose.position.x = predict_pose_odom.x;
        predict_pose_odom_msg.pose.position.y = predict_pose_odom.y;
        predict_pose_odom_msg.pose.position.z = predict_pose_odom.z;
        predict_pose_odom_msg.pose.orientation.x = predict_q_odom.x();
        predict_pose_odom_msg.pose.orientation.y = predict_q_odom.y();
        predict_pose_odom_msg.pose.orientation.z = predict_q_odom.z();
        predict_pose_odom_msg.pose.orientation.w = predict_q_odom.w();
        predict_pose_odom_pub.publish(predict_pose_odom_msg);

        tf2::Quaternion predict_q_imu_odom;
        predict_q_imu_odom.setRPY(predict_pose_imu_odom.roll,
                                  predict_pose_imu_odom.pitch,
                                  predict_pose_imu_odom.yaw);
        predict_pose_imu_odom_msg.header.frame_id = "map";
        predict_pose_imu_odom_msg.header.stamp = input->header.stamp;
        predict_pose_imu_odom_msg.pose.position.x = predict_pose_imu_odom.x;
        predict_pose_imu_odom_msg.pose.position.y = predict_pose_imu_odom.y;
        predict_pose_imu_odom_msg.pose.position.z = predict_pose_imu_odom.z;
        predict_pose_imu_odom_msg.pose.orientation.x = predict_q_imu_odom.x();
        predict_pose_imu_odom_msg.pose.orientation.y = predict_q_imu_odom.y();
        predict_pose_imu_odom_msg.pose.orientation.z = predict_q_imu_odom.z();
        predict_pose_imu_odom_msg.pose.orientation.w = predict_q_imu_odom.w();
        predict_pose_imu_odom_pub.publish(predict_pose_imu_odom_msg);

        ndt_q.setRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);
        if (_use_local_transform == true) {
            tf2::Vector3 v(ndt_pose.x, ndt_pose.y, ndt_pose.z);
            tf2::Transform transform(ndt_q, v);
            ndt_pose_msg.header.frame_id = "map";
            ndt_pose_msg.header.stamp = current_scan_time;
            ndt_pose_msg.pose.position.x =
                (local_transform * transform).getOrigin().getX();
            ndt_pose_msg.pose.position.y =
                (local_transform * transform).getOrigin().getY();
            ndt_pose_msg.pose.position.z =
                (local_transform * transform).getOrigin().getZ();
            ndt_pose_msg.pose.orientation.x =
                (local_transform * transform).getRotation().x();
            ndt_pose_msg.pose.orientation.y =
                (local_transform * transform).getRotation().y();
            ndt_pose_msg.pose.orientation.z =
                (local_transform * transform).getRotation().z();
            ndt_pose_msg.pose.orientation.w =
                (local_transform * transform).getRotation().w();
        } else {
            ndt_pose_msg.header.frame_id = "map";
            ndt_pose_msg.header.stamp = current_scan_time;
            ndt_pose_msg.pose.position.x = ndt_pose.x;
            ndt_pose_msg.pose.position.y = ndt_pose.y;
            ndt_pose_msg.pose.position.z = ndt_pose.z;
            ndt_pose_msg.pose.orientation.x = ndt_q.x();
            ndt_pose_msg.pose.orientation.y = ndt_q.y();
            ndt_pose_msg.pose.orientation.z = ndt_q.z();
            ndt_pose_msg.pose.orientation.w = ndt_q.w();
        }

        current_q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);

        localizer_q.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);
        if (_use_local_transform == true) {
            tf2::Vector3 v(localizer_pose.x, localizer_pose.y, localizer_pose.z);
            tf2::Transform transform(localizer_q, v);
            localizer_pose_msg.header.frame_id = "map";
            localizer_pose_msg.header.stamp = current_scan_time;
            localizer_pose_msg.pose.position.x =
                (local_transform * transform).getOrigin().getX();
            localizer_pose_msg.pose.position.y =
                (local_transform * transform).getOrigin().getY();
            localizer_pose_msg.pose.position.z =
                (local_transform * transform).getOrigin().getZ();
            localizer_pose_msg.pose.orientation.x =
                (local_transform * transform).getRotation().x();
            localizer_pose_msg.pose.orientation.y =
                (local_transform * transform).getRotation().y();
            localizer_pose_msg.pose.orientation.z =
                (local_transform * transform).getRotation().z();
            localizer_pose_msg.pose.orientation.w =
                (local_transform * transform).getRotation().w();
        } else {
            localizer_pose_msg.header.frame_id = "map";
            localizer_pose_msg.header.stamp = current_scan_time;
            localizer_pose_msg.pose.position.x = localizer_pose.x;
            localizer_pose_msg.pose.position.y = localizer_pose.y;
            localizer_pose_msg.pose.position.z = localizer_pose.z;
            localizer_pose_msg.pose.orientation.x = localizer_q.x();
            localizer_pose_msg.pose.orientation.y = localizer_q.y();
            localizer_pose_msg.pose.orientation.z = localizer_q.z();
            localizer_pose_msg.pose.orientation.w = localizer_q.w();
        }

        predict_pose_pub.publish(predict_pose_msg);

        if (_auto_pilot_status == 1)
            ndt_pose_pub.publish(ndt_pose_msg);
        else
            ndt_pose_pub_lidarmode.publish(ndt_pose_msg);
        localizer_pose_pub.publish(localizer_pose_msg);

        // Send TF "base_link" to "map"
        transform.setOrigin(
            tf2::Vector3(current_pose.x, current_pose.y, current_pose.z));
        transform.setRotation(current_q);
        if (_use_local_transform == true) {
            transform = local_transform * transform;
        }
        tf2::Stamped<tf2::Transform> tf(transform, current_scan_time, "map");
        if (_auto_pilot_status == 1) {
            geometry_msgs::TransformStamped tf_msg = tf2::toMsg(tf);
            tf_msg.child_frame_id = _output_tf_frame_id;
            br.sendTransform(tf_msg);
        }

        matching_end = std::chrono::system_clock::now();
        exe_time = std::chrono::duration_cast<std::chrono::microseconds>(
                       matching_end - matching_start)
                       .count() /
                   1000.0;
        time_ndt_matching.data = exe_time;
        // health_checker_ptr_->CHECK_MAX_VALUE("time_ndt_matching",
        // time_ndt_matching.data, 50, 70, 100, "value time_ndt_matching is too
        // high.");
        time_ndt_matching_pub.publish(time_ndt_matching);

        // Set values for /estimate_twist
        estimate_twist_msg.header.stamp = current_scan_time;
        estimate_twist_msg.header.frame_id = "base_link";
        estimate_twist_msg.twist.linear.x = current_velocity;
        estimate_twist_msg.twist.linear.y = 0.0;
        estimate_twist_msg.twist.linear.z = 0.0;
        estimate_twist_msg.twist.angular.x = 0.0;
        estimate_twist_msg.twist.angular.y = 0.0;
        estimate_twist_msg.twist.angular.z = angular_velocity;

        estimate_twist_pub.publish(estimate_twist_msg);

        geometry_msgs::Vector3Stamped estimate_vel_msg;
        estimate_vel_msg.header.stamp = current_scan_time;
        estimate_vel_msg.vector.x = current_velocity;
        estimated_vel_pub.publish(estimate_vel_msg);

        /* Compute NDT_Reliability */
        ndt_reliability.data = Wa * (exe_time / 100.0) * 100.0 +
                               Wb * (iteration / 10.0) * 100.0 +
                               Wc * ((2.0 - trans_probability) / 2.0) * 100.0;
        ndt_reliability_pub.publish(ndt_reliability);

        // Write log
        if (_output_log_data) {
            if (!ofs) {
                std::cerr << "Could not open " << filename << "." << std::endl;
            } else {
                ofs << input->header.seq << "," << scan_points_num << ","
                    << step_size << "," << trans_eps << "," << std::fixed
                    << std::setprecision(5) << current_pose.x << ","
                    << std::fixed << std::setprecision(5) << current_pose.y
                    << "," << std::fixed << std::setprecision(5)
                    << current_pose.z << "," << current_pose.roll << ","
                    << current_pose.pitch << "," << current_pose.yaw << ","
                    << predict_pose.x << "," << predict_pose.y << ","
                    << predict_pose.z << "," << predict_pose.roll << ","
                    << predict_pose.pitch << "," << predict_pose.yaw << ","
                    << current_pose.x - predict_pose.x << ","
                    << current_pose.y - predict_pose.y << ","
                    << current_pose.z - predict_pose.z << ","
                    << current_pose.roll - predict_pose.roll << ","
                    << current_pose.pitch - predict_pose.pitch << ","
                    << current_pose.yaw - predict_pose.yaw << ","
                    << predict_pose_error << "," << iteration << ","
                    << fitness_score << "," << trans_probability << ","
                    << ndt_reliability.data << "," << current_velocity << ","
                    << current_velocity_smooth << "," << current_accel << ","
                    << angular_velocity << "," << time_ndt_matching.data << ","
                    << align_time << "," << getFitnessScore_time << std::endl;
            }
        }
        if (is_debug){
            std::cout << "---------------------------------------------------------"
                        "--------"
                    << std::endl;
            std::cout << "Sequence: " << input->header.seq << std::endl;
            std::cout << "Timestamp: " << input->header.stamp << std::endl;
            std::cout << "Frame ID: " << input->header.frame_id << std::endl;
            //    std::cout << "Number of Scan Points: " << scan_ptr->size() << "
            //    points." << std::endl;
            std::cout << "Number of Filtered Scan Points: " << scan_points_num
                    << " points." << std::endl;
            std::cout << "NDT has converged: " << has_converged << std::endl;
            std::cout << "Fitness Score: " << fitness_score << std::endl;
            std::cout << "Transformation Probability: " << trans_probability
                    << std::endl;
            std::cout << "Execution Time: " << exe_time << " ms." << std::endl;
            std::cout << "Number of Iterations: " << iteration << std::endl;
            std::cout << "NDT Reliability: " << ndt_reliability.data << std::endl;
            std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
            std::cout << "(" << current_pose.x << ", " << current_pose.y << ", "
                    << current_pose.z << ", " << current_pose.roll << ", "
                    << current_pose.pitch << ", " << current_pose.yaw << ")"
                    << std::endl;
            std::cout << "Transformation Matrix: " << std::endl;
            std::cout << t << std::endl;
            std::cout << "Align time: " << align_time << std::endl;
            std::cout << "Get fitness score time: " << getFitnessScore_time
                    << std::endl;
            std::cout << "---------------------------------------------------------"
                        "--------"
                    << std::endl;
        }

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

        // Update previous_***
        previous_pose.x = current_pose.x;
        previous_pose.y = current_pose.y;
        previous_pose.z = current_pose.z;
        previous_pose.roll = current_pose.roll;
        previous_pose.pitch = current_pose.pitch;
        previous_pose.yaw = current_pose.yaw;

        previous_scan_time = current_scan_time;

        previous_previous_velocity = previous_velocity;
        previous_velocity = current_velocity;
        previous_velocity_x = current_velocity_x;
        previous_velocity_y = current_velocity_y;
        previous_velocity_z = current_velocity_z;
        previous_accel = current_accel;

        previous_estimated_vel_kmph.data = estimated_vel_kmph.data;
    } else {
        static ros::Time _pre_receive_time = ros::Time::now();
        static double _warn_msg_interval = 1.0;
        if ((ros::Time::now() - _pre_receive_time).toSec() >
            _warn_msg_interval) {
            if (!init_pos_set) {
                ROS_WARN("[ndt_localization] init pose not set.");
            }
            if (!map_loaded) {
                ROS_WARN("[ndt_localization] map not loaded");
            }
            _pre_receive_time = ros::Time::now();
        }
    }
}

void* thread_func(void* args) {
    ros::NodeHandle nh_map;
    ros::CallbackQueue map_callback_queue;
    nh_map.setCallbackQueue(&map_callback_queue);

    ros::Subscriber map_sub = nh_map.subscribe("/points_map", 10, map_callback);
    ros::Rate ros_rate(10);
    while (nh_map.ok()) {
        map_callback_queue.callAvailable(ros::WallDuration());
        ros_rate.sleep();
    }

    return nullptr;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ndt_matching");
    pthread_mutex_init(&mutex, NULL);

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set log file name.
    private_nh.getParam("output_log_data", _output_log_data);
    if (_output_log_data) {
        char buffer[80];
        std::time_t now = std::time(NULL);
        std::tm* pnow = std::localtime(&now);
        std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
        std::string directory_name = "/tmp/htc-robot/log/ndt_matching";
        filename = directory_name + "/" + std::string(buffer) + ".csv";
        boost::filesystem::create_directories(
            boost::filesystem::path(directory_name));
        ofs.open(filename.c_str(), std::ios::app);
    }

    // Geting parameters
    int method_type_tmp = 0;
    private_nh.param<bool>("enable_debug", is_debug, false);
    private_nh.getParam("method_type", method_type_tmp);
    _method_type = static_cast<MethodType>(method_type_tmp);
    private_nh.getParam("use_gnss", _use_gnss);
    private_nh.getParam("queue_size", _queue_size);
    private_nh.getParam("offset", _offset);
    private_nh.getParam("get_height", _get_height);
    private_nh.getParam("use_local_transform", _use_local_transform);
    private_nh.getParam("use_imu", _use_imu);
    private_nh.getParam("use_odom", _use_odom);
    private_nh.getParam("imu_upside_down", _imu_upside_down);
    private_nh.getParam("imu_topic", _imu_topic);
    private_nh.param<double>("gnss_reinit_fitness", _gnss_reinit_fitness, 500.0);
    private_nh.getParam("output_tf_frame_id", _output_tf_frame_id);
    double voxel_leaf_size;
    private_nh.param<double>("voxel_leaf_size", voxel_leaf_size, 1.5);
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    map_voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);

    std::string lidar_frame, lidar_topic;
    nh.param("lidar_frame", lidar_frame, std::string("lidar"));
    nh.param("lidar_topic", lidar_topic, std::string("/points_raw"));
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped tf_baselink2primarylidar;
    bool received_tf = true;

    // 1. Try getting base_link -> lidar TF from TF tree
    try {
        tf_baselink2primarylidar = tf_buffer.lookupTransform("base_link", lidar_frame, ros::Time().now(), ros::Duration(3.0));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Query base_link to primary lidar frame through TF tree failed: %s", ex.what()); 
        received_tf = false;
    }

    // 3. Try getting base_link -> lidar TF from tf_* params
    if (!received_tf) {
        float tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
        if (nh.getParam("tf_x", tf_x) && nh.getParam("tf_y", tf_y) &&
            nh.getParam("tf_z", tf_z) && nh.getParam("tf_roll", tf_roll) &&
            nh.getParam("tf_pitch", tf_pitch) &&
            nh.getParam("tf_yaw", tf_yaw)) {
            tf2::Vector3 tf_trans(tf_x, tf_y, tf_z);
            tf2::Quaternion tf_quat;
            tf_quat.setRPY(tf_roll, tf_pitch, tf_yaw);
            tf_baselink2primarylidar.transform.translation =
                tf2::toMsg(tf_trans);
            tf_baselink2primarylidar.transform.rotation = tf2::toMsg(tf_quat);

            received_tf = true;
        } else {
            ROS_WARN(
                "Query base_link to primary lidar frame through tf_* params "
                "failed");
        }
    }

    if (received_tf) {
        ROS_INFO("base_link to primary lidar transform queried successfully");
    } else {
        ROS_ERROR("Failed to query base_link to primary lidar transform");
        return 1;
    }

    tf_btol = tf2::transformToEigen(tf_baselink2primarylidar).matrix().cast<float>();

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Log file: " << filename << std::endl;
    std::cout << "method_type: " << static_cast<int>(_method_type) << std::endl;
    std::cout << "use_gnss: " << _use_gnss << std::endl;
    std::cout << "queue_size: " << _queue_size << std::endl;
    std::cout << "offset: " << _offset << std::endl;
    std::cout << "get_height: " << _get_height << std::endl;
    std::cout << "use_local_transform: " << _use_local_transform << std::endl;
    std::cout << "use_odom: " << _use_odom << std::endl;
    std::cout << "use_imu: " << _use_imu << std::endl;
    std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
    std::cout << "imu_topic: " << _imu_topic << std::endl;
    std::cout << "lidar_frame: " << lidar_frame << std::endl;
    std::cout << "gnss_reinit_fitness: " << _gnss_reinit_fitness << std::endl;
    std::cout << "tf_baselink2primarylidar: \n" << tf_btol << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    // Updated in initialpose_callback or gnss_callback
    initial_pose.x = 0.0;
    initial_pose.y = 0.0;
    initial_pose.z = 0.0;
    initial_pose.roll = 0.0;
    initial_pose.pitch = 0.0;
    initial_pose.yaw = 0.0;

    // Publishers
    predict_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/predict_pose", 10);
    predict_pose_imu_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_imu", 10);
    predict_pose_odom_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_odom", 10);
    predict_pose_imu_odom_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_imu_odom", 10);
    ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    ndt_pose_pub_lidarmode = nh.advertise<geometry_msgs::PoseStamped>("/ndt/current_pose", 10);
    // current_pose_pub =
    // nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    localizer_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/ndt/localizer_pose", 10);
    estimate_twist_pub =
        nh.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 10);
    estimated_vel_mps_pub =
        nh.advertise<std_msgs::Float32>("/estimated_vel_mps", 10);
    estimated_vel_kmph_pub =
        nh.advertise<std_msgs::Float32>("/estimated_vel_kmph", 10);
    estimated_vel_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("/estimated_vel", 10);
    time_ndt_matching_pub =
        nh.advertise<std_msgs::Float32>("/time_ndt_matching", 10);
    //   ndt_stat_pub = nh.advertise<autoware_msgs::NDTStat>("/ndt_stat", 10);
    ndt_reliability_pub =
        nh.advertise<std_msgs::Float32>("/ndt_reliability", 10);

    // Subscribers
    // ros::Subscriber param_sub = nh.subscribe("config/ndt", 10, param_callback);
    ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 10, gnss_callback);
     ros::Subscriber map_sub = nh.subscribe("points_map", 3, map_callback);
    ros::Subscriber initialpose_sub =
        nh.subscribe("initialpose", 10, initialpose_callback);
    ros::Subscriber points_sub =
        nh.subscribe(lidar_topic, _queue_size, points_callback);
    ros::Subscriber odom_sub =
        nh.subscribe("vehicle/odom", _queue_size * 10, odom_callback);
    ros::Subscriber imu_sub =
        nh.subscribe(_imu_topic.c_str(), _queue_size * 10, imu_callback);
    ros::Subscriber twist_sub =
        nh.subscribe("vehicle/twist", 10, vehicle_twist_callback);

    ros::spin();

    return 0;
}