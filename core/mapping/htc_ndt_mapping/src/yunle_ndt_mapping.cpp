#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

#include <ndt_cpu/NormalDistributionsTransform.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

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

static pcl::PointCloud<pcl::PointXYZI> global_transformed_scan;
static bool flag_update_map = false;

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad) {
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
        diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
        diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
}

static inline double get_absolute_angle_diff(const double &current_yaw, const double &added_yaw) {
    double diff = (current_yaw - added_yaw) * 180.0 / PI;
    diff = diff + 360*4;
    int res = int(diff) % 360;
    if ( res > 180 ) res = 360 -res;
    return res;
}

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {
    // if (!is_on_mapping) return;
    // static bool _is_running = false;
    // if ( !_is_running ) {
    //     _is_running = true;
    // }
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
        if (min_scan_range < r && r < max_scan_range) {
            scan.push_back(p);
        }
    }
    // scan保存的是去除最近最远点后的原始点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

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

    ros::Time start_set_ndt_target = ros::Time::now();
    if (is_first_map == true) {
        if (_method_type == MethodType::PCL_GENERIC)
            ndt.setInputTarget(map_ptr);
        else if (_method_type == MethodType::PCL_ANH)
            anh_ndt.setInputTarget(map_ptr);
        is_first_map = false;
        // _is_running = false;
        return;
    }
            
    ndt_set_target_time = (ros::Time::now() - start_set_ndt_target).toSec();

    guess_pose.x = previous_pose.x + diff_x;
    guess_pose.y = previous_pose.y + diff_y;
    guess_pose.z = previous_pose.z + diff_z;
    guess_pose.roll = previous_pose.roll;
    guess_pose.pitch = previous_pose.pitch;
    guess_pose.yaw = previous_pose.yaw + diff_yaw;

    pose guess_pose_for_ndt;
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

    ndt_align_time = (ros::Time::now() - start_ndt_align).toSec();
    t_base_link = t_localizer * tf_ltob;
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

    double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) +
                        pow(current_pose.y - added_pose.y, 2.0));
    double _diff_angle = get_absolute_angle_diff(current_pose.yaw, added_pose.yaw) ; 
    //  || _diff_angle > 7
    if (shift >= min_add_scan_shift)  // 更新地图并发布
    {
        global_transformed_scan = *transformed_scan_ptr;
        flag_update_map = true;
        static ros::Time update_st = ros::Time::now();
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

        if (_method_type == MethodType::PCL_GENERIC)
            new_ndt.setInputTarget(map_ptr);
        else if (_method_type == MethodType::PCL_ANH) {
            if (_incremental_voxel_update == true)
                new_anh_ndt.updateVoxelGrid(input_source);
            else
                new_anh_ndt.setInputTarget(map_ptr);
        }  
        if (_method_type == MethodType::PCL_GENERIC)
            ndt = new_ndt;
        else if (_method_type == MethodType::PCL_ANH) {
            anh_ndt = new_anh_ndt;
        }
        // 发布更新后的地图
        sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(map, *map_msg_ptr);
        ndt_map_pub.publish(*map_msg_ptr);
        flag_update_map = false;
        static ros::Time update_end = ros::Time::now();
        ROS_INFO("[ndt_mapping] update map used %.2f seconds", (update_end - update_st).toSec());
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

    // _is_running = false;

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Sequence number: " << input->header.seq << std::endl;
    std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
    std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
    std::cout << "map: " << map.points.size() << " points." << std::endl;
    std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
    std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
    std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
                << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t_localizer << std::endl;
    std::cout << "shift: " << shift << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

}

int main(int argc, char **argv) 
{

    ros::init(argc, argv, "yunle_ndt_mapping");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set log file name.
    char buffer[80];
    std::time_t now = std::time(NULL);
    std::tm *pnow = std::localtime(&now);
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
    filename = "ndt_mapping_" + std::string(buffer) + ".csv";
    ofs.open(filename.c_str(), std::ios::app);

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

    map.header.frame_id = "map";
    history_trajectory.header.frame_id = "map";

    ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 10);
    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    history_trajectory_pub = nh.advertise<nav_msgs::Path>("/ndt/history_trajectory", 10);

    ros::Subscriber points_sub = nh.subscribe("points_raw", 10, points_callback);

    ros::spin();

    return 0;

}