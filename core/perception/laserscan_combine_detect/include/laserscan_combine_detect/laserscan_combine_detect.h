#ifndef LASERSCAN_COMBINE_DETECTOR_H
#define LASERSCAN_COMBINE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <automotive_msgs/SimpleObstacleDist.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <vector>
#include <mutex>

namespace LaserScanCombineDetector {

class LaserCombineDetectorApp{
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    // subscriber
    ros::Subscriber sub_laser_left;
    ros::Subscriber sub_laser_right;

    // publisher
    ros::Publisher pub_obs_info;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    sensor_msgs::LaserScan _scan_left, _scan_right;

    std::string scan_left_topic = "/scan_left";
    std::string scan_left_frame = "laser_left";
    std::string scan_right_topic = "/scan_right";
    std::string scan_right_frame = "laser_right";
    tf::TransformListener transform_listener;

    int rate;  // 计算和发布频率

    double vehicle_width, dist_front_to_base;

    double hor_detect_dist, ver_detect_dist;  // 用于提取ROI区域

    int obs_points_num;  // 多少个点会被认为是障碍物

    double y_min, y_max, x_min, x_max;

    std::mutex _laser_left_lock, _laser_right_lock;
    std::mutex _calc_lock;

    // ------ ros callback ------

    void cb_laser_left_scan(const sensor_msgs::LaserScanConstPtr &msg);

    void cb_laser_right_scan(const sensor_msgs::LaserScanConstPtr &msg);

    // ------ Helper functions ------
    void init_ros();

    // geometry_msgs::Point _transform_point(const geometry_msgs::Point& in_point, const tf::StampedTransform& in_transform);

    pcl::PointXYZ _transform_point(float x, float y, float z, const tf::StampedTransform& in_transform);

    float _dist_of_two_point(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

    float _dist_of_two_point(const geometry_msgs::Point &p1, const pcl::PointXYZ &p2);

    float __dist_of_two_point(float x1, float y1, float x2, float y2);

    void get_roi_points(std::vector<pcl::PointXYZ> &roi_points, pcl::PointCloud<pcl::PointXYZ> &raw_points);

    void transform_scan_to_base_link(pcl::PointCloud<pcl::PointXYZ> &output, std::string scan_frame, const pcl::PointCloud<pcl::PointXYZ> &scan_cloud);

    int _calc_once();  // 执行一次计算

    void _reset();

public:
    LaserCombineDetectorApp();
    ~LaserCombineDetectorApp();

    void run();


};  // end class LaserCombineDetectorApp

};  // end name space LaserScanCombineDetector



#endif  // end LASERSCAN_COMBINE_DETECTOR_H
