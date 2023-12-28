#include "laserscan_combine_detect/laserscan_combine_detect.h"
#include <limits>

namespace LaserScanCombineDetector {

//升序排列
bool compare(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    return p1.x < p2.x;
}

LaserCombineDetectorApp::LaserCombineDetectorApp():nh_private("~")
{
}

LaserCombineDetectorApp::~LaserCombineDetectorApp()
{
}

void LaserCombineDetectorApp::init_ros() {
    nh_private.param<int>("rate", rate, 10);
    nh_private.param<std::string>("scan_left_topic", scan_left_topic, "/scan_left");
    nh_private.param<std::string>("scan_left_frame", scan_left_frame, "laser_left");
    nh_private.param<std::string>("scan_right_topic", scan_right_topic, "/scan_right");
    nh_private.param<std::string>("scan_right_frame", scan_right_frame, "laser_right");
    nh_private.param<double>("vehicle_width", vehicle_width, 0.8);
    nh_private.param<double>("dist_front_to_base", dist_front_to_base, 1.0);
    nh_private.param<double>("hor_detect_dist", hor_detect_dist, 0.2);
    nh_private.param<double>("ver_detect_dist", ver_detect_dist, 6.0);
    nh_private.param<int>("obs_points_num", obs_points_num, 5);

    y_min = -(vehicle_width/2+hor_detect_dist);
    y_max = vehicle_width/2+hor_detect_dist;
    x_min = dist_front_to_base;
    x_max = dist_front_to_base + ver_detect_dist;

    // add default frame_id to make tf lookup happy
    _scan_left.header.frame_id = scan_left_frame;
    _scan_right.header.frame_id = scan_right_frame;

    // publisher
    pub_obs_info = nh.advertise<automotive_msgs::SimpleObstacleDist>("/detection/laser_combined_detect/simple", 2);

    // subscriber
    sub_laser_left  = nh.subscribe(scan_left_topic,   2, &LaserCombineDetectorApp::cb_laser_left_scan,  this);
    sub_laser_right = nh.subscribe(scan_right_topic,  2, &LaserCombineDetectorApp::cb_laser_right_scan, this);
}

void LaserCombineDetectorApp::run()
{
    init_ros();
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        this->_calc_once();
        
	ros::spinOnce();
        loop_rate.sleep();
    }
}

/*
 *  ROS Callback functions
*/
void LaserCombineDetectorApp::cb_laser_left_scan(const sensor_msgs::LaserScanConstPtr &msg)
{
    this->_laser_left_lock.lock();	
    this->_scan_left = *msg;
    this->_laser_left_lock.unlock();
}

void LaserCombineDetectorApp::cb_laser_right_scan(const sensor_msgs::LaserScanConstPtr &msg)
{
    this->_laser_right_lock.lock();	
    this->_scan_right = *msg;
    this->_laser_right_lock.unlock();
}

/*
 *  Helper functions
*/
void LaserCombineDetectorApp::_reset()
{   
    return;
}

// geometry_msgs::Point LaserCombineDetectorApp::_transform_point(const geometry_msgs::Point& in_point, const tf::StampedTransform& in_transform)
// {
//     return this->_transform_point(in_point.x, in_point.y, in_point.z, in_transform);
// }

pcl::PointXYZ LaserCombineDetectorApp::_transform_point(float x, float y, float z, const tf::StampedTransform& in_transform)
{
    tf::Vector3 tf_point(x, y, z);
    tf::Vector3 tf_point_transformed = in_transform * tf_point;
    pcl::PointXYZ p;
    p.x = tf_point_transformed.x();
    p.y = tf_point_transformed.y();
    p.z = tf_point_transformed.z();
    return p;
}

float LaserCombineDetectorApp::_dist_of_two_point(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    return this->__dist_of_two_point(p1.x, p1.y, p2.x, p2.y);
}

float LaserCombineDetectorApp::_dist_of_two_point(const geometry_msgs::Point &p1, const pcl::PointXYZ &p2)
{
    return this->__dist_of_two_point(p1.x, p1.y, p2.x, p2.y);
}

float LaserCombineDetectorApp::__dist_of_two_point(float x1, float y1, float x2, float y2)
{
    return std::sqrt(std::pow(x2-x1, 2) + std::pow(y2-y1, 2));
}

void LaserCombineDetectorApp::get_roi_points(std::vector<pcl::PointXYZ> &roi_points, pcl::PointCloud<pcl::PointXYZ> &raw_points)
{
    for (int i = 0; i < raw_points.points.size(); i++) {
        if (this->x_min < raw_points.points[i].x && raw_points.points[i].x < this->x_max
            && this->y_min < raw_points.points[i].y && raw_points.points[i].y < this->y_max) {
            // ROS_INFO("push back x: %.2f, y: %.2f, range  %.2f, %.2f, %.2f, %.2f", rawCloud.points[i].x, rawCloud.points[i].y, laser_x_min, laser_x_max, laser_y_min, laser_y_max);
            roi_points.push_back(raw_points.points[i]);
	    }
    }
}

void LaserCombineDetectorApp::transform_scan_to_base_link(pcl::PointCloud<pcl::PointXYZ> &output, std::string scan_frame, const pcl::PointCloud<pcl::PointXYZ> &scan_cloud)
{
    // 查找scan到map的tf变换
    tf::StampedTransform _cur_transform;
    bool scan_tf_get = false;
    static ros::Time pre_find_time = ros::Time::now();
    try {
        // ros::Time(0)指定了时间为0，即获得最新有效的变换。
        // 改变获取当前时间的变换，即改为ros::Time::now(),不过now的话因为监听器有缓存区的原因。一般会出错
        // 参考：https://www.ncnynl.com/archives/201702/1313.html
        transform_listener.lookupTransform("base_link", scan_frame, ros::Time(0), _cur_transform);
        scan_tf_get = true;
    } catch (tf::TransformException ex) {
        ros::Time t = ros::Time::now();
        if ( (t-pre_find_time).toSec() > 1.0 ) {
            ROS_INFO("[laser_detector] : find base_link->%s tf failed", scan_frame);
            pre_find_time = t;
        }
        
        return;  // 无法找到tf
    }

    // 转换roi cloud到和轨迹统一坐标系下
    for (int i = 0; i < scan_cloud.points.size(); i++)
    {
        pcl::PointXYZ p = this->_transform_point(scan_cloud.points[i].x, scan_cloud.points[i].y, scan_cloud.points[i].z, _cur_transform);
        // ROS_INFO("x: %.2f -> %.2f, y: %.2f -> %.2f", roiCloud.points[i].x, p.x, roiCloud.points[i].y, p.y);
	    output.push_back(p);
    }    
}


int LaserCombineDetectorApp::_calc_once()
{
    // laser scan转换到车身坐标系
    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    // // left scan
    sensor_msgs::PointCloud2 scan_left_cloud;
    this->_laser_left_lock.lock();
    this->projector_.transformLaserScanToPointCloud(scan_left_frame, this->_scan_left, scan_left_cloud, tfListener_);
    this->_laser_left_lock.unlock();

    pcl::PointCloud<pcl::PointXYZ> scan_left_raw;
    pcl::fromROSMsg(scan_left_cloud, scan_left_raw);

    this->transform_scan_to_base_link(raw_cloud, scan_left_frame, scan_left_raw);

    // // right scan
    sensor_msgs::PointCloud2 scan_right_cloud;
    this->_laser_right_lock.lock();
    this->projector_.transformLaserScanToPointCloud(scan_right_frame, this->_scan_right, scan_right_cloud, tfListener_);
    this->_laser_right_lock.unlock();

    pcl::PointCloud<pcl::PointXYZ> scan_right_raw;
    pcl::fromROSMsg(scan_right_cloud, scan_right_raw);

    this->transform_scan_to_base_link(raw_cloud, scan_right_frame, scan_right_raw);
    
    // 获取点云ROI
    std::vector<pcl::PointXYZ> roi_points;
    this->get_roi_points(roi_points, raw_cloud);
    // ROS_INFO("ROI cloud size = %d", roiCloud.points.size());

    automotive_msgs::SimpleObstacleDist _obs_info;
    if (roi_points.size() < obs_points_num) {
        // 点云过少, 认为没有障碍物
        _obs_info.distance = 1000.0;
    }else{
        // 排序并计算障碍物的最近距离
        std::sort(roi_points.begin(), roi_points.end(), compare);
        for (int i = 0; i < obs_points_num; i++)
            _obs_info.distance += roi_points[i].x;
        
        _obs_info.distance /= obs_points_num;
        _obs_info.distance -= dist_front_to_base;
    }

    pub_obs_info.publish(_obs_info);

    return 0;
}

}  // end namespace LaserDetector
