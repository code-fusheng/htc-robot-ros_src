
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>

class pcd_map_manager
{

public:

    pcd_map_manager(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~pcd_map_manager();

    void run();

private:

    ros::NodeHandle nh, private_nh;

    ros::Subscriber current_pose_sub;
    ros::Subscriber inittial_pose_sub;

    ros::Publisher points_map_pub;
    ros::Publisher static_map_pub;

    struct Area
    {
        std::string filename;
        double x_min;
        double y_min;
        double z_min;
        double x_max;
        double y_max;
        double z_max;
        sensor_msgs::PointCloud2 points;
    };

    typedef std::vector<Area> AreaList;

    double UPDATE_INTERVAL = 1000; // ms
    double MARGIN = 100;		   // meter

    std::string AREALIST_FILENAME = "pcd_info.csv";

    std::string dir_static_map;
    std::string dir_dynamic_map;
    
    AreaList default_AreaList, cached_AreaList;

    uint8_t _FUNCTION_STATUS = 0;

    void init_params();

    void callback_current_pose(const geometry_msgs::PoseStamped &msg);
    void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped &msg);

    sensor_msgs::PointCloud2 create_pcd(const geometry_msgs::Point &p);
    AreaList create_wantedAreaList(const geometry_msgs::Point &p);
    bool is_in_area(double x, double y, const Area &area, double m);
    int is_in_cachedAreaList(Area wanted_area);
    void publish_pcd(sensor_msgs::PointCloud2 pcd);
    AreaList read_arealist(const std::string &path);
    bool publish_sparse_map();
};