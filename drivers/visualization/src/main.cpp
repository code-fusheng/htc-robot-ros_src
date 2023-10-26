#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <smartcar_msgs/Lane.h>
#include <smartcar_msgs/Waypoint.h>
#include <vector>
#include <cmath>

const double TH_MAX_POINTS = 10000;
const double TH_UPDATE_TRAJ = 0.3;  // m

namespace
{
double distance2points(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    return std::fabs(std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2)));
}

std::vector<double> quat2rpy(geometry_msgs::Quaternion orit){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(orit, quat);
    double r,p,y;
    tf::Matrix3x3(quat).getRPY(r,p,y);
    return std::vector<double>{r,p,y};
}

// geometry_msgs::Quaternion rpy2quat(double r, double p, double y){
//     tf::Quaternion t = tf::createQuaternionFromRPY(r,p,y);
// }

class Visualization
{
private:
    ros::NodeHandle nh_, pnh_;

    // current pose
    bool show_model_car_;
    bool show_history_path_;
    bool show_followed_path;
    bool show_default_pathes;
    std::string data_base;

    nav_msgs::Path history_path;
    ros::Subscriber sub_ndt_current_pose_, sub_gps_current_pose_;
    ros::Subscriber sub_scrino_info, sub_global_path;
    ros::Subscriber sub_best_local_trajectory;  // 基于采样的open planner局部路径规划算法输出

    ros::Publisher pub_global_followed_path, pub_default_pathes;
    ros::Publisher pub_model_current_pose_;
    ros::Publisher pub_path;
    ros::Publisher pub_local_followed_path_1;
    geometry_msgs::PoseStamped previous_added_pose;

    visualization_msgs::Marker _car_model;

    visualization_msgs::Marker _maker_global_followed_path, _maker_local_followed_path;
    visualization_msgs::MarkerArray _marker_array_patches;

    // params
    double dist_interval = 0.5;

    void _init(){
        // init car model
        _car_model.header.frame_id = "/map";
        _car_model.ns = "myns";
        _car_model.id = 1;
        _car_model.type = visualization_msgs::Marker::MESH_RESOURCE;
        _car_model.action = visualization_msgs::Marker::ADD;
        _car_model.mesh_resource = "package://visualization/models/car/poly_911.dae";
        _car_model.mesh_use_embedded_materials = true;
        _car_model.scale.x = 1;
        _car_model.scale.y = 1;
        _car_model.scale.z = 1;
        // _car_model.color.b = 0.0f;
        // _car_model.color.g = 0.0f;
        // _car_model.color.r = 1.0;
        _car_model.color.a = 1.0;
        _car_model.lifetime = ros::Duration();
    }

    void callbackFromNdtPose(const geometry_msgs::PoseStamped &ndt_pose){
        handle_pose(ndt_pose);
    }

    void callbackGlobalPath(const smartcar_msgs::LaneConstPtr& msg_global_path) {
        this->_maker_global_followed_path.points.clear();
        for (const auto& p: msg_global_path->waypoints) {
            if (this->_maker_global_followed_path.points.size() > 0 && distance2points(p.pose.pose.position, this->_maker_global_followed_path.points.back()) < this->dist_interval) {
                continue;
            }
            this->_maker_global_followed_path.points.push_back(p.pose.pose.position);
            // this->_maker_global_followed_path.points.back().z = 0;  // 清除z轴数据
        }
        this->pub_global_followed_path.publish(this->_maker_global_followed_path);
    }

    void callbackBestLocalTrajectory(const smartcar_msgs::LaneConstPtr& msg_local_path) {
        this->_maker_local_followed_path.points.clear();
        for (const auto& p: msg_local_path->waypoints) {
            if (this->_maker_local_followed_path.points.size() > 0 && distance2points(p.pose.pose.position, this->_maker_local_followed_path.points.back()) < this->dist_interval) {
                continue;
            }
            this->_maker_local_followed_path.points.push_back(p.pose.pose.position);
            // this->_maker_local_followed_path.points.back().z = 0;  // 清除z轴数据
        }
        this->pub_local_followed_path_1.publish(this->_maker_local_followed_path);
    }

    void handle_pose(const geometry_msgs::PoseStamped &current_pose){
        if (show_model_car_){
            _car_model.header.stamp = ros::Time::now();
            _car_model.pose.position.x = current_pose.pose.position.x;
            _car_model.pose.position.y = current_pose.pose.position.y;
            _car_model.pose.position.z = 0;
            // _car_model.pose.orientation.x = current_pose.pose.orientation.x;
            // _car_model.pose.orientation.y = current_pose.pose.orientation.y;
            // _car_model.pose.orientation.z = current_pose.pose.orientation.z;
            // _car_model.pose.orientation.w = current_pose.pose.orientation.w;

            tf::Quaternion quat;
            tf::quaternionMsgToTF(current_pose.pose.orientation, quat);

            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            yaw += 3.141692654 / 2.0;

            quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
            _car_model.pose.orientation.x = quat.getX();
            _car_model.pose.orientation.y = quat.getY();
            _car_model.pose.orientation.z = quat.getZ();
            _car_model.pose.orientation.w = quat.getW();
            pub_model_current_pose_.publish(_car_model);
        }

        if (show_history_path_){
            if (history_path.poses.size() > TH_MAX_POINTS){
                int size = history_path.poses.size();
                auto first = history_path.poses.begin();
                history_path.poses.assign(first + int(size / 2), history_path.poses.end());
            }

            if(std::sqrt(std::pow(current_pose.pose.position.x -previous_added_pose.pose.position.x, 2) + std::pow(current_pose.pose.position.y - previous_added_pose.pose.position.y, 2)) > TH_UPDATE_TRAJ){
                history_path.poses.push_back(current_pose);
                pub_path.publish(history_path);
                previous_added_pose = current_pose;
            }
        }
    }

    // void history_path_cb(const geometry_msgs::PoseStamped &current_pose)
    // {
    //     geometry_msgs::PoseStamped p = current_pose;

    //     history_path.poses.push_back(p);
    //     if (history_path.poses.size() > 10000 || distance2points(history_path.poses.begin()->pose.position, (history_path.poses.end() - 1)->pose.position) > 100.0) // >50m
    //     {
    //         int total_length = history_path.poses.size() / 2;
    //         auto first = history_path.poses.begin();
    //         auto last = first + total_length;
    //         history_path.poses.erase(first, last);
    //     }
    //     pub_history_path_.publish(history_path);
    // }

public:
    Visualization() : nh_(""), pnh_("~"){};
    ~Visualization(){};

    void run(){
        _init();
        pnh_.param<bool>("show_model_car", show_model_car_, false);
        pnh_.param<bool>("show_history_path", show_history_path_, true);
        pnh_.param<bool>("show_followed_path", show_followed_path, true);
        pnh_.param<bool>("show_default_pathes", show_default_pathes, true);
        pnh_.param<double>("dist_interval", dist_interval, 0.5);
        nh_.param<std::string>("data_base", data_base, "/home/data");

        history_path.header.frame_id = "map";

        // init default path marker
        this->_maker_global_followed_path.header.frame_id = "map";
        this->_maker_global_followed_path.header.stamp = ros::Time::now();
        this->_maker_global_followed_path.id = 2;
        this->_maker_global_followed_path.action = visualization_msgs::Marker::ADD;
        this->_maker_global_followed_path.type = visualization_msgs::Marker::LINE_STRIP;
        this->_maker_global_followed_path.pose.orientation.w = 1;
        this->_maker_global_followed_path.scale.x = 1.0;  // 路径的宽度
        this->_maker_global_followed_path.color.a = 0.7;
        this->_maker_global_followed_path.color.g = 1;

        // init local followed path
        this->_maker_local_followed_path.header.frame_id = "map";
        this->_maker_local_followed_path.header.stamp = ros::Time::now();
        this->_maker_local_followed_path.id = 3;
        this->_maker_local_followed_path.action = visualization_msgs::Marker::ADD;
        this->_maker_local_followed_path.type = visualization_msgs::Marker::LINE_STRIP;
        this->_maker_local_followed_path.pose.orientation.w = 1;
        this->_maker_local_followed_path.scale.x = 0.2;  // 路径的宽度
        this->_maker_local_followed_path.color.a = 0.7;
        this->_maker_local_followed_path.color.r = 255;
        this->_maker_local_followed_path.color.g = 165;
        this->_maker_local_followed_path.color.b = 0;

        pub_model_current_pose_ = nh_.advertise<visualization_msgs::Marker>("/model/car", 10);
        pub_path = nh_.advertise<nav_msgs::Path>("/trajectory/vehicle", 100);
        pub_default_pathes = nh_.advertise<visualization_msgs::MarkerArray>("/trajectory/default_pathes", 10);
        pub_global_followed_path = nh_.advertise<visualization_msgs::Marker>("/trajectory/global_path", 10);
        pub_local_followed_path_1 = nh_.advertise<visualization_msgs::Marker>("/trajectory/local_path", 10);

        sub_ndt_current_pose_ = nh_.subscribe("/current_pose", 1, &Visualization::callbackFromNdtPose, this);
        sub_global_path = nh_.subscribe("/global_path", 1, &Visualization::callbackGlobalPath, this);
        sub_best_local_trajectory = nh_.subscribe("best_local_trajectories", 1, &Visualization::callbackBestLocalTrajectory, this);
        // sub_scrino_info = 
    }
};
}; // namespace

int main(int argc, char **argv){
    ros::init(argc, argv, "visualization_node");
    Visualization app;
    app.run();
    ros::spin();
    return 0;
}