#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

// ROS includes
#include <can_msgs/ecu.h>
#include <can_msgs/vehicle_status.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/service.h>
#include <queue>
#include <string>

#include "smartcar_msgs/ControlCommandStamped.h"
#include "smartcar_msgs/Lane.h"
#include "smartcar_msgs/LaneArray.h"
#include <automotive_msgs/PPTSet.h>
#include <automotive_msgs/AvoidSwitch.h>

namespace waypoint_follower {
class PurePursuitNode {
   private:
    // handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // publisher
    ros::Publisher pub_ctl, pub_target;
    ros::Publisher pub_path;
    ros::Publisher pub_control;
    ros::Publisher pub_pure_pursuit_finished;
    ros::Publisher pub_reversed_current_pose;
    ros::Publisher pub_closet_index;

    // subscriber
    ros::Subscriber sub_currentpose, sub_lane, sub_speed;
    ros::Subscriber sub_safe_waypoints;  // For A* 局部路径规划
    ros::Subscriber sub_best_local_trajectory;  // For 基于采样的open planner局部路径规划

    ros::ServiceServer ppt_service, avoid_service;

    // constant
    const int LOOP_RATE_;  // processing frequency
    const double curvature_MIN_;
    double const_velocity_;
    double const_lookahead_distance_;  // meter

    // variables
    bool is_linear_interpolation_;
    bool is_waypoint_set_;
    bool is_pose_set_;
    bool is_const_lookahead_dis_;
    bool is_const_speed_command_;
    double command_linear_velocity_;
    int next_waypoint_number_;
    int end_waypoint_index;
    // bool use_point_speed_limit;  // 使用点上的速度限制
    geometry_msgs::Point next_target_position_;

    geometry_msgs::Pose current_pose_;

    double cur_speed = 0.5;
    ros::Time last_cpose_update_time = ros::Time::now();

    std::vector<smartcar_msgs::Waypoint> current_waypoints_;
    double wheel_base_;
    double lookahead_distance_;
    double lookahead_distance_ratio_;  // lookahead_distance_ = cur_speed * lookahead_distance_ratio_, longer lookahead_distance_ = more stable
    double minimum_lookahead_distance_;

    bool is_last_point;

    bool is_in_cross;
    bool is_not_lane;
    double cross_lookahead_dis;
    double lane_lookahead_dis;

    int pre_index;
    int search_start_index;
    int clearest_points_index;

    float search_radius;
    bool cross_in;
    bool cross_out;
    bool almost_reach;
    int lock_index;
    int save_index;
    int current_pose_closet_index;

    double lane_speed_limit;
    double cross_speed_limit;

    double pre_pre_steer = 0;
    double pre_steer = 0;

    std::string current_pose_type = "null";
    std::string automotive_mode = "";
    std::string input_current_pose_topic = "";
    double stop_distance = 1.0;
    double step_size = 0.1;

    bool is_target_front;

    int current_avoid_type = automotive_msgs::AvoidSwitch::Response::AVOID_STOP;

   public:
    PurePursuitNode();

    ~PurePursuitNode();

    void run();

    // callbacks
    void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);

    void callbackFromWayPoints(const smartcar_msgs::Lane &msg);
    void callbackFromSafeWayPoints(const nav_msgs::PathConstPtr &msg);
    void callbackFromBestLocalTrajectory(const smartcar_msgs::Lane &msg);

    void callbackFromCurrentVelocity(const can_msgs::vehicle_status &msg);

    bool serviceSetTarget(automotive_msgs::PPTSet::Request &req, automotive_msgs::PPTSet::Response &res);
    bool on_service_avoid_type(automotive_msgs::AvoidSwitch::Request &req, automotive_msgs::AvoidSwitch::Response &res);

    bool setPPTTarget(smartcar_msgs::Lane lane);

    void __handle_followed_path(smartcar_msgs::Lane msg);

    // initializer
    void initForROS();

    void reset();

    // functions

    void visualInRviz();

    bool computeCurvature(double *output_curvature);

    double calcCurvature(geometry_msgs::Point target);

    bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target);

    void getNextWaypoint();

    void publishControlCommandStamped(const bool &can_get_curvature, const double &curvature);

    void publishBrakeCommand();

    double computeLookaheadDistance() const;

    double computeCommandVelocity();

    geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose);

    double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2);

    double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c);

    tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree);

    tf::Vector3 point2vector(geometry_msgs::Point point);

    bool do_ending_process(can_msgs::ecu msg_ecu);

    bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c);

    double deg2rad(double deg) { return deg * M_PI / 180; }

};

}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_H
