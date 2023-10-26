#include "pure_persuit.h"

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <smartcar_msgs/Waypoint.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/tf.h>
#include <cmath>
#include <std_msgs/Int32.h>
#include <automotive_msgs/PPTWayPointStatus.h>

namespace waypoint_follower {
// Constructor
PurePursuitNode::PurePursuitNode()
    : private_nh_("~"),
      LOOP_RATE_(25),
      curvature_MIN_(1 / 9e10),
      is_waypoint_set_(false),
      is_pose_set_(false),
      command_linear_velocity_(0),
      next_waypoint_number_(-1),
      end_waypoint_index(-1),
      lookahead_distance_(0),
      is_last_point(false),
      is_in_cross(false),
      is_not_lane(true),
      pre_index(0),
      search_start_index(0),
      clearest_points_index(-1),
      search_radius(2),
      cross_in(false),
      cross_out(false),
      almost_reach(false),
      lock_index(-1),
      save_index(-1),
      current_pose_closet_index(0),
      is_target_front(true) {
    // 设置了订阅，发布的话题消息， 读取了launch文件中的配置变量
    initForROS();
}

// Destructor
PurePursuitNode::~PurePursuitNode() {}

void PurePursuitNode::initForROS() {
    // ros parameter settings
    private_nh_.param("is_linear_interpolation", is_linear_interpolation_, bool(false));
    private_nh_.param("wheel_base", wheel_base_, double(0.5));

    private_nh_.param("lookahead_distance_ratio", lookahead_distance_ratio_, double(6.0));  //假设速度的单位是m/s, 1M/S的预瞄距离是4m
    private_nh_.param("minimum_lookahead_distance", minimum_lookahead_distance_, double(2));
    private_nh_.param("const_lookahead_distance", const_lookahead_distance_, double(5));
    private_nh_.param("is_const_lookahead_dis", is_const_lookahead_dis_, true);
    private_nh_.param("is_const_speed_command", is_const_speed_command_, false);

    private_nh_.param("cross_lookahead_dis", cross_lookahead_dis, double(2));
    private_nh_.param("lane_lookahead_dis", lane_lookahead_dis, double(9));

    private_nh_.param("const_velocity", const_velocity_, double(5.4));  // km/h
    private_nh_.param("lane_speed_limit", lane_speed_limit, double(3.6));
    private_nh_.param("cross_speed_limit", cross_speed_limit, double(3.6 * 0.5));
    private_nh_.param("stop_distance", stop_distance, double(0.5));
    nh_.param("automotive_mode", automotive_mode, std::string(""));

    input_current_pose_topic = "/current_pose";

    // setup publisher
    // pub_ctl = nh_.advertise<geometry_msgs::Twist>("/ecu_raw", 10);
    pub_target = nh_.advertise<visualization_msgs::MarkerArray>("/pure_pursuit/target_waypoint", 10);
    pub_path = nh_.advertise<nav_msgs::Path>("/pure_pursuit/followed_path", 10);
    pub_control = nh_.advertise<can_msgs::ecu>("/pure_pursuit/ecu", 5);
    pub_pure_pursuit_finished = nh_.advertise<std_msgs::Bool>("/pure_pursuit/task_finished", 5);
    pub_reversed_current_pose = nh_.advertise<geometry_msgs::PoseStamped>("/ppt/current_pose_reversed", 5);
    pub_closet_index = nh_.advertise<automotive_msgs::PPTWayPointStatus>("/ppt/waypoints_status", 3);

    ppt_service = nh_.advertiseService("/set_ppt_target", &PurePursuitNode::serviceSetTarget, this);
    avoid_service = nh_.advertiseService("/avoid_type", &PurePursuitNode::on_service_avoid_type, this);

    sub_lane = nh_.subscribe("/global_path", 10, &PurePursuitNode::callbackFromWayPoints, this);
    sub_currentpose = nh_.subscribe(input_current_pose_topic, 10, &PurePursuitNode::callbackFromCurrentPose, this);
    sub_speed = nh_.subscribe("/vehicle_status", 10, &PurePursuitNode::callbackFromCurrentVelocity, this);
}

void PurePursuitNode::run() {
    // ROS_INFO_STREAM("pure pursuit start");
    ros::Rate loop_rate(LOOP_RATE_);

    while (ros::ok()) {
        ros::spinOnce();
        if (!is_pose_set_ || !is_waypoint_set_) {
            static ros::Time last_record = ros::Time::now();
            if ((ros::Time::now() - last_record).toSec() > 1.0) {
                ROS_WARN("[waypoint_follower]Necessary topics are not subscribed yet, is_pose_set_ : %d, is_waypoint_set_ : %d", is_pose_set_,
                         is_waypoint_set_);
                last_record = ros::Time::now();
            }
            publishBrakeCommand();  // if there is no task (no pose set or no global path set), send brake command
        } else {
            if ((ros::Time::now() - last_cpose_update_time).toSec() > 0.5) {
                // ROS_WARN("[waypoint_follower] long time after last current pose update");
                // loop_rate.sleep();
                continue;
            }
            // ROS_INFO("[waypoint_follwer] do follower");
            lookahead_distance_ = computeLookaheadDistance();
            double curvature = 0;
            bool can_get_curvature = computeCurvature(&curvature);
            publishControlCommandStamped(can_get_curvature, curvature);

            // show closet point and target point
            visualInRviz();

            // is_pose_set_ = false;
        }
        // ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_WARN("[waypoint follower] terminated.");
}

void PurePursuitNode::reset() {
    this->is_waypoint_set_ = false;
    this->is_target_front = true;
}

bool PurePursuitNode::serviceSetTarget(automotive_msgs::PPTSet::Request &req, automotive_msgs::PPTSet::Response &res){
    ROS_INFO("[waypoint follower] Received service call of setting followed path, points=%d", req.waypoints.size());
    this->reset();
    if (req.set_target == true) {
        this->is_target_front = req.is_target_front;
        if (this->is_target_front) ROS_INFO("[Waypoint follower] Will go front");
        else ROS_INFO("[Waypoint follower] Will go back");
        smartcar_msgs::Lane lane;

        for (auto p: req.waypoints) {
            smartcar_msgs::Waypoint w;
            w.is_lane = 1;
            w.pose = p;
            lane.waypoints.push_back(w);
        }

        // if (lane.waypoints.size() == req.speed_limit_list.size()) {
        //     for (int i = 0; i < req.speed_limit_list.size(); i++) {
        //         lane.waypoints[i].speed_limit = req.speed_limit_list[i];  // m/s
        //     }
        // }
        // this->setPPTTarget(lane);
        this->callbackFromWayPoints(lane);
    }

    res.success = true;
    return true;
}

bool PurePursuitNode::on_service_avoid_type(automotive_msgs::AvoidSwitch::Request &req, automotive_msgs::AvoidSwitch::Response &res) {
    if ( !req.is_set ) {
        res.current_avoid_type = current_avoid_type;
        return true;
    }
    if (req.set_avoid_type == current_avoid_type) {
        res.current_avoid_type = current_avoid_type;
        return true;
    }

    current_avoid_type = req.set_avoid_type;
    if (current_avoid_type == res.AVOID_AUTO) {
        ROS_INFO("[PurePursuit] Set avoid AUTO");
        // 开启主动避障, 将订阅局部路径规划的输出
        sub_safe_waypoints = nh_.subscribe("safety_waypoints", 10, &PurePursuitNode::callbackFromSafeWayPoints, this);
        sub_best_local_trajectory = nh_.subscribe("best_local_trajectories", 10, &PurePursuitNode::callbackFromBestLocalTrajectory, this);
    }else if (current_avoid_type == res.AVOID_STOP) {
        ROS_INFO("[PurePursuit] Set avoid STOP");
        // 关闭主动避障, 只按照global_path的路径行驶
        sub_safe_waypoints.shutdown();
        sub_best_local_trajectory.shutdown();
    }
    res.current_avoid_type = current_avoid_type;
    return true;
}

bool PurePursuitNode::setPPTTarget(smartcar_msgs::Lane lane){
    // NOTE:: This function is a copy of callbackFromWayPoints
    ROS_INFO("[waypoint_follower] received global path");

    // interplot waypoints
    current_waypoints_.clear();
    current_waypoints_ = lane.waypoints;
    // for (int i = 0; i < lane.waypoints.size() - 1; i++) {
    //     smartcar_msgs::Waypoint p = lane.waypoints[i];
    //     current_waypoints_.push_back(p);
    //     double d = getPlaneDistance(p.pose.pose.position, lane.waypoints[i+1].pose.pose.position);
    //     if ( d > 0.05) {
    //         int steps = int(d/0.05);
    //         double diff_x = (lane.waypoints[i+1].pose.pose.position.x - p.pose.pose.position.x)/steps;
    //         double diff_y = (lane.waypoints[i+1].pose.pose.position.y - p.pose.pose.position.y)/steps;
    //         for (int j = 1; j < steps-1; j++) {
    //             smartcar_msgs::Waypoint t = lane.waypoints[i];
    //             t.pose.pose.position.x = p.pose.pose.position.x + j * diff_x;
    //             t.pose.pose.position.y = p.pose.pose.position.y + j * diff_y;
    //             current_waypoints_.push_back(t);
    //         }
    //     }
    // }
    // ROS_INFO("Previous size=%d, interplot size = %d", msg->waypoints.size(), current_waypoints_.size());

    is_waypoint_set_ = true;

    is_last_point = false;
    next_waypoint_number_ = -1;
    end_waypoint_index = -1;
    is_in_cross = false;
    pre_index = 0;
    search_start_index = 0;
    clearest_points_index = -1;

    search_radius = 2;  // 搜索半径
    cross_in = false;
    cross_out = false;
    almost_reach = false;
    lock_index = -1;
    save_index = -1;
    is_not_lane = true;

    current_pose_closet_index = 0;

    return true;
}

void PurePursuitNode::publishControlCommandStamped(const bool &can_get_curvature, const double &curvature) {
    can_msgs::ecu ecu_ctl;

    // ecu_ctl.motor = can_get_curvature ? computeCommandVelocity() : 0;

    double steer = atan(wheel_base_ * curvature);

    steer = (steer + pre_steer + pre_pre_steer) / 3.0;

    pre_pre_steer = pre_steer;
    pre_steer = steer;
    // steer = steer / 0.06 * 30;  // 转化弧度为角度
    steer = steer * 180.0 / 3.141592654;

    // 如果获取失败则置为0
    ecu_ctl.steer = can_get_curvature ? steer : 0;

    ecu_ctl.shift = ecu_ctl.SHIFT_D;
    ecu_ctl.brake = false;

    ecu_ctl.motor = can_get_curvature ? computeCommandVelocity() : 0;
    // static bool enter_ending_process = false;

    // if (getPlaneDistance(current_pose_.position, current_waypoints_.at(current_waypoints_.size() - 1).pose.pose.position) < 3.0) {
    // ROS_INFO("current_pose_closet_index=%d, current_waypoints_.size()=%d", current_pose_closet_index, current_waypoints_.size());
    if (current_pose_closet_index >= end_waypoint_index - 5) {  // TODO:: 增加考虑刹车距离, end_waytpoint_index应减去 (刹车距离/点密度)
        // ecu_ctl.shift = ecu_ctl.SHIFT_D;
        // if (ecu_ctl.motor > 0.01) {
        //     ecu_ctl.brake = true;
        //     std::cout << "reach last point" << std::endl;
        // }
        // ecu_ctl.motor = 0;
        // ecu_ctl.steer = 0;
        // std_msgs::Bool finished;
        // finished.data = true;
        // pub_pure_pursuit_finished.publish(finished);
        do_ending_process(ecu_ctl);
        return;
    }

    if (!this->is_target_front){
        ecu_ctl.shift = ecu_ctl.SHIFT_R;
    }
    ecu_ctl.header.stamp = ros::Time::now();
    pub_control.publish(ecu_ctl);
}

void PurePursuitNode::publishBrakeCommand() {
    can_msgs::ecu ecu_ctl;
    ecu_ctl.header.stamp = ros::Time::now();
    ecu_ctl.brake = true;
    ecu_ctl.shift = 0;
    ecu_ctl.motor = 0;
    ecu_ctl.steer = 0;
    pub_control.publish(ecu_ctl);
}

bool PurePursuitNode::do_ending_process(can_msgs::ecu msg_ecu) {
    ROS_INFO("[waypoint_follower] entering ending process");

    // double dist_to_last_point = getPlaneDistance(current_pose_.position, current_waypoints_.at(current_waypoints_.size() - 1).pose.pose.position);
    // // cur_speed 来自vehicle_status,如果没有该消息，则用最后一个ecu.motor代替(暂时方案)
    // double cur_vel = (cur_speed < 0.0) ? msg_ecu.motor : cur_speed;
    // double sum_t = 2.0 * dist_to_last_point / cur_vel;  // 两倍的时间用来减速
    // double pre_dist = dist_to_last_point + 0.5;
    // double time_consumed = 0.0;

    // // std::cout << cur_vel << ", " << dist_to_last_point << std::endl;
    // while (cur_vel > 0.1 && pre_dist >= dist_to_last_point && time_consumed < sum_t) {
    //     // ROS_INFO("do_ending_process >> 1");
    //     static ros::Time st = ros::Time::now();
    //     std::cout << cur_vel << ", " << dist_to_last_point << std::endl;
    //     pre_dist = dist_to_last_point + 0.01;
    //     // msg_ecu.motor = dist_to_last_point / (2.0 * cur_vel);
    //     msg_ecu.motor = msg_ecu.motor * 0.97;
    //     msg_ecu.header.stamp = ros::Time::now();
    //     msg_ecu.brake = false;
    //     msg_ecu.steer = 0.0;
    //     msg_ecu.shift = msg_ecu.SHIFT_D;

    //     pub_control.publish(msg_ecu);
    //     ros::Duration(0.05).sleep();

    //     dist_to_last_point = getPlaneDistance(current_pose_.position, current_waypoints_.at(current_waypoints_.size() - 1).pose.pose.position);
    //     cur_vel = (cur_speed < 0.0) ? msg_ecu.motor : cur_speed;
    //     time_consumed += (ros::Time::now() - st).toSec();
    // }

    msg_ecu.header.stamp = ros::Time::now();
    msg_ecu.brake = true;
    msg_ecu.motor = 0.0;
    msg_ecu.steer = 0.0;
    msg_ecu.shift = msg_ecu.SHIFT_N;
    pub_control.publish(msg_ecu);

    // TODO:: reset waypoint follower
    is_waypoint_set_ = false;

    std_msgs::Bool finished;
    finished.data = true;
    pub_pure_pursuit_finished.publish(finished);
    ROS_INFO("[waypoint_follower] Task finished! Waitting for new task...");
    return true;
}

double PurePursuitNode::computeLookaheadDistance() const {
    if (is_const_lookahead_dis_ == true) {
        if (is_in_cross || is_not_lane) {
            return cross_lookahead_dis;
        } else {
            return lane_lookahead_dis;
        }
    }
    // else
    // {
    //   return 5;
    // }

    // 通过速度乘系数得到的前视距离不小于最小前视距离，不大于当前速度×7
    double maximum_lookahead_distance = cur_speed * 7;
    double ld = cur_speed * lookahead_distance_ratio_;
    return ld < (minimum_lookahead_distance_) ? minimum_lookahead_distance_ : (ld > maximum_lookahead_distance) ? maximum_lookahead_distance : ld;
}

double PurePursuitNode::computeCommandVelocity() {
    if (is_const_speed_command_ == true) return const_velocity_;
    // 判断是否在直线上，修改前进速度
    if (is_in_cross || is_not_lane) {
        command_linear_velocity_ = cross_speed_limit;  // 弯道处的速度
    } else {
        command_linear_velocity_ = lane_speed_limit;  // 直线的速度
    }
    // printf("--------------speed: %f\n", command_linear_velocity_);
    return command_linear_velocity_;
}

// 获取当前位置
void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg) {
    // 读取了当前的车辆pose消息，里面有xyz和四元数
    last_cpose_update_time = ros::Time::now();
    current_pose_ = msg->pose;
    if ( !this->is_target_front ){
        // yaw反向180度
        // TODO::
        //     1. yaw旋转180度
        //     2. 发布ecu的时候, 如果非istarget_front, 则设置shift=R,
        //     3. fcontroller逻辑处理
        //     4. 测试
        double yaw, pitch, roll;
        tf2::Matrix3x3 mat(tf2::Quaternion(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        ));
        mat.getEulerYPR(yaw, pitch, roll);
        // ROS_INFO("RPY = %.2f, %.2f, %.2f", roll, pitch, yaw);

        yaw += 3.141592654;

        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
        current_pose_.orientation = quat;

        // geometry_msgs::PoseStamped current_pose_reversed;
        // current_pose_reversed.header.frame_id = "map";
        // current_pose_reversed.pose = current_pose_;
        // this->pub_reversed_current_pose.publish(current_pose_reversed);
    }
    is_pose_set_ = true;
    if (!is_waypoint_set_) return;

    float min_dist = 9999999999.9;
    int search_start = ((current_pose_closet_index-200) > 0) ? (current_pose_closet_index-200) : 0;
    int search_end = ((search_start+400) < current_waypoints_.size()) ? (search_start+400) : current_waypoints_.size();
    int temp_closet_index = 0;

    bool closet_index_got = false;
    bool is_second_search = false;
    while ( !closet_index_got ) {
        for (int i = search_start; i < search_end; i++){
            smartcar_msgs::Waypoint p = current_waypoints_[i];
            float cur_distance = getPlaneDistance(p.pose.pose.position, current_pose_.position);
            if ( cur_distance < min_dist ){
                min_dist = cur_distance;
                temp_closet_index = i;
            }
        }

        // 临时方案：要求车辆距离轨迹线不能超过1m
        if (min_dist > 1) {
            if (!is_second_search) {
                is_second_search = true;
                search_start = 0;
                search_end = current_waypoints_.size();
                min_dist = 9999999999.9;
                continue;
            }else{
                closet_index_got = false;
                break;
            }
        }

        closet_index_got = true;
    }

    if (closet_index_got){
        current_pose_closet_index = temp_closet_index;
    }else{
        // 车辆距离路径过远, 丢失, 终止
        current_pose_closet_index = current_waypoints_.size()-1;
    }

    automotive_msgs::PPTWayPointStatus msg_waypoints_status;
    msg_waypoints_status.closet_index = current_pose_closet_index;
    msg_waypoints_status.end_index = end_waypoint_index;
    msg_waypoints_status.step_size = this->step_size;
    this->pub_closet_index.publish(msg_waypoints_status);

    // ROS_INFO("path_size: %d, search_index: %d, search_end: %d, min_dist: %d, closet_index: %d", current_waypoints_.size(), search_start, search_end, min_dist, current_pose_closet_index);
}

// 获取当前速度
void PurePursuitNode::callbackFromCurrentVelocity(const can_msgs::vehicle_status &msg) { cur_speed = msg.cur_speed; }

void PurePursuitNode::callbackFromSafeWayPoints(const nav_msgs::PathConstPtr &msg) {
    smartcar_msgs::Lane smartcar_lane;
    for (const auto p: msg->poses) {
        smartcar_msgs::Waypoint w;
        w.pose.pose = p.pose;
        smartcar_lane.waypoints.push_back(w);
    }
    this->__handle_followed_path(smartcar_lane);
}

void PurePursuitNode::callbackFromBestLocalTrajectory(const smartcar_msgs::Lane &msg) {
    this->__handle_followed_path(msg);
}

void PurePursuitNode::callbackFromWayPoints(const smartcar_msgs::Lane &msg) {

    this->__handle_followed_path(msg);
}

void PurePursuitNode::__handle_followed_path(smartcar_msgs::Lane msg) {
    if (msg.waypoints.size() < 2) return;
    end_waypoint_index = msg.waypoints.size();
    float _total_distance = 0.0;
    for (int i = 1; i < msg.waypoints.size(); i++) {
        _total_distance += getPlaneDistance(msg.waypoints[i-1].pose.pose.position, msg.waypoints[i].pose.pose.position);
    }
    ROS_INFO("[pure_pursuit] Total trajectory length = %.2f m", _total_distance);
    this->step_size = _total_distance / msg.waypoints.size();

    // 延长尾部, 避免终点震荡
    smartcar_msgs::Lane lane_extended;
    lane_extended.waypoints = msg.waypoints;
    if (lane_extended.waypoints.size() > 5) {
        smartcar_msgs::Waypoint st = lane_extended.waypoints[lane_extended.waypoints.size() - 5];
        smartcar_msgs::Waypoint end = lane_extended.waypoints[lane_extended.waypoints.size()-1];
        double rad = atan2(end.pose.pose.position.y - st.pose.pose.position.y, end.pose.pose.position.x - st.pose.pose.position.x);
        for (int i = 1; i < 100; i++){
            smartcar_msgs::Waypoint t;
            t.pose.pose.position.x = end.pose.pose.position.x + i*0.1*cos(rad);
            t.pose.pose.position.y = end.pose.pose.position.y + i*0.1*sin(rad);
            t.speed_limit = end.speed_limit;
            lane_extended.waypoints.push_back(t);
        }
    }

    current_waypoints_ = lane_extended.waypoints;

    is_waypoint_set_ = true;

    is_last_point = false;
    next_waypoint_number_ = -1;
    is_in_cross = false;
    pre_index = 0;
    search_start_index = 0;
    clearest_points_index = -1;

    search_radius = 2;  // 搜索半径
    cross_in = false;
    cross_out = false;
    almost_reach = false;
    lock_index = -1;
    save_index = -1;
    is_not_lane = true;

    current_pose_closet_index = 0;
}

}  // namespace waypoint_follower
