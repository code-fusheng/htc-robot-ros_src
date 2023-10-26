#include <can_adapter.h>

const double TH_LANE_CROSS = 10;  // steer低于该值表示位于直线,否则位于弯道
const double default_obstacle = 100000.0;

namespace can_adapter
{

    CanAdapterNode::CanAdapterNode()
        : pnh_("~")
    {
        initForROS();
        front_obstacle_dist = default_obstacle;
        front_obstacle_update_time = ros::Time::now();
        front_obstacle_stop_time = ros::Time::now();

        back_obstacle_dist = default_obstacle;
        back_obstacle_update_time = ros::Time::now();
        back_obstacle_stop_time = ros::Time::now();

        lean_obs_dist_rear = 0.0;
        lean_obs_dist_remote = default_obstacle;
        lean_obs_update_time = ros::Time::now();
        lean_obs_stop_time = ros::Time::now();

    }

    CanAdapterNode::~CanAdapterNode() {}

    int CanAdapterNode::brake_check(const can_msgs::ecu &source_msg) {
        ros::Time now = ros::Time::now();

        if ( car_status.main_state != smartcar_msgs::State::RUN  || source_msg.brake == true) {
            return automotive_msgs::CanAdapterStatus::BRAKE_MANUAL;  // TODO::修改为系统触发刹车
        }

        // if ( (now-urgent_stop_time).toSec() < this->_obstacle_lock_time )
        //     return automotive_msgs::CanAdapterStatus::BRAKE_OBSTACLE;

        if ( (now - front_obstacle_stop_time).toSec() < this->_obstacle_lock_time) {
            // ROS_INFO("brake lock_time");
            return automotive_msgs::CanAdapterStatus::BRAKE_OBSTACLE;
        }

        if ( (now - front_obstacle_update_time).toSec() < 0.3 && 0.05 < front_obstacle_dist && front_obstacle_dist < front_dist_stop ) {
            front_obstacle_stop_time = ros::Time::now();
            ROS_INFO("brake obs front_obstacle_dist: %.2f, front_dist_stop: %.2f", front_obstacle_dist, front_dist_stop);
            return automotive_msgs::CanAdapterStatus::BRAKE_OBSTACLE;
        }

        // 如果laser数据有延迟, 则标识传感器异常, 禁止行车
        if ( (now - front_obstacle_update_time).toSec() > 0.5 ) {
            // ROS_INFO("brake front_obstacle_update_time delay");
            return automotive_msgs::CanAdapterStatus::DEVICE_UNFUNC;
        }

        return automotive_msgs::CanAdapterStatus::NO_BRAKE;
    }

    // callbacks
    void CanAdapterNode::callbackFromSelfDriving(const can_msgs::ecu &source_msg)
    {
        // 如果当前不是自动驾驶模式, 则不执行自动驾驶系统的指令
        if (smartcar_run_type != automotive_msgs::SmartcarRunType::Response::RUNTYME_AUTO) {
            return;
        }

        ros::Time now = ros::Time::now();
        can_msgs::ecu pub_msg;
        static int previous_status = -1;

        // 检车刹车条件, 及时处理
        int ret = this->brake_check(source_msg);

        if (ret != previous_status) {
            previous_status = ret;
            if (ret == automotive_msgs::CanAdapterStatus::NO_BRAKE)
                publish_brake_switch_and_reason(true, ret);
            else
                publish_brake_switch_and_reason(false, ret);
        }

        if (ret != automotive_msgs::CanAdapterStatus::NO_BRAKE) {
            // 对于fcontroller jd01双转,为了实现低速稳定性，因此取消了刹车时候shift=N的限制，以使得能够利用点击反拖的特性减速
            pub_msg.shift = pub_msg.SHIFT_D;  // 对于非JD01车型, 需注释掉该行, 并开启下一行
            // pub_msg.shift = can_msgs::ecu::SHIFT_N;

            pub_msg.brake = true;
            pub_msg.motor = 0;
            pub_ctl.publish(pub_msg);
            return;
        }

        // if ( (now - lean_obs_update_time).toSec() < 0.3){
        //     // 斜坡检测雷达检测到距离过近, 有障碍物影响同行
        //     if (lean_obs_dist_rear < th_lean_obs_dist_rear) {
        //         pub_msg.motor = 0.0;
        //         pub_msg.brake = true;
        //         pub_msg.shift = pub_msg.SHIFT_N;
        //         pub_ctl.publish(pub_msg);
        //         lean_obs_stop_time = now;
        //         return;
        //     }
        //     // 斜坡检测雷达检测到距离过远, 可能有斜坡
        //     if (lean_obs_dist_remote > th_lean_obs_dist_remote) {
        //         pub_msg.motor = 0.0;
        //         pub_msg.brake = true;
        //         pub_msg.shift = pub_msg.SHIFT_N;
        //         pub_ctl.publish(pub_msg);
        //         lean_obs_stop_time = now;
        //         return;
        //     }
        // }

        // 非刹车状态, 对速度和角度进行重新设置
        static double pre_pre_speed = 0.0, pre_speed = 0.0, speed = 0.0;
        double steer = source_msg.steer;
        steer += bias;

        // 角度滤波
        static double pre_pre_steer = 0, pre_steer = 0;
        steer = steer > 0 ? steer * steer_right_param : steer * steer_left_param;  // firtst check lane/cross, then fix steer value

        speed = std::fmin(this->max_speed_limit, source_msg.motor);  // 取其小  // when speed < 0, it will remain its value
        speed = (pre_speed + speed) / 2.0;  // 简单的滤一下波
        // ROS_INFO("Final speed = %.2f", speed);

        pub_msg.brake = false;
        pub_msg.motor = speed;
        pub_msg.steer = steer;
        pub_msg.shift = source_msg.shift;

        // step1: 根据障碍物距离调整速度 current speed is frontward and frontward has obstacle
        double _speed_obstacle_fix = pub_msg.motor;
        if ( (now - front_obstacle_update_time).toSec() < 0.3 && pub_msg.motor > 0 && pub_msg.shift == pub_msg.SHIFT_D ) {
            _speed_obstacle_fix = this->min_speed_limit + (pub_msg.motor-this->min_speed_limit)*(front_obstacle_dist-front_dist_stop)/(max_detection_distance-front_dist_stop);
        }

        // 后置单线
        // if ( (now - back_obstacle_update_time).toSec() < 0.3 && (pub_msg.motor < 0 || pub_msg.shift == pub_msg.SHIFT_R) ) {
        //     _speed_obstacle_fix = (back_obstacle_dist/back_dist_stop)*pub_msg.motor;
        // }

        // step2: 根据距离终点位置的距离调整速度
        double _speed_endpoint_fix = pub_msg.motor;
        if (this->is_current_waypoint_status_valid) {
            this->_waypoint_status_lock.lock();
            int current_closet_index = this->current_waypoint_status.closet_index;
            int end_index = this->current_waypoint_status.end_index;
            float step_size = this->current_waypoint_status.step_size;
            this->_waypoint_status_lock.unlock();

            // BUG_ON() 不合理的current_closet_index, 当前waypoint index不应该大于end_index
            if (current_closet_index > end_index) {
                ROS_WARN("[can_adapter] Invalid closet waypoint index! received closet waypoint index = %d, end_index = %d",
                          current_closet_index,
                          end_index);
                current_closet_index = end_index;
            }

            double dist_to_end = step_size*(end_index - current_closet_index);
            if (dist_to_end < this->dist_slow_speed_at_end) {
                // _speed_endpoint_fix = pub_msg.motor * dist_to_end/this->dist_slow_speed_at_end;
                _speed_endpoint_fix = this->min_speed_limit + (pub_msg.motor-this->min_speed_limit)*(dist_to_end/this->dist_slow_speed_at_end);
            }
        }

        // step3: 根据当前转角大小调整速度, 转弯幅度越大, 速度越低.
        //        最低过弯速度为limit_min, 对应转角30
        double steer_speed_fix = pub_msg.motor;
        if (std::fabs(pub_msg.steer) >= 30.0)
            steer_speed_fix = this->min_speed_limit;
        else {
            // steer_speed_fix = this->min_speed_limit + (pub_msg.motor - this->min_speed_limit) * (30 - std::fabs(pub_msg.steer)) / 30;
            steer_speed_fix = pub_msg.motor - (pub_msg.motor - this->min_speed_limit) * (std::fabs(pub_msg.steer) / 30.0);
            // ROS_INFO("steer_fpeed_fix = %.2f", steer_speed_fix);
        }
        
        // 综合以上, 设定pub_msg.motor为其最小值
        pub_msg.motor = std::fmin(_speed_obstacle_fix, _speed_endpoint_fix);
        pub_msg.motor = std::fmin(pub_msg.motor, steer_speed_fix);

        // TODO:: BUG_ON() 如果速度非常小，认为其已经为结束状态

        if (pub_msg.motor < this->min_speed_limit) pub_msg.motor = this->min_speed_limit;  // 最低速度限制
        if (pub_msg.motor > this->max_speed_limit) {      // 最高速度限制
            ROS_WARN("[can_adapter] vehicle speed cannot be larger than %.2f, now motor=%.2f", this->max_speed_limit, pub_msg.motor);
            pub_msg.motor = this->max_speed_limit;
        }

        pub_ctl.publish(pub_msg);
        pre_pre_speed = pre_speed;
        pre_speed = speed;
        pre_pre_steer = pre_steer;
        pre_steer = steer;
    }

    void CanAdapterNode::callbackFromRemoteControl(const can_msgs::ecu &source_msg)
    {
        pub_ctl.publish(source_msg);
    }

    void CanAdapterNode::callbackFromStatus(const smartcar_msgs::State &status_msg)
    {
        car_status.main_state = status_msg.main_state;
    }

    void CanAdapterNode::callbackFromVehicleStatus(const can_msgs::vehicle_statusPtr &msg){
        vehicle_status_update_time = ros::Time::now();
        cur_speed = msg->cur_speed;  // m/s
        cur_steer = msg->cur_steer; // angle
    }

    void CanAdapterNode::callbackSimpleObsDetect(const automotive_msgs::SimpleObstacleDistConstPtr &msg) {
        switch ( msg->direction )
        {
        case (automotive_msgs::SimpleObstacleDist::DIRECTION_FRONT) :
            front_obstacle_dist = msg->distance;
            front_obstacle_update_time = ros::Time::now();
            break;
        case (automotive_msgs::SimpleObstacleDist::DIRECTION_BACK) :
            back_obstacle_dist = msg->distance;
            back_obstacle_update_time = ros::Time::now();
            break;

        default:
            ROS_WARN("[can adapter] Unknown obstacle direction: %d", msg->direction);
            break;
        }
    }

    void CanAdapterNode::callbackUserLaserRange(const automotive_msgs::SimpleObstacleDistConstPtr &msg) {
        lean_obs_update_time = ros::Time::now();
        lean_obs_dist_rear = msg->distance;
        lean_obs_dist_remote = msg->distance_remote;
        // ROS_INFO("lean dist rear = %.2f, lean dist remote = %.2f", lean_obs_dist_rear, lean_obs_dist_remote);
    }

    void CanAdapterNode::callbackFromPPTWaypointStatus(const automotive_msgs::PPTWayPointStatus &msg) {
        _waypoint_status_lock.lock();

        this->is_current_waypoint_status_valid = true;
        this->current_waypoint_status = msg;

        _waypoint_status_lock.unlock();
    }

    bool CanAdapterNode::onServiceRequestSmartcarType(automotive_msgs::SmartcarRunType::Request &req, automotive_msgs::SmartcarRunType::Response &res) {
        res.res_type = smartcar_run_type;
        return true;
    }

    bool CanAdapterNode::onServiceSetSmartcarType(automotive_msgs::SmartcarRunType::Request &req, automotive_msgs::SmartcarRunType::Response &res) {
        std::string message = "[Can adapter] Change Run Type from ";
        if (smartcar_run_type == automotive_msgs::SmartcarRunType::Response::RUNTYME_AUTO) {
            message += "AUTO ";
        }else{
            message += "MANUAL ";
        }

        smartcar_run_type = req.run_type;
        res.res_type = smartcar_run_type;

        if (smartcar_run_type == automotive_msgs::SmartcarRunType::Response::RUNTYME_AUTO) {
            message += "to AUTO.";
        }else{
            message += "to MANUAL.";
        }
        ROS_INFO(message.c_str());

        return true;
    }

    void CanAdapterNode::publish_brake_switch_and_reason(bool is_running, int reason) {
        static ros::Publisher pub_can_adapter_status = nh_.advertise<automotive_msgs::CanAdapterStatus>("/can_adapter_status", 3);

        automotive_msgs::CanAdapterStatus pub_msg;
        pub_msg.is_running = is_running;
        pub_msg.brake_reason = reason;
        
        pub_can_adapter_status.publish(pub_msg);
    }

    // initializer
    void CanAdapterNode::initForROS()
    {
        smartcar_run_type = automotive_msgs::SmartcarRunType::Response::RUNTYME_AUTO;

        pnh_.param<std::string>("sub_topic", sub_topic, "");
        pnh_.param<std::string>("pub_topic", pub_topic, "");
        pnh_.param<std::string>("sub_status_topic", sub_status_topic, "");
        pnh_.param<float>("distance_ending_process", dist_slow_speed_at_end, 4.0);  // 进入终点进行减速刹车的距离
        pnh_.param<float>("min_speed_limit", min_speed_limit, 1.0);                 // 最小速度限制
        pnh_.param<float>("max_speed_limit", max_speed_limit, 4.0);                 // 最大速度限制
        pnh_.param<float>("max_detection_distance", max_detection_distance, 6.0);   // 该范围内的障碍物将触发减速

        if (sub_topic == "" || sub_status_topic == "" || pub_topic == "")
        {
            ROS_ERROR("[can_adapter]: topic is not set!");
        }
        car_status.main_state = smartcar_msgs::State::PAUSE; // 起步暂停,需手动出发启动
        vehicle_status_update_time = ros::Time::now();

        pnh_.param<double>("bias", bias, 0);
        pnh_.param<double>("steer_left_param", steer_left_param, 1);
        pnh_.param<double>("steer_right_param", steer_right_param, 1);

        // 这两个参数已废弃使用deprecated
        // pnh_.param<double>("lane_speed_limit", lane_speed_limit, 7.2);  // km/h  
        // pnh_.param<double>("cross_speed_limit", cross_speed_limit, 3.6);

        pnh_.param<double>("ratio_fix_speed", ratio_fix_speed, 2);
        pnh_.param<double>("dist_front_stop", front_dist_stop, 2);      // 前向检测到障碍物执行抱死刹车的距离
        pnh_.param<double>("dist_back_stop", back_dist_stop, 1);        // 后向检测到障碍物执行抱死刹车的距离
        
        pnh_.param<std::string>("sub_laser_urgent_obs", sub_laser_urgent_obs, "/detection/simple_obstacle_detection"); // laser单线障碍物信息topic

        // 斜坡检测
        pnh_.param<double>("th_lean_obs_dist_rear", th_lean_obs_dist_rear, 0.5);
        pnh_.param<double>("th_lean_obs_dist_remote", th_lean_obs_dist_remote, 0.8);
        
        pub_ctl = nh_.advertise<can_msgs::ecu>(pub_topic, 1);

        request_automotive_type_service = nh_.advertiseService("request_smartcar_type", &CanAdapterNode::onServiceRequestSmartcarType, this);
        set_automotive_type_service = nh_.advertiseService("set_smartcar_type", &CanAdapterNode::onServiceSetSmartcarType, this);

        sub_ecu = nh_.subscribe(sub_topic, 10, &CanAdapterNode::callbackFromSelfDriving, this);
        sub_remote_control = nh_.subscribe("/remote_control/ecu", 10, &CanAdapterNode::callbackFromRemoteControl, this);
        sub_status = nh_.subscribe(sub_status_topic, 10, &CanAdapterNode::callbackFromStatus, this);
        sub_simple_obstacle_detection = nh_.subscribe(sub_laser_urgent_obs, 10, &CanAdapterNode::callbackSimpleObsDetect, this);
        sub_ppt_waypoints_status = nh_.subscribe("/ppt/waypoints_status", 3, &CanAdapterNode::callbackFromPPTWaypointStatus, this);

    }
} // namespace can_adapter
