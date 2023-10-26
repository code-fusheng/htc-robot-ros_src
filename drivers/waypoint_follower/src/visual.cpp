#include <ros/ros.h>

#include "pure_persuit.h"

namespace waypoint_follower {

void PurePursuitNode::visualInRviz() {
    visualization_msgs::MarkerArray msg_marker_array;

    if (0 <= next_waypoint_number_ < current_waypoints_.size()) {
        visualization_msgs::Marker marker_next_waypoint;
        marker_next_waypoint.header.frame_id = "/map";
        marker_next_waypoint.header.stamp = ros::Time::now();
        marker_next_waypoint.ns = "target_waypoint";
        marker_next_waypoint.id = 0;
        marker_next_waypoint.type = visualization_msgs::Marker::ARROW;
        marker_next_waypoint.action = visualization_msgs::Marker::ADD;
        marker_next_waypoint.pose = current_waypoints_[next_waypoint_number_].pose.pose;
        marker_next_waypoint.color.a = 1.0;
        marker_next_waypoint.color.r = 1.0;
        marker_next_waypoint.color.g = 0.0;
        marker_next_waypoint.color.b = 0.0;
        marker_next_waypoint.scale.x = 1.5;
        marker_next_waypoint.scale.y = 0.45;
        marker_next_waypoint.scale.z = 0.45;
        msg_marker_array.markers.push_back(marker_next_waypoint);
    } else {
        ROS_ERROR("[pure_pursuit] Unexpected target point index: %d, current_waypoints size is: %d", next_waypoint_number_,
                  current_waypoints_.size());
    }

    // if (0 <= clearest_points_index < current_waypoints_.size()) {
    //     visualization_msgs::Marker marker_closet_waypoint;
    //     marker_closet_waypoint.header.frame_id = "/map";
    //     marker_closet_waypoint.header.stamp = ros::Time::now();
    //     marker_closet_waypoint.ns = "target_waypoint";
    //     marker_closet_waypoint.id = 1;
    //     marker_closet_waypoint.type = visualization_msgs::Marker::ARROW;
    //     marker_closet_waypoint.action = visualization_msgs::Marker::ADD;
    //     marker_closet_waypoint.pose = current_waypoints_[clearest_points_index].pose.pose;
    //     marker_closet_waypoint.color.a = 1.0;
    //     marker_closet_waypoint.color.r = 1.0;
    //     marker_closet_waypoint.color.g = 0.3;
    //     marker_closet_waypoint.color.b = 0.8;
    //     marker_closet_waypoint.scale.x = 1.5;
    //     marker_closet_waypoint.scale.y = 0.3;
    //     marker_closet_waypoint.scale.z = 0.3;
    //     msg_marker_array.markers.push_back(marker_closet_waypoint);
    // } else {
    //     ROS_ERROR("[pure_pursuit] Unexpected closet point index: %d, current_waypoints size is: %d", clearest_points_index,
    //               current_waypoints_.size());
    // }
    // std::cout << msg_marker_array << std::endl;
    pub_target.publish(msg_marker_array);

    // for (size_t i = 0; i < current_waypoints_.size(); i++)
    // {
    //     // visual global path in rviz
    //     // geometry_msgs::PoseStamped msg_pose;
    //     // msg_path.header.stamp = ros::Time();
    //     // msg_path.header.frame_id = "/map";
    //     // msg_pose.pose = current_waypoints_[i].pose.pose;
    //     // msg_path.poses.push_back(msg_pose);

    //     // visual waypoint pose in rviz
    //     visualization_msgs::Marker msg_arrow_marker;
    //     msg_arrow_marker.header.frame_id = "/map";
    //     msg_arrow_marker.header.stamp = ros::Time();
    //     msg_arrow_marker.ns = "target_waypoint";
    //     msg_arrow_marker.id = i;
    //     msg_arrow_marker.type = visualization_msgs::Marker::ARROW;
    //     msg_arrow_marker.action = visualization_msgs::Marker::ADD;
    //     msg_arrow_marker.pose = current_waypoints_[i].pose.pose;
    //     if (int(i) == next_waypoint_number_)
    //     {
    //         msg_arrow_marker.color.r = 1.0;
    //         msg_arrow_marker.color.g = 0.0;
    //         msg_arrow_marker.color.b = 0.0;
    //         msg_arrow_marker.scale.x = 1.5;
    //         msg_arrow_marker.scale.y = 0.45;
    //         msg_arrow_marker.scale.z = 0.45;
    //     }
    //     else if (int(i) == clearest_points_index)
    //     {
    //         msg_arrow_marker.color.r = 1.0;
    //         msg_arrow_marker.color.g = 0.3;
    //         msg_arrow_marker.color.b = 0.8;
    //         msg_arrow_marker.scale.x = 1.5;
    //         msg_arrow_marker.scale.y = 0.35;
    //         msg_arrow_marker.scale.z = 0.35;
    //     }
    //     else
    //     {
    //         msg_arrow_marker.color.r = 0.0;
    //         msg_arrow_marker.color.g = 1.0;
    //         msg_arrow_marker.color.b = 0.0;
    //         msg_arrow_marker.scale.x = 0.7;
    //         msg_arrow_marker.scale.y = 0.05;
    //         msg_arrow_marker.scale.z = 0.05;
    //     }
    //     msg_arrow_marker.color.a = 1.0; // Don't forget to set the alpha!
    //     msg_marker_array.markers.push_back(msg_arrow_marker);
    // }
    // pub_path.publish(msg_path);

    // // visual car model
    // visualization_msgs::Marker msg_car_marker;
    // msg_car_marker.header.frame_id = "/map";
    // msg_car_marker.header.stamp = ros::Time();
    // msg_car_marker.ns = "car";
    // msg_car_marker.id = 0;
    // msg_car_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    // msg_car_marker.action = visualization_msgs::Marker::ADD;
    // double current_roll, current_yaw, current_pitch;
    // tf::Quaternion quat;
    // tf::quaternionMsgToTF(current_waypoints_[next_waypoint_number_ - 4].pose.pose.orientation, quat);
    // tf::Matrix3x3(quat).getRPY(current_roll, current_pitch, current_yaw);
    // msg_car_marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(90 * (M_PI / 180.0),
    //     0 * (M_PI / 180.0),
    //     current_yaw + M_PI / 2.0);
    // msg_car_marker.pose.position = current_pose_.position;
    // msg_car_marker.pose.position.z = current_waypoints_[next_waypoint_number_ - 4].pose.pose.position.z;
    // msg_car_marker.color.r = 0.7;
    // msg_car_marker.color.g = 0.7;
    // msg_car_marker.color.b = 0.7;
    // msg_car_marker.color.a = 1; // Don't forget to set the alpha!
    // msg_car_marker.scale.x = 0.6;
    // msg_car_marker.scale.y = 0.6;
    // msg_car_marker.scale.z = 0.6;
    // msg_car_marker.mesh_use_embedded_materials = true;
    // msg_car_marker.mesh_resource = "package://car_model/ferrari/dae.DAE";
    // pub_car_model.publish(msg_car_marker);
}
}  // namespace waypoint_follower