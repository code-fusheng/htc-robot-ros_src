#include <ros/ros.h>

#include "pure_persuit.h"

namespace waypoint_follower {

double PurePursuitNode::getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2) {
    // distance between target 1 and target2 in 2-D
    tf::Vector3 v1 = point2vector(target1);
    v1.setZ(0);
    tf::Vector3 v2 = point2vector(target2);
    v2.setZ(0);
    return tf::tfDistance(v1, v2);
}

double PurePursuitNode::getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c) {
    double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

    return d;
}

tf::Vector3 PurePursuitNode::rotateUnitVector(tf::Vector3 unit_vector, double degree) {
    tf::Vector3 w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(), sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
    tf::Vector3 unit_w1 = w1.normalize();
    return unit_w1;
}

tf::Vector3 PurePursuitNode::point2vector(geometry_msgs::Point point) {
    tf::Vector3 vector(point.x, point.y, point.z);
    return vector;
}

bool PurePursuitNode::getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double* a, double* b, double* c) {
    //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
    double sub_x = fabs(start.x - end.x);
    double sub_y = fabs(start.y - end.y);
    double error = pow(10, -5);  // 0.00001

    if (sub_x < error && sub_y < error) {
        ROS_INFO("two points are the same point!!");
        return false;
    }

    *a = end.y - start.y;
    *b = (-1) * (end.x - start.x);
    *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

    return true;
}

geometry_msgs::Point PurePursuitNode::calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose) {
    // calculation relative coordinate of point from current_pose frame
    tf::Transform inverse;
    tf::poseMsgToTF(current_pose, inverse);
    tf::Transform transform = inverse.inverse();

    tf::Point p;
    pointMsgToTF(point_msg, p);
    tf::Point tf_p = transform * p;
    geometry_msgs::Point tf_point_msg;
    pointTFToMsg(tf_p, tf_point_msg);

    return tf_point_msg;
}
}  // namespace waypoint_follower