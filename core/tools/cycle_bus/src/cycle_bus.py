#!/usr/bin/python
# -*-coding:utf-8-*-

import rospy
from geometry_msgs.msg import PoseStamped
from smartcar_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path
import csv
import math
import os
import time

class App:
    def __init__(self):
        rospy.init_node("cycle_bus", log_level=rospy.DEBUG)
        self._reverse = False
        self.path_dir = None
        self.current_pose = None
        self.followed_path = []
        self.publish_interval = 10
        self._init()

    def _init(self):
        self.path_dir = rospy.get_param("~path_dir", default="")
        if not os.path.exists(self.path_dir):
            rospy.logerr("[cycle bus] Path dir ({}) not exist, exit.".format(self.path_dir))
            exit(1)
        
        self._reverse = rospy.get_param("~reverse", default=False)
        self.publish_interval = rospy.get_param("~publish_interval", default=10)
        current_pose_topic = rospy.get_param("~current_pose_topic", default="/current_pose")

        self.pub_global_path = rospy.Publisher("/global_path", Lane, queue_size=5)
        self.sub_current_pose = rospy.Subscriber(current_pose_topic, PoseStamped, self._cb_current_pose, queue_size=5)

    def _cb_current_pose(self, msg):
        self.current_pose = msg

    def init_path(self):
        files = os.listdir(self.path_dir)
        path_file = None
        for file in files:
            if file.endswith(".csv"):
                path_file = os.path.join(self.path_dir, file)
                break
        if path_file is None:
            rospy.logerr("[cycle bus] No path file in {}, exit.".format(self.path_dir))
            exit(1)
        
        self.load_path(path_file)
    
    def load_path(self, path_file):
        with open(path_file, 'r') as f:
            csv_reader = csv.reader(f)
            t = list(csv_reader)
            point_num = len(t)
            for i in range(5, point_num):
                _current_pose = t[i]
                p = Waypoint()
                p.pose.pose.position.x = float(_current_pose[0])
                p.pose.pose.position.y = float(_current_pose[1])
                p.pose.pose.position.z = float(_current_pose[2])
                p.pose.pose.orientation.x = float(_current_pose[3])
                p.pose.pose.orientation.y = float(_current_pose[4])
                p.pose.pose.orientation.z = float(_current_pose[5])
                p.pose.pose.orientation.w = float(_current_pose[6])
                self.followed_path.append(p)
        
        if self._reverse:
            self.followed_path.reverse()
    
    def _publish_global_path(self):
        msg_global_path = Lane()
        if self.current_pose is not None:
            closet_index = self._find_closet_index()
            msg_global_path.waypoints.extend(self.followed_path[closet_index:-1])
            msg_global_path.waypoints.extend(self.followed_path[0:closet_index])
        else:
            msg_global_path.waypoints = self.followed_path
        msg_global_path.header.frame_id = "map"
        msg_global_path.header.stamp = rospy.Time().now()
        if len(msg_global_path.waypoints) > 1000:
            msg_global_path.waypoints = msg_global_path.waypoints[0:1000]  # 只截取前1000个点
        else:
            msg_global_path.waypoints = msg_global_path.waypoints[0:len(msg_global_path.waypoints)/2]
        self.pub_global_path.publish(msg_global_path)
        rospy.loginfo("[cycle bus] Publish global path success")
    
    def _find_closet_index(self):
        min_dist = 99999999
        closet_index = 0
        for i in range(len(self.followed_path)):
            dist = self._distance_of_two_point(self.current_pose.pose.position, self.followed_path[i].pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                closet_index = i
        return closet_index

    def _distance_of_two_point(self, p1, p2):
        return math.sqrt(math.pow(p2.x - p1.x, 2) + math.pow(p2.y - p1.y, 2))

    def run(self):
        self.init_path()

        self.rate = rospy.Rate(1.0/self.publish_interval)
        time.sleep(10)  # 延迟10s启动
        while not rospy.is_shutdown():
            self._publish_global_path()
            self.rate.sleep()

    def _on_shutdown(self):
        return

    def shutdown(self):
        self._on_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._on_shutdown()
        return exc_type, exc_val, exc_tb


if __name__ == "__main__":
    with App() as app:
        app.run()
