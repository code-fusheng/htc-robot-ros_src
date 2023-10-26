#!/usr/bin/python2
# encoding: utf-8
"""
Dijkstra.py
Created by Gabriel Rocha on 2013-05-15.
Copyright (c) 2013 __MyCompanyName__. All rights reserved.
"""
from cgitb import reset
import os
from sys import path_importer_cache
from automotive_msgs import srv
import rospy
import csv
# import networkx as nx
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as rosPath
from visualization_msgs.msg import Marker, MarkerArray
from automotive_msgs.msg import ShowGraph, ConfigTrajSet
from automotive_msgs.srv import FunctionSwitch, FunctionSwitchRequest, FunctionSwitchResponse, TransPoints, TransPointsRequest, TransPointsResponse
from smartcar_msgs.msg import Lane, Waypoint
from sensor_msgs.msg import NavSatFix
from lb_cloud_msgs.srv import lbsrv_set_target, lbsrv_set_targetRequest, lbsrv_set_targetResponse
from lb_cloud_msgs.msg import lb_error_code
import copy
import math
# import scipy
import tf
import time

_FUNCTION_STATUS = FunctionSwitchResponse.FUNCTION_ENABLE
sub_initpose = None
sub_current_pose = None
sub_target_pose = None
sub_showgraph = None
sub_config = None

PILOT_MODE = ConfigTrajSet.PILOT_LIDAR

pathid_to_path = {}

pure_data = {}
origin_data = {}  # origin data is dict{(node_st,node_end): Path}
# we can find an exact path based on its id or its node pair(pre_node_id, next_node_id)

graph = {}   # {pre_node:{next_node:length}, ...}
#    graph = {
#         1: { 2: 1, 4: 3 },
#         2: { 1: 1, 4: 1, 3: 5 },
#         3: { 2: 5, 5: 3, 6: 3 },
#         4: { 1: 3, 2: 1, 5: 1 },
#         5: { 4: 1, 3: 3, 6: 7 },
#         6: { 3: 3, 5: 7 },
#         }

tmp_data = None
tmp_graph = None
tmp_pure_data = None

max_node_id = -1
max_path_id = -1

# add publisher
pub_global_path = rospy.Publisher("/global_path", Lane, queue_size=10)
pub_debug_path = rospy.Publisher("/debug_path", rosPath, queue_size=5)
pub_start_pose = rospy.Publisher("/start_pose", Marker, queue_size=5)
pub_target_pose = rospy.Publisher("/target_pose", Marker, queue_size=5)
pub_result_path = rospy.Publisher("/global_planning_out", rosPath, queue_size=10)
pub_default_path = rospy.Publisher("/default_path", MarkerArray, queue_size=10)
pub_default_nav_path = rospy.Publisher("/default_path_nav", rosPath, queue_size=10)
default_path = MarkerArray()
default_path_nav = rosPath()
cnt = 0

# add service client
points_trans_client = rospy.ServiceProxy("/gps2utm/trans_points", TransPoints)

# param
_current_pose_initialized = False
_current_pose_direction_initialized = False
_pathes_initialized = False
# _target_initialized = False

current_pose = PoseStamped()
target_pose = PoseStamped()

min_distance_to_path = 3  # if car is too far from path, then end global planning
distance_start_pose_to_path = None
distance_target_pose_to_path = None

# link st|end param
smooth_st_rate = 5  # min_distance*smooth_st_rate
distance_st_smooth_limit_min = 3
distance_st_sommth_limit_max = 7
smooth_end_rate = 5
distance_end_smooth_limit_min = 3
distance_end_smooth_limit_max = 7
_dist_point_step = 0.3

# smooth param


class Dijkstra():
    def __init__(self, graph, start, end):
        self.graph = graph
        self.start = start
        self.end = end
        self.weights = self.init_weights()
        self.Preds = self.init_Preds()
        self.S = self.init_S()
        self.shortest_path = []
        self._show_base_info()

    def _show_base_info(self):
        for value in range(1, len(self.graph)+1):
            print(value, self.graph.get(value))

        print("\n")
        print("Start: %s \nEnd: %s" % (self.start, self.end))

    def calculate_preds(self):
        position = self.start
        while self.S[self.end-1] == False:
            try:
                self.calculate_weight(position)
                weight, vertex = min(
                    [(weight, vertex)
                     for vertex, weight in enumerate(self.weights)
                     if self.S[vertex] == False]
                )
                if self.S[position-1] == False:
                    self.S[position-1] = True
                position = vertex+1
            except:
                print("Erro - Without shortest path, please check the vertex and weights")
                self.S[self.end-1] = True

    def calculate_weight(self, position):
        for vertex, weight in self.graph.get(position).iteritems():
            if (weight + self.weights[position-1] <= self.weights[vertex-1]):
                self.Preds[vertex-1] = position
                self.weights[vertex-1] = weight + self.weights[position-1]

    def calculate_shortest_path(self):
        end = self.end
        self.shortest_path.append(end)
        while end != -1:
            if self.Preds[end-1] != -1:
                self.shortest_path.append(self.Preds[end-1])
            end = self.Preds[end-1]
        self.shortest_path.reverse()

    def get_Pred(self, vertex):
        return self.Preds[vertex-1]

    def get_weight(self, cost):
        return self.weights[cost-1]

    def init_weights(self):
        weights = []
        for position in range(len(self.graph)):
            weights.append(float("inf"))
        weights[self.start-1] = 0
        return weights

    def init_Preds(self):
        Preds = []
        for position in range(len(self.graph)):
            Preds.append(None)
        Preds[self.start-1] = -1
        return Preds

    def init_S(self):
        S = []
        for position in range(len(self.graph)):
            S.append(False)
        return S

#     def show_graph(self):
#         graph = nx.Graph(self.graph)
#         pos = nx.circular_layout(graph)
#         # pos=nx.spectral_layout(graph)
#         nx.draw_networkx_nodes(graph, pos, node_color='r', node_size=500, alpha=0.8)
#         nx.draw_networkx_edges(graph, pos, width=1, alpha=0.5)
#         nx.draw_networkx_edges(graph, pos,
#                                edge_labels={},
#                                edgelist=self.get_edgelist(),
#                                width=8, alpha=0.5, edge_color='r')
#         nx.draw_networkx_edge_labels(graph, pos, self.get_list_weights_edge(), label_pos=0.3)
#         labels = self.set_labels()
#         nx.draw_networkx_labels(graph, pos, labels, font_size=16)
#         plt.title("Dijkstra")
#         plt.text(0.5, 0.97, "Start: "+str(self.start)+" End: "+str(self.end),
#                  horizontalalignment='center',
#                  transform=plt.gca().transAxes)
#         plt.text(0.5, 0.94, "Shortest Path: "+str(self.shortest_path),
#                  horizontalalignment='center',
#                  transform=plt.gca().transAxes)
#         plt.text(0.5, 0.90, "Weights: "+str(self.weights),
#                  horizontalalignment='center',
#                  transform=plt.gca().transAxes)
#         plt.text(0.5, 0.86, "Pred: "+str(self.Preds),
#                  horizontalalignment='center',
#                  transform=plt.gca().transAxes)
#         plt.axis('off')
#         plt.show()

    def set_labels(self):
        labels = {}
        for position in self.graph.keys():
            labels[position] = position
        return labels

    def get_edgelist(self):
        start = self.start
        list_shortest_path = []
        for vertex in self.shortest_path:
            neighbor = (start, vertex)
            list_shortest_path.append(neighbor)
            start = vertex
        return list_shortest_path

    def get_list_weights_edge(self):
        list_weights_edge = {}
        for position in self.graph.keys():
            for vertex, weight in self.graph.get(position).iteritems():
                if not(list_weights_edge.get((vertex, position))):
                    list_weights_edge[(position, vertex)] = weight
        return list_weights_edge


dij = None


class Path:
    def __init__(self):
        self.id = -1
        self.reverse = False
        self.pre_node = -1
        self.next_node = -1
        self.length = -1
        self.type = "lane|cross"  # trajectory type
        self.poses = []  # [geometry_msgs/Pose]


"""
traj.csv structure
    traj_id
    length  #
    reverse  # 1:true 0:false
    preid    # int
    nextid
    points[x,y,z,ox,oy,ox,ow]
"""


def ARROW():  # return: visualization_msgs::Marker
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time().now()
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.5
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    # set marker.id
    # set marker.pose
    # marker_start.lifetime = ros: : Duration();
    return marker


def START_POSE(pose):  # (geometry_msgs/Pose)
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time().now()
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.2
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    # set marker.id
    # set marker.pose
    # marker_start.lifetime = ros: : Duration();
    marker.pose = pose
    return marker


def TARGET_POSE(pose):  # (geometry_msgs/Pose)
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time().now()
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    # set marker.id
    # set marker.pose
    # marker_start.lifetime = ros: : Duration();
    marker.pose = pose
    return marker


def __insert_path_to_default_path(path):
    global default_path, cnt
    pre_pose = None
    for pose in path.poses:
        if pre_pose is None or __distance_of_two_point(pre_pose.position, pose.position) > 1.0:
            arrow = ARROW()
            arrow.header.frame_id = "map"
            arrow.pose = pose
            arrow.color.r = 1.0
            arrow.color.g = 1.0  # color of default path
            arrow.scale.x = 0.6
            arrow.scale.y = 0.1
            arrow.scale.z = 0.1
            arrow.id = cnt
            cnt += 1
            default_path.markers.append(arrow)
            pre_pose = pose

def __insert_path_to_default_nav_path(path):
    global default_path_nav, cnt
    default_path_nav.header.frame_id = "map"
    default_path_nav.header.stamp = rospy.Time.now()
    for pose in path.poses:
        p = PoseStamped()
        p.pose = pose
        cnt += 1
        default_path_nav.poses.append(p)

def __init_data_base_lidar(dir):
    rospy.loginfo("[global planning] Creating global pathes (Lidar), traj dir={}".format(dir))
    global max_node_id, max_path_id
    global origin_data, graph, pure_data
    if not os.path.exists(dir):
        rospy.logerr("data_base_folder:{} is empty".format(dir))
        return False
    file_names = os.listdir(dir)
    for file_name in file_names:
        rospy.loginfo("[global planning] Reading lidar path {}".format(file_name))
        file = os.path.join(dir, file_name)
        with open(file, 'r') as f:
            csv_reader = csv.reader(f)
            path = Path()
            t = list(csv_reader)
            path.id = int(t[0][0])
            path.length = float(t[1][0])
            path.reverse = bool(t[2][0])
            path.pre_node = int(t[3][0])
            path.next_node = int(t[4][0])
            if file_name.startswith("lane"):
                path.type = "lane"
            else:
                path.type = "cross"
            point_num = len(t)
            for i in range(5, point_num):
                _current_pose = t[i]
                p = Pose()
                p.position.x = float(_current_pose[0])
                p.position.y = float(_current_pose[1])
                # p.position.z = float(_current_pose[2])
                if ( i + 1 < point_num ):
                    next_x = float(t[i+1][0])
                    next_y = float(t[i+1][1])
                    q = tf.transformations.quaternion_from_euler(0, 0, math.atan2(next_y-p.position.y, next_x-p.position.x))
                    p.orientation.x = q[0]
                    p.orientation.y = q[1]
                    p.orientation.z = q[2]
                    p.orientation.w = q[3]
                elif ( i - 1 >= 0):
                    # 最后一个点,方向和前一个一致
                    p.orientation = path.poses[-1].orientation
                path.poses.append(p)
            origin_data[(path.pre_node, path.next_node)] = path
            pure_data[(path.pre_node, path.next_node)] = path

            # if this path is reversable, create a new one with different direction
            if path.reverse:
                rpath = copy.deepcopy(path)
                rpath.pre_node, rpath.next_node = path.next_node, path.pre_node
                rpath.poses.reverse()
                origin_data[(rpath.pre_node, rpath.next_node)] = rpath
            # update max_node_id and max_path_id
            max_node_id = max(max_node_id, path.pre_node)
            max_node_id = max(max_node_id, path.next_node)
            max_path_id = max(max_path_id, path.id)
            __insert_path_to_default_path(path)
            # __insert_path_to_default_nav_path(path)
    time.sleep(2.0)
    global default_path
    pub_default_path.publish(default_path)
    # pub_default_nav_path.publish(default_path_nav)
    return True

def __init_data_base_gps(dir):
    rospy.loginfo("[global planning] Creating global pathes (RTK-GPS), traj dir={}".format(dir))
    global max_node_id, max_path_id
    global origin_data, graph, pure_data
    if not os.path.exists(dir):
        rospy.logerr("data_base_folder:{} is empty".format(dir))
        return False
    file_names = os.listdir(dir)
    for file_name in file_names:
        rospy.loginfo("[global planning] Reading gps path {}".format(file_name))
        # 1. 读取路点
        file = os.path.join(dir, file_name)
        with open(file, 'r') as f:
            csv_reader = csv.reader(f)
            path = Path()
            t = list(csv_reader)
            path.id = int(t[0][0])
            path.length = float(t[1][0])
            path.reverse = bool(t[2][0])
            path.pre_node = int(t[3][0])
            path.next_node = int(t[4][0])
            if file_name.startswith("lane"):
                path.type = "lane"
            else:
                path.type = "cross"
            point_num = len(t)
            # srv_msg = TransPointsRequest()
            # srv_msg.type = TransPointsResponse.TYPE_GPS2LLA
            # for i in range(5, point_num):
            #     _current_pose = t[i]
            #     p = NavSatFix()
            #     p.latitude = float(_current_pose[0])
            #     p.longitude = float(_current_pose[1])
            #     srv_msg.gps_points.append(p)
            for i in range(5, point_num):
                _current_pose = t[i]
                p = Pose()
                p.position.x = float(_current_pose[0])
                p.position.y = float(_current_pose[1])
                # p.position.z = float(_current_pose[2])
                if ( i + 1 < point_num ):
                    next_x = float(t[i+1][0])
                    next_y = float(t[i+1][1])
                    q = tf.transformations.quaternion_from_euler(0, 0, math.atan2(next_y-p.position.y, next_x-p.position.x))
                    p.orientation.x = q[0]
                    p.orientation.y = q[1]
                    p.orientation.z = q[2]
                    p.orientation.w = q[3]
                elif ( i - 1 >= 0):
                    # 最后一个点,方向和前一个一致
                    p.orientation = path.poses[-1].orientation
                path.poses.append(p)

        # try:
        #     rospy.wait_for_service("/gps2utm/trans_points")
        #     res = points_trans_client.call(srv_msg)
        # except Exception as e:
        #     rospy.logerr("[Global planning] Call service GPS points to LLA points failed! e: {}".format(e))
        #     return

        # # 1. GPS坐标转本地坐标
        # rospy.loginfo("[global planning] Transing {} points from gps to lla".format(len(res.res_lla_points)))
        # _size = len(res.res_lla_points)
        # for i in range(0, _size):
        #     p = Pose()
        #     p.position.x = float(res.res_lla_points[i].x)
        #     p.position.y = float(res.res_lla_points[i].y)
        #     # p.position.z = float(_current_pose[2])
        #     if ( i + 1 < _size ):
        #         next_x = res.res_lla_points[i+1].x
        #         next_y = res.res_lla_points[i+1].y
        #         q = tf.transformations.quaternion_from_euler(0, 0, math.atan2(next_y-p.position.y, next_x-p.position.x))
        #         p.orientation.x = q[0]
        #         p.orientation.y = q[1]
        #         p.orientation.z = q[2]
        #         p.orientation.w = q[3]
        #     elif ( i - 1 >= 0):
        #         # 最后一个点,方向和前一个一致
        #         p.orientation = path.poses[-1].orientation
        #     path.poses.append(p)

        # 2. 添加到全局路径点中
        origin_data[(path.pre_node, path.next_node)] = path
        pure_data[(path.pre_node, path.next_node)] = path

        # if this path is reversable, create a new one with different direction
        if path.reverse:
            rpath = copy.deepcopy(path)
            rpath.pre_node, rpath.next_node = path.next_node, path.pre_node
            rpath.poses.reverse()
            origin_data[(rpath.pre_node, rpath.next_node)] = rpath
        # update max_node_id and max_path_id
        max_node_id = max(max_node_id, path.pre_node)
        max_node_id = max(max_node_id, path.next_node)
        max_path_id = max(max_path_id, path.id)
        __insert_path_to_default_path(path)
        # __insert_path_to_default_nav_path(path)

    time.sleep(2.0)
    global default_path
    pub_default_path.publish(default_path)
    # pub_default_nav_path.publish(default_path_nav)
    return True

def __init_graph():
    global origin_data, graph
    for key, path in origin_data.items():
        path_id = path.id
        path_pre_node = path.pre_node
        path_next_node = path.next_node
        path_length = path.length
        if path_pre_node not in graph.keys():
            graph[path_pre_node] = {}
        graph[path_pre_node][path_next_node] = path_length
    # display its connection and length
    # for pre_id, data in graph.items():
    #     print(pre_id, data)
    return True


def __display_graph():
    # TODO::Use networkx and ros::subscribe to display graph
    pass


"""
workflow:
1. read data files and create origin datas
    origin data is dict{(node_st,node_end): Path}
2. create Graph based on origin data
    2.1 create dict, refer is:
        #    graph = {
        #         1: { 2: 1, 4: 3 },
        #         2: { 1: 1, 4: 1, 3: 5 },
        #         3: { 2: 5, 5: 3, 6: 3 },
        #         4: { 1: 3, 2: 1, 5: 1 },
        #         5: { 4: 1, 3: 3, 6: 7 },
        #         6: { 3: 3, 5: 7 },
        #         }
        record maximum node id = node_maxium_id
3. sub current_pose and target_pose
    3.1 receive current_pose:
        update current position value
    3.2 receive target_pose
        3.2.1 deepcopy graph to tmp_graph
        3.2.2 search the nearest point of current_pose
                record: (start_node_id, next_node_id), nearest_point index
                compare if current_pose and nearest point is the same direction
                if not:
                    swap (start_node_id, next_node_id)
                    refind the nearest point index
                create new node and add ${node_maxium_id+1}:{} to tmp_graph
                create new node-id ${node_maxium_id+1},next=next_node_id, and its path--current point_index to end, and reverse=false
                add new node-id and its length to next_node_id
                from start_node_id remove next_node_id
                from next_node_id remove start_node_id
        3.2.3 search the nearest point of target pose  (for target pose, the graph defines it direction so its handling is a little different)
                record: (start_node_id, next_node_id), nearest_point index
                add ${node_maxium_id+1}:{} to tmp_graph
                create new node-id ${node_maxium_id+2},
                create path1: (start_node_id, ${node_maxium_id+2}), points(0:point_index), reverse=false
                add new node-id and its length to start_node_id
                if (next_node_id, start_node_id) exists  --this incluence the new path's reverse property
                    create path2: (next_node_id, ${node_maxium_id+2}), points(end:point_index), reverse=false
                    add new node-id and its length to next_node_id
                from start_node_id remove next_node_id
                from next_node_id remove start_node_id
        3.2.4 dij
        3.2.5 concat trajectory based on shortest path
        3.2.6 optimize and pub

"""


def _initial_pose_callback(msg):
    # PoseWithCovarianceStamped
    global _current_pose_initialized, _current_pose_direction_initialized
    rospy.loginfo("[global_planning] received /initial_pose")
    current_pose.pose = msg.pose.pose
    _current_pose_initialized = True
    _current_pose_direction_initialized = True
    msg_start_pose = START_POSE(msg.pose.pose)
    pub_start_pose.publish(msg_start_pose)


def _current_pose_callback(msg):
    global current_pose, _current_pose_initialized, _current_pose_direction_initialized
    current_pose = msg
    _current_pose_initialized = True
    _current_pose_direction_initialized = True

def __distance_of_two_point(p1, p2):
    return math.sqrt(math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2))


def __find_nearest(p):  # find the nearst point (index), return it's node pair (pre_node_id, next_node_id) and index, and min_dist
    global tmp_pure_data
    res_node_pair = (-1, -1)
    res_index = -1
    min_dist = float('inf')
    for path in tmp_pure_data.values():
        for index, pose in enumerate(path.poses):
            dist = __distance_of_two_point(p.pose.position, pose.position)
            if dist < min_dist:
                min_dist = dist
                res_index = index
                res_node_pair = (path.pre_node, path.next_node)

    if res_index == 0:  # nearest point cannot be the first or last point
        res_index += 1
    if res_index == len(tmp_pure_data[res_node_pair].poses)-1:
        res_index -= 1
    return res_node_pair, res_index, min_dist

# def quaternion2euler(quaternion):
#     r = R.from_quat(quaternion)
#     euler = r.as_euler('xyz', degrees=True)
#     return euler

# def euler2quaternion(euler):
#     r = R.from_euler('xyz', euler, degrees=True)
#     quaternion = r.as_quat()
#     return quaternion


def _get_pose_list_length(poses):
    length = 0.0
    pre_pose = poses[0]
    for pose in poses:
        length += __distance_of_two_point(pre_pose.position, pose.position)
        pre_pose = pose
    return length

def is_same_direction(yaw1, yaw2):
    diff = yaw1 - yaw2
    while not ( 0 <= diff <= 2*math.pi ):
        if diff < 0:
            diff += 2*math.pi
        else:
            diff -= 2*math.pi
    if 0.5 * math.pi < diff and diff < 1.5 * math.pi:
        return False
    return True

def __handle_start_pose():
    global tmp_data, tmp_graph, current_pose, distance_start_pose_to_path, min_distance_to_path
    node_pair, nearest_point_index, distance_start_pose_to_path = __find_nearest(current_pose)
    if distance_start_pose_to_path > min_distance_to_path:
        rospy.logwarn("[global_planning] min distance of current pose to path is {} > {}, can't conduct global planing, please get closer".format(distance_start_pose_to_path, min_distance_to_path))
        return False
    tmp_pure_data.pop(node_pair)  # delete current nearest path, and new two path will be added later
    # check if current position and nearest point's position is the same
    st_nearest_pose = tmp_data[node_pair].poses[nearest_point_index]
    # (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    # self.fill_euler_msg(msg, r, p, y)
    c4 = [current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w]
    n4 = [st_nearest_pose.orientation.x, st_nearest_pose.orientation.y, st_nearest_pose.orientation.z, st_nearest_pose.orientation.w]
    euler_current_pose = tf.transformations.euler_from_quaternion(c4)
    euler_nearest_pose = tf.transformations.euler_from_quaternion(n4)
    # if not math.fabs(euler_current_pose[2] - euler_nearest_pose[2]) < math.pi/2.0:  # not the same direction
    if not is_same_direction(euler_current_pose[2], euler_nearest_pose[2]):
        rospy.loginfo("[global_planning] start direction is not the same with current path({},{})".format(*node_pair))
        if not tmp_data[node_pair].reverse:  # current path is not reverse
            rospy.logerr("[global_planning] current path is not reversiable, please change car's direction")
            return False
        node_pair = (node_pair[1], node_pair[0])
        nearest_point_index = len(tmp_data[node_pair].poses) - nearest_point_index
    # cut current path, create new node, update tmp_data and tmp_graph
    # pre_node_id -> st_node_id -> next_node_id
    st_node_id = max_node_id + 1
    pre_node_id = node_pair[0]
    next_node_id = node_pair[1]
    # create pre path: path1
    path1 = Path()
    path1.id = max_path_id + 1
    path1.pre_node = pre_node_id
    path1.next_node = st_node_id
    path1.type = tmp_data[node_pair].type
    path1.reverse = False
    path1.poses = tmp_data[node_pair].poses[:nearest_point_index]
    path1.length = _get_pose_list_length(path1.poses)
    # create next path: path2
    path2 = Path()
    path2.id = max_path_id + 2
    path2.pre_node = st_node_id
    path2.next_node = next_node_id
    path2.type = tmp_data[node_pair].type
    path2.reverse = False
    path2.poses = tmp_data[node_pair].poses[nearest_point_index:]
    path2.length = _get_pose_list_length(path2.poses)
    # update tmp_data and tmp_graph
    if_path_reverse = tmp_data[node_pair].reverse

    # add
    tmp_data[(pre_node_id, st_node_id)] = path1
    tmp_data[(st_node_id, next_node_id)] = path2
    tmp_pure_data[(pre_node_id, st_node_id)] = path1
    tmp_pure_data[(st_node_id, next_node_id)] = path2

    tmp_graph[pre_node_id][st_node_id] = path1.length
    tmp_graph[st_node_id] = {}
    tmp_graph[st_node_id][next_node_id] = path2.length

    # del
    tmp_data.pop(node_pair)
    tmp_graph[pre_node_id].pop(next_node_id)
    if if_path_reverse:
        tmp_data.pop((node_pair[1], node_pair[0]))
        tmp_graph[next_node_id].pop(pre_node_id)
    return True


def __handle_target_pose():
    global min_distance_to_path
    global tmp_data, tmp_graph, target_pose, distance_target_pose_to_path
    node_pair, nearest_point_index,  distance_target_pose_to_path = __find_nearest(target_pose)
    if distance_target_pose_to_path > min_distance_to_path:
        rospy.logwarn("[global_planning] min distance of target pose to path is {} > {}, can't conduct global planing, please get closer".format(distance_target_pose_to_path, min_distance_to_path))
        return False
    # check if target position and nearest point's position is the same
    st_nearest_pose = tmp_data[node_pair].poses[nearest_point_index]
    # euler_target_pose = tf.transformations.euler_from_quaternion(target_pose.pose.orientation)
    # euler_nearest_pose = tf.transformations.euler_from_quaternion(st_nearest_pose.pose.orientation)
    if_path_reverse = tmp_data[node_pair].reverse

    # if not math.fabs(euler_target_pose[2] - euler_nearest_pose[2]) < math.pi/2.0:  # not the same direction
    #     rospy.loginfo("[global_planning] target direction is not the same with current path({},{})".format(*node_pair))
    #     if not tmp_data[node_pair].reverse:  # current path is not reverse
    #         rospy.logerr("[global_planning] current path is not reversiable, please change target's direction")
    #         return None
    #     node_pair = (node_pair[1], node_pair[0])
    #     nearest_point_index = len(tmp_data[node_pair].poses) - nearest_point_index
    # cut current path, create new node, update tmp_data and tmp_graph
    # pre_node_id -> st_node_id -> next_node_id
    st_node_id = max_node_id + 2
    pre_node_id = node_pair[0]
    next_node_id = node_pair[1]
    # create pre path: path1
    path3 = Path()
    path3.id = max_path_id + 3
    path3.pre_node = pre_node_id
    path3.next_node = st_node_id
    path3.type = tmp_data[node_pair].type
    path3.reverse = False
    path3.poses = tmp_data[node_pair].poses[:nearest_point_index]
    path3.length = _get_pose_list_length(path3.poses)

    # create next path: path2
    if if_path_reverse:
        path4 = Path()
        path4.id = max_path_id + 4
        path4.pre_node = next_node_id
        path4.next_node = st_node_id
        path4.type = tmp_data[node_pair].type
        path4.reverse = False
        path4.poses = tmp_data[node_pair].poses[nearest_point_index:]
        path4.poses.reverse()
        path4.length = _get_pose_list_length(path4.poses)

        tmp_data.pop((node_pair[1], node_pair[0]))
        tmp_graph[next_node_id].pop(pre_node_id)
        tmp_data[(path4.pre_node, path4.next_node)] = path4
        tmp_graph[path4.pre_node][path4.next_node] = path4.length
    tmp_graph[max_node_id+2] = {}

    tmp_data.pop(node_pair)
    tmp_graph[pre_node_id].pop(next_node_id)
    tmp_data[(path3.pre_node, path3.next_node)] = path3
    tmp_graph[path3.pre_node][path3.next_node] = path3.length

    return True


def _link_start_end(global_path):
    global current_pose, target_pose
    global smooth_st_rate
    global _dist_point_step
    tmp_poses = []  # [PoseStamped(),...]
    # tmp_poses.append(current_pose)
    # link start
    global distance_st_smooth_limit_min, distance_st_sommth_limit_max
    global distance_start_pose_to_path
    d = distance_start_pose_to_path * smooth_st_rate
    d = min(d, distance_st_sommth_limit_max)
    d = max(d, distance_st_smooth_limit_min)
    front_index = 0
    while __distance_of_two_point(current_pose.pose.position, global_path.poses[front_index].pose.position) < d:
        front_index += 1
        if front_index >= len(global_path.poses):
            break
    if front_index < len(global_path.poses):
        steps = int(d / _dist_point_step)
        diff_x = (global_path.poses[front_index].pose.position.x - current_pose.pose.position.x) / steps
        diff_y = (global_path.poses[front_index].pose.position.y - current_pose.pose.position.y) / steps
        diff_z = (global_path.poses[front_index].pose.position.z - current_pose.pose.position.z) / steps
        print(steps, diff_x, diff_y, diff_z)
        # pre_pose = current_pose.pose
        for i in range(steps):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = current_pose.pose.position.x + diff_x * i
            pose.pose.position.y = current_pose.pose.position.y + diff_y * i
            pose.pose.position.z = current_pose.pose.position.z + diff_z * i
            tmp_poses.append(pose)

    # link end
    global distance_end_smooth_limit_min, distance_end_smooth_limit_max
    global distance_target_pose_to_path, smooth_end_rate
    d = distance_target_pose_to_path * smooth_end_rate
    d = min(d, distance_end_smooth_limit_max)
    d = max(d, distance_end_smooth_limit_min)
    end_index = len(global_path.poses) - 1
    while __distance_of_two_point(target_pose.pose.position, global_path.poses[end_index].pose.position) < d:
        end_index -= 1
        if end_index <= front_index:
            break
    # add global_path.poses[front_index:end_indx] to tmp_poses
    tmp_poses.extend(global_path.poses[front_index:end_index])

    steps = int(d / _dist_point_step)
    diff_x = (target_pose.pose.position.x - global_path.poses[end_index].pose.position.x) / steps
    diff_y = (target_pose.pose.position.y - global_path.poses[end_index].pose.position.y) / steps
    diff_z = (target_pose.pose.position.z - global_path.poses[end_index].pose.position.z) / steps
    # pre_pose = global_path.poses[end_index]
    for i in range(1, steps):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = global_path.poses[end_index].pose.position.x + diff_x * i
        pose.pose.position.y = global_path.poses[end_index].pose.position.y + diff_y * i
        pose.pose.position.z = global_path.poses[end_index].pose.position.z + diff_z * i
        tmp_poses.append(pose)
    tmp_poses.append(target_pose)
    global_path.poses = tmp_poses
    return global_path


def _smooth(global_path):
    # TODO::if needed, add smooth method here
    return global_path


def __planning():
    global tmp_data, tmp_graph, tmp_pure_data, dij
    tmp_data = copy.deepcopy(origin_data)
    tmp_graph = copy.deepcopy(graph)
    tmp_pure_data = copy.deepcopy(pure_data)
    if not __handle_start_pose():
        rospy.loginfo("[global_planning] handling start pose failed")
        return None
    if not __handle_target_pose():
        rospy.loginfo("[global_planning] handling target pose failed")
        return None
    # do dijkstra
    try:
        dij = Dijkstra(tmp_graph, max_node_id+1, max_node_id+2)
        dij.calculate_preds()
        dij.calculate_shortest_path()
        print("Preds   : %s" % (dij.Preds))
        print("Weights : %s" % (dij.weights))
        print("Shortest path : %s" % (dij.shortest_path))
    except Exception as e:
        rospy.logerr("[global_planning] Check planing log, error: {}".format(e))
        return None

    # concat path
    global_path = rosPath()
    global_path.header.frame_id = "map"
    for i in range(1, len(dij.shortest_path)):
        path = tmp_data[(dij.shortest_path[i-1], dij.shortest_path[i])]
        for pose in path.poses:
            point = PoseStamped()
            point.header.frame_id = "map"
            point.pose = pose
            global_path.poses.append(point)

    # link start_pose and target pose
    global_path = _link_start_end(global_path)

    # smooth --optional
    global_path = _smooth(global_path)

    return global_path


def _target_pose_callback(msg):
    rospy.loginfo("[global_planning] received target pose")
    global _current_pose_initialized, _current_pose_direction_initialized
    global current_pose, target_pose
    target_pose = msg
    msg_start_pose = START_POSE(current_pose.pose)
    pub_start_pose.publish(msg_start_pose)
    msg_target_pose = TARGET_POSE(target_pose.pose)
    pub_target_pose.publish(msg_target_pose)

    rospy.loginfo("[global_planning] check planning environment")
    if not _pathes_initialized:
        rospy.logerr("[global_planning] Trajectories not initialized, can't do global planning")
        return
    if not _current_pose_initialized:
        rospy.logerr("[global_planning] current_pose not initialized, can't do global planning")
        return
    if not _current_pose_direction_initialized:
        rospy.logerr("[global_planning] current direction not initialized, can't do global planning. If you are using GPS, please set the initial pose with correct direction in RVIZ.")
        return
    rospy.loginfo("[global_planning] start planning process")
    path = __planning()

    if path is None:
        rospy.loginfo("[global_planning] planning process failed.")
        return

    rospy.loginfo("[global_planning] publish global path")
    pub_result_path.publish(path)

    # create smartcar_msgs/Lane
    resLane = Lane()
    resLane.header.frame_id = "map"
    resLane.header.stamp = rospy.Time().now()
    # IMPORTANT:here we ignore its type(lane|cross), and use real twist|yaw to control speed
    for pst in path.poses:
        waypoint = Waypoint()
        waypoint.speed_limit = 20
        waypoint.is_lane = 1
        waypoint.lane_id = 1
        waypoint.pose = pst
        resLane.waypoints.append(waypoint)
    pub_global_path.publish(resLane)

def _cb_service_set_target(req):
    assert type(req) is lbsrv_set_targetRequest
    rospy.loginfo("[global_planning] received target pose request")
    global _current_pose_initialized, _current_pose_direction_initialized
    global current_pose, target_pose
    target_pose = PoseStamped()
    target_pose.pose.position.x = req.target_x
    target_pose.pose.position.y = req.target_y

    msg_start_pose = START_POSE(current_pose.pose)
    pub_start_pose.publish(msg_start_pose)
    msg_target_pose = TARGET_POSE(target_pose.pose)
    pub_target_pose.publish(msg_target_pose)

    res = lbsrv_set_targetResponse()

    rospy.loginfo("[global_planning] check planning environment")
    if not _pathes_initialized:
        _err_msg = "[global_planning] Trajectories not initialized, can't do global planning"
        rospy.logwarn(_err_msg)
        res.result = False
        res.error_code = lb_error_code.ERROR_UNDEFINED
        res.error_msg = _err_msg
        return res

    if not _current_pose_initialized:
        _err_msg = "[global_planning] current_pose not initialized, can't do global planning"
        rospy.logwarn(_err_msg)
        res.result = False
        res.error_code = lb_error_code.ERROR_UNDEFINED
        res.error_msg = _err_msg
        return res

    if not _current_pose_direction_initialized:
        _err_msg = "[global_planning] current direction not initialized, can't do global planning."
        rospy.logwarn(_err_msg)
        res.result = False
        res.error_code = lb_error_code.ERROR_UNDEFINED
        res.error_msg = _err_msg
        return res

    rospy.loginfo("[global_planning] start planning process")
    try:
        path = __planning()
    except Exception as e:
        rospy.logwarn("Error while do global planning, err: {}".format(e))
        res.result = False
        res.error_code = lb_error_code.ERROR_UNDEFINED
        res.error_msg = _err_msg
        return res

    if path is None:
        _err_msg = "[global_planning] planning process failed, return NaN path"
        rospy.loginfo(_err_msg)
        res.result = False
        res.error_code = lb_error_code.ERROR_UNDEFINED
        res.error_msg = _err_msg
        return res

    rospy.loginfo("[global_planning] publish global path")
    pub_result_path.publish(path)

    # create smartcar_msgs/Lane
    resLane = Lane()
    resLane.header.frame_id = "map"
    resLane.header.stamp = rospy.Time().now()
    # IMPORTANT:here we ignore its type(lane|cross), and use real twist|yaw to control speed
    for pst in path.poses:
        res.traj.append(pst.pose.position)
        waypoint = Waypoint()
        waypoint.speed_limit = 20
        waypoint.is_lane = 1
        waypoint.lane_id = 1
        waypoint.pose = pst
        resLane.waypoints.append(waypoint)
    pub_global_path.publish(resLane)

    res.result = True
    return res

def _show_graph(msg):
    pass
#     if msg.show_graph:
#         global dij
#         if dij is not None:
#             rospy.loginfo("[global_planning] show graph")
#             dij.show_graph()

def _cb_config_traj_set(msg):
    global PILOT_MODE
    PILOT_MODE = msg.pilot_mode
    dir_trajs = msg.path_traj_path

    # 清除当前缓存的路径点数据
    global origin_data, pure_data
    origin_data.clear()
    pure_data.clear()

    ret = True
    if PILOT_MODE == ConfigTrajSet.PILOT_LIDAR:
        ret = __init_data_base_lidar(dir_trajs)
    elif PILOT_MODE == ConfigTrajSet.PILOT_RTKGPS:
        ret = __init_data_base_gps(dir_trajs)
    else:
        rospy.logerr("[global planning] 无法识别当前的导航模式, code={}".format(PILOT_MODE))
        return
    
    if ret == False:
        rospy.logerr("[global planning] 无法读取路径文件, 当前导航模式为 code = {} (0=Lidar, 1=RTK-GPS)!".format(PILOT_MODE))
        return

    if not __init_graph():
        rospy.logerr("[global planning] 创建路网数据失败!")
        return

    global _pathes_initialized
    _pathes_initialized = True

def _cb_service_function_switch(req):
    global _FUNCTION_STATUS
    if ( req.switch_to != _FUNCTION_STATUS ):
        global sub_initpose, sub_current_pose, sub_target_pose, sub_showgraph, sub_config, _FUNCTION_STATUS

        if ( req.switch_to == FunctionSwitchResponse.FUNCTION_ENABLE ):
            _FUNCTION_STATUS = FunctionSwitchResponse.FUNCTION_ENABLE
            sub_initpose = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, _initial_pose_callback, queue_size=5)
            sub_current_pose = rospy.Subscriber("/current_pose", PoseStamped, _current_pose_callback, queue_size=10)
            sub_target_pose = rospy.Subscriber("/move_base_simple/goal", PoseStamped, _target_pose_callback)
            sub_showgraph = rospy.Subscriber("/show_graph", ShowGraph, _show_graph, queue_size=1)
            sub_config = rospy.Subscriber("/config_traj_path", ConfigTrajSet, _cb_config_traj_set, queue_size=2)
            rospy.loginfo("[global planning] Function ON")
        elif ( req.switch_to == FunctionSwitchResponse.FUNCTION_DISABLE ):
            _FUNCTION_STATUS = FunctionSwitchResponse.FUNCTION_DISABLE
            sub_initpose.unregister()
            sub_current_pose.unregister()
            sub_target_pose.unregister()
            sub_showgraph.unregister()
            sub_config.unregister()
            rospy.loginfo("[global planning] Function OFF")
        else:
            rospy.logwarn("[global planning] Unrecognized FunctionSwitch command")
            pass
    res = FunctionSwitchResponse()
    res.current_function_status = _FUNCTION_STATUS
    return res

def main():
    # add sub and spin
    global sub_initpose, sub_current_pose, sub_target_pose, sub_showgraph, sub_config
    sub_initpose = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, _initial_pose_callback, queue_size=5)
    sub_current_pose = rospy.Subscriber("/current_pose", PoseStamped, _current_pose_callback, queue_size=10)
    sub_target_pose = rospy.Subscriber("/move_base_simple/goal", PoseStamped, _target_pose_callback)
    sub_showgraph = rospy.Subscriber("/show_graph", ShowGraph, _show_graph, queue_size=1)
    sub_config = rospy.Subscriber("/config_traj_path", ConfigTrajSet, _cb_config_traj_set, queue_size=2)

    rospy.Service("/function_switch/global_planning", FunctionSwitch, _cb_service_function_switch)
    rospy.Service("/lb_cloud_service/set_target", lbsrv_set_target, _cb_service_set_target)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("global_planning", log_level=rospy.INFO)
    main()


# if __name__ == '__main__':
#    print("Exemplo 1 - Graph")
#    graph = {
#         1: { 2: 1, 4: 3 },
#         2: { 1: 1, 4: 1, 3: 5 },
#         3: { 2: 5, 5: 3, 6: 3 },
#         4: { 1: 3, 2: 1, 5: 1 },
#         5: { 4: 1, 3: 3, 6: 7 },
#         # 6: { 3: 3, 5: 7 },
#         6:{}
#         }


#    dijkstra = Dijkstra(graph,1,6)
#    dijkstra.calculate_preds()
#    dijkstra.calculate_shortest_path()
#    print("Preds   : %s" %(dijkstra.Preds))
#    print("Weights : %s" %(dijkstra.weights))
#    print("Shortest path : %s" %(dijkstra.shortest_path))
#    dijkstra.show_graph()
