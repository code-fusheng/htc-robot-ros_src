#!/usr/bin/python
# -*-coding:utf-8-*-

from telnetlib import NAOVTS
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
import csv
import time
import os
import getpass
from automotive_msgs.msg import SavePath, SaveStation, CmdPoseRecord, PathRecordingStatus
from nmea_navsat_driver.msg import NavSatGHFPD
import copy
import math
import numpy as np
from threading import Lock
# from apscheduler.schedulers.background import BackgroundScheduler
from htcbot_msgs.msg import MapPathConf, ModeSwitch

USERNAME = getpass.getuser()


class App:
    def __init__(self):
        rospy.init_node("pose_recorder", log_level=rospy.DEBUG)
        self.last_pose = None
        self.current_rtk_gps_msg = None
        self.path = Path()
        self.path.header.frame_id = "map"
        self._pilot_mode = 1
        self.is_record_start = False
        self.last_say_time = rospy.Time.now()

        # code-robot
        self.route_path_dir = None

        self.sub_pose = None
        self.sub_cmd_save_path = None
        self.sub_cmd_save_station = None
        self.sub_path_record_control = None
        self.sub_fused_gps = None
        
        self._rtk_pose_lock = Lock()

        self._init()

    def _init(self):
        self.is_filter = rospy.get_param("~filter", default=True)
        self.filter_distance = rospy.get_param("~filter_distance", default=0.1)

        # add pub&sub
        self._add_pub()
        self._add_sub()

    def _distance_of_two_pose(self, p1, p2):
        return math.sqrt(math.pow(p1.pose.position.x - p2.pose.position.x, 2) + math.pow(p1.pose.position.y - p2.pose.position.y, 2))

    def _add_pub(self):
        self.path_pub = rospy.Publisher("/path/recored_poses", Path, queue_size=5)
        self.pub_recording_status = rospy.Publisher("/path_recording_status", PathRecordingStatus, queue_size=5)
    
    def _add_sub(self):
        self.sub_cmd_save_station = rospy.Subscriber("/save_station", SaveStation, self._cb_save_station, queue_size=5)
        self.sub_path_record_control = rospy.Subscriber("/path_record_control", CmdPoseRecord, self._cb_cmd_pose_record, queue_size=5)
        self.sub_map_path_conf = rospy.Subscriber("/htcbot/map_path_conf", MapPathConf, self.handle_map_path_conf, queue_size=5)
        self.sub_mode_switch = rospy.Subscriber("/htcbot/mode_switch", ModeSwitch, self.handle_mode_switch, queue_size=5)
        
    def _enable_subscribe(self):
        self.sub_pose = rospy.Subscriber("/current_pose", PoseStamped, callback=self._cb_current_pose, queue_size=100)
        if self._pilot_mode == CmdPoseRecord.PILOT_RTKGPS:
            self.sub_fused_gps = rospy.Subscriber("/fused_fix", NavSatGHFPD, callback=self._cb_rtk_gps_fixed, queue_size=100)
    
    def _disable_subscribe(self):
        if self.sub_pose:
            self.sub_pose.unregister()
            self.sub_pose = None
        if self._pilot_mode == CmdPoseRecord.PILOT_RTKGPS and self.sub_fused_gps:
            self.sub_fused_gps.unregister()
            self.sub_fused_gps = None

    def _get_path_length(self, path):
        if len(path.poses) < 1:
            return 0
        length = 0.0
        pre = path.poses[0]
        for pose in path.poses:
            length += self._distance_of_two_pose(pre, pose)
            pre = pose
        return length
    
    def handle_map_path_conf(self, msg):
        rospy.loginfo("[pose_recorder] Recivced Map Path Conf ==> {}".format(msg.route_path))
        self.route_path_dir = msg.route_path

    def _cb_cmd_pose_record(self, msg):
        if msg.start_record is None:
            rospy.logwarn("[pose_recorder] Invalid /start_record_path msg, start_record is empty")
            return
        if self.is_record_start and msg.start_record == msg.RECORD_START:
            msg_status = PathRecordingStatus()
            msg_status.is_recording = True
            msg_status.info = "Path-recording is already started, if you want to record a new path, send /start_record_path=false first"
            self.pub_recording_status.publish(msg_status)
            rospy.logwarn("[pose_recorder] {}".format(msg_status.info))
            return
        
        if not self.is_record_start and msg.start_record == msg.RECORD_STOP:
            msg_status = PathRecordingStatus()
            msg_status.is_recording = False
            msg_status.info = "Path-recording is not running"
            self.pub_recording_status.publish(msg_status)
            rospy.logwarn("[pose_recorder] {}".format(msg_status.info))
            return

        if msg.start_record == msg.RECORD_START:
            # 记录的时候就要明确当前是记录什么数据, gps还是lidar
            self._pilot_mode = msg.pilot_mode

            self.is_record_start = True
            self.filter_distance = msg.step_size
            self._enable_subscribe()
            msg_status = PathRecordingStatus()
            msg_status.is_recording = True
            self.pub_recording_status.publish(msg_status)
            rospy.loginfo("[pose_recorder] Start recording")
        elif msg.start_record == msg.RECORD_STOP:
            self.is_record_start = False
            self._disable_subscribe()
            msg_status = PathRecordingStatus()
            msg_status.is_recording = False
            self.pub_recording_status.publish(msg_status)
            self._do_save_path(msg.save_path)
            self.reset()
            rospy.loginfo("[pose_recorder] Stop recording, recording path is empty now")
        else:
            rospy.logwarn("[pose_recorder] Unexpected error")

    def handle_mode_switch(self, msg):
        if msg.mode is not ModeSwitch.POSE_RECORD:
            rospy.loginfo("[pose_recorder ???] {}".format(msg.switch_to))
            return
        if self.is_record_start and msg.switch_to == msg.ON:
            msg_status = PathRecordingStatus()
            msg_status.is_recording = True
            msg_status.info = "Path-recording is already started, if you want to record a new path, send /start_record_path=false first"
            self.pub_recording_status.publish(msg_status)
            rospy.logwarn("[pose_recorder] {}".format(msg_status.info))
            return
        if not self.is_record_start and msg.switch_to == msg.OFF:
            msg_status = PathRecordingStatus()
            msg_status.is_recording = False
            msg_status.info = "Path-recording is not running"
            self.pub_recording_status.publish(msg_status)
            rospy.logwarn("[pose_recorder] {}".format(msg_status.info))
            return
        if msg.start_record == msg.RECORD_START:
            # 记录的时候就要明确当前是记录什么数据, gps还是lidar
            self.is_record_start = True
            self._enable_subscribe()
            msg_status = PathRecordingStatus()
            msg_status.is_recording = True
            self.pub_recording_status.publish(msg_status)
            rospy.loginfo("[pose_recorder] Start recording")
        elif msg.start_record == msg.RECORD_STOP:
            self.is_record_start = False
            self._disable_subscribe()
            msg_status = PathRecordingStatus()
            msg_status.is_recording = False
            self.pub_recording_status.publish(msg_status)
            self._do_save_path(self.route_path_dir)
            self.reset()
            rospy.loginfo("[pose_recorder] Stop recording, recording path is empty now")
        else:
            rospy.logwarn("[pose_recorder] Unexpected error")

    def _do_save_path(self, path_dir):
        # now = time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime(time.time()))
        filename = "lane_1.csv"
        rospy.logerr("saveing {} points".format(len(self.path.poses)))
        self._save_path_new(os.path.join(path_dir, filename), self.path)
        self.path.poses = []

    def _save_path_new(self, filename, path):
        if path == None or len(path.poses) == 0:
            rospy.logwarn("[pose recorder] Cannot save the path, because the path size is 0")
            return

        with open(filename, 'w') as f:
            writer = csv.writer(f)
            path_length = self._get_path_length(path)
            # write trajectory header
            writer.writerow([1])  # id
            writer.writerow([float(path_length)])  # length
            writer.writerow([1])  # reverse
            writer.writerow([1])  # pre_node_ids
            writer.writerow([2])  # next_node_ids
            for pst in path.poses:
                pose = [
                    float(pst.pose.position.x,),
                    float(pst.pose.position.y),
                    float(pst.pose.position.z),
                    float(pst.pose.orientation.x),
                    float(pst.pose.orientation.y),
                    float(pst.pose.orientation.z),
                    float(pst.pose.orientation.w)
                ]
                writer.writerow(pose)
            # if self._pilot_mode == CmdPoseRecord.PILOT_LIDAR:
            #     # write pose
            #     for pst in path.poses:
            #         pose = [
            #             float(pst.pose.position.x,),
            #             float(pst.pose.position.y),
            #             float(pst.pose.position.z),
            #             float(pst.pose.orientation.x),
            #             float(pst.pose.orientation.y),
            #             float(pst.pose.orientation.z),
            #             float(pst.pose.orientation.w)
            #         ]
            #         writer.writerow(pose)
            # elif self._pilot_mode == CmdPoseRecord.PILOT_RTKGPS:
            #     # write pose
            #     for pst in path.poses:
            #         pose = [
            #             float(pst.pose.position.x),  # latitude
            #             float(pst.pose.position.y),   # longitude
            #         ]
            #         writer.writerow(pose)
        rospy.loginfo("[pose recording] Saved {} points to {}".format(len(self.path.poses), filename))

    def _save_path(self, file_name, path, msg):
        file_res = msg.file_res
        if len(path.poses) == 0:
            # rospy.loginfo("[pose_recorder] {} is empty".format(file_name))
            return
        tmp_path = copy.deepcopy(path)
        if file_res > 0.01:
            pre = 0
            for i in range(1, len(path.poses)):
                if self._distance_of_two_pose(tmp_path.poses[pre], path.poses[i]) >= file_res:
                    pre += 1
                    tmp_path.poses[pre] = tmp_path.poses[i]
            tmp_path.poses = tmp_path.poses[:pre]

        with open(file_name, 'w') as f:
            writer = csv.writer(f)
            # tmp_Path = copy.deepcopy(path)
            # modify orientation
            tmp_path = self._modify_path_orientation(tmp_path)
            # do write back
            self._do_write(writer, tmp_path, msg, file_name)
        rospy.loginfo("[pose recorder] path saved to {}".format(file_name))

    def _do_write(self, writer, tmp_path, msg, filename):
        path_length = self._get_path_length(tmp_path)
        # write trajectory header
        writer.writerow(str(msg.id))  # id
        writer.writerow(str(int(msg.reverse)))  # reverse
        writer.writerow([float(path_length)])  # length
        writer.writerow(str(msg.pre_node_id))  # pre_node_ids
        writer.writerow(str(msg.next_node_id))  # next_node_ids
        # write pose
        for pst in tmp_path.poses:
            pose = [
                float(pst.pose.position.x,),
                float(pst.pose.position.y),
                float(pst.pose.position.z),
                float(pst.pose.orientation.x),
                float(pst.pose.orientation.y),
                float(pst.pose.orientation.z),
                float(pst.pose.orientation.w)
            ]
            writer.writerow(pose)
        rospy.loginfo("[pose recorder] path saved to {} \n \
                                  id = {} \n \
                             reverse = {} \n \
                              length = {:.2f} \n \
                         pre_node_id = {} \n \
                        next_node_id = {} \n"
                        .format(filename, msg.id, msg.reverse, path_length, msg.pre_node_id, msg.next_node_id))

    def euler_to_quaternion(self, yaw, pitch, roll):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
            np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
            np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return qx, qy, qz, qw

    def _modify_path_orientation(self, path):
        size = len(path.poses)
        for i in range(0, size-1):
            p1 = path.poses[i]
            p2 = path.poses[i+1]
            yaw = math.atan2(p2.pose.position.y - p1.pose.position.y,
                             p2.pose.position.x - p1.pose.position.x)
            x, y, z, w = self.euler_to_quaternion(yaw, 0, 0)
            p1.pose.orientation.x = x
            p1.pose.orientation.y = y
            p1.pose.orientation.z = z
            p1.pose.orientation.w = w
        if size > 1:
            path.poses[size-1].pose.orientation = copy.deepcopy(
                path.poses[size-2].pose.orientation)
        return path

    def _cb_current_pose(self, msg_pose):
        self.recored_pose = msg_pose
        if self.is_record_start:
            if (self._pilot_mode == CmdPoseRecord.PILOT_RTKGPS and self.current_rtk_gps_msg is None):
                rospy.logwarn("[pose recorder] Start recording gps path, but rtk gps message is None!")
                return
                
            if self.is_filter:
                if len(self.path.poses) > 0 and self._distance_of_two_pose(self.last_pose, msg_pose) < self.filter_distance:
                    return
            self.last_pose = msg_pose
            self.path.poses.append(msg_pose)
            # if self._pilot_mode == CmdPoseRecord.PILOT_LIDAR:
            #     self.path.poses.append(msg_pose)
            # elif self._pilot_mode == CmdPoseRecord.PILOT_RTKGPS:
            #     pose = PoseStamped()
            #     self._rtk_pose_lock.acquire()
            #     pose.pose.position.x = self.current_rtk_gps_msg.latitude
            #     pose.pose.position.y = self.current_rtk_gps_msg.longitude
            #     self._rtk_pose_lock.release()
            #     self.path.poses.append(pose)
                
            self.path_pub.publish(self.path)
            # rospy.loginfo("input and size: {}".format(len(self.path.poses)))

    def _cb_rtk_gps_fixed(self, msg):
        self._rtk_pose_lock.acquire()
        self.current_rtk_gps_msg = msg
        self._rtk_pose_lock.release()

    def _cb_save_station(self, msg):
        msg = SaveStation()

        if msg.station_name == '':
            rospy.logwarn("[pose recorder] msg station name not set!")
            return
        if msg.save_path == "":
            rospy.logwarn("[pose recorder] msg station save path not set!")
            return

        filename = os.path.join(msg.save_path, msg.station_name) + ".csv"
        with open(filename, 'w') as f:
            f.write(str(self.recored_pose.pose.position.x)+"\n")
            f.write(str(self.recored_pose.pose.position.y)+"\n")
            f.write(str(self.recored_pose.pose.position.z)+"\n")
        rospy.loginfo("[pose recorder] Save ndt pose [{}, {}, {}] to {}".format(
            self.recored_pose.pose.position.x,
            self.recored_pose.pose.position.y,
            self.recored_pose.pose.position.z,
            filename
        ))

    def run(self):
        rospy.spin()
    
    def reset(self):
        self.last_pose = None
        self.current_rtk_gps_msg = None
        self.path = Path()
        self.path.header.frame_id = "map"
        self._pilot_mode = None
        self.is_record_start = False
        self.last_say_time = rospy.Time.now()

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
