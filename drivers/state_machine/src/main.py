#!/usr/bin/python
# -*-coding:utf-8-*-
import rospy
from smartcar_msgs.msg import State, CloudInterface, CrossLock, SonarInterface
from automotive_msgs.msg import UserCmd
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
import threading
import time
from std_msgs.msg import Int16
from enum import Enum
# from sdhy_driverlesscar.msg import hy_ultrasonic_value, hy_millimeterwave_frame
from std_msgs.msg import Bool
from ultrasonic_driver.msg import UltraSonicDetect
from automotive_msgs.srv import SmartcarSolution, SmartcarSolutionRequest, SmartcarSolutionResponse


SPIN_RATE = 20 # hz
LOCK_TIME = 5  # s

class SensorType(Enum):
    User = 0
    Lidar = 1
    UltroSonic = 2
    Radar = 3


class RState(Enum):
    ALLOW = 0
    FORBID = 1


class StateContainer:
    def __init__(self):
        self.state = {
            SensorType.User: RState.FORBID,
            SensorType.Lidar: RState.ALLOW,
            SensorType.UltroSonic: RState.ALLOW,
            SensorType.Radar: RState.ALLOW
        }

        self.ultrasonic_state = [RState.ALLOW, RState.ALLOW, RState.ALLOW, RState.ALLOW, RState.ALLOW, RState.ALLOW, RState.ALLOW, RState.ALLOW]

    def is_state_ok(self):
        if RState.FORBID in self.ultrasonic_state:
            self.state[SensorType.UltroSonic] = RState.FORBID
        else:
            self.state[SensorType.UltroSonic] = RState.ALLOW

        for state in self.state.values():  # 任何一个是Forbid，表示当前不能行驶
            if state is RState.FORBID:
                return RState.FORBID
        return RState.ALLOW

    def set_ultrasonic_state(self, sensor_type, state, sensor_id):
        self.ultrasonic_state[sensor_id] = state

    def set_state(self, sensor_type, state):
        self.state[sensor_type] = state

    def get_state(self, sensor_type):
        return self.state[sensor_type]


class App():
    def __init__(self):
        rospy.init_node("state_machine", log_level=rospy.INFO)
        self.state = StateContainer()
        self.last_state = None
        self.last_lock_time = 0.0
        self.current_solution = SmartcarSolutionResponse.SMARTCAR_LIDAR

    def run(self):
        self._init_params()
        self._add_pub()
        self._add_sub()
        self._add_srv()
        rate = rospy.Rate(SPIN_RATE)
        while not rospy.is_shutdown():
            self.update_state()
            rate.sleep()

    def _init_params(self):
        # lidar cluster params
        self.lidar_range_left = rospy.get_param("~lidar_range_left", default=1.5)
        self.lidar_range_left = -self.lidar_range_left
        self.lidar_range_right = rospy.get_param("~lidar_range_right", default=1.5)
        self.lidar_range_front = rospy.get_param("~lidar_range_front", default=4)

        # UltroSonic params

        sonic1_limit = rospy.get_param("~ultrosonic_detect_distance_1", default=0.5)
        sonic2_limit = rospy.get_param("~ultrosonic_detect_distance_2", default=0.5)
        sonic3_limit = rospy.get_param("~ultrosonic_detect_distance_3", default=0.5)
        sonic4_limit = rospy.get_param("~ultrosonic_detect_distance_4", default=0.5)
        sonic5_limit = rospy.get_param("~ultrosonic_detect_distance_5", default=0.5)
        sonic6_limit = rospy.get_param("~ultrosonic_detect_distance_6", default=0.5)
        sonic7_limit = rospy.get_param("~ultrosonic_detect_distance_7", default=0.5)
        sonic8_limit = rospy.get_param("~ultrosonic_detect_distance_8", default=0.5)

        self.ultrasonic_distance_limit = [sonic1_limit,sonic2_limit,sonic3_limit,sonic4_limit,sonic5_limit,sonic6_limit,sonic7_limit,sonic8_limit,]
        self.ultrasonic_distance_bump = [0, 0, 0, 0, 0, 0, 0, 0]
        self.ultrasonic_distance_max_bump = 3

        # radar params

    def _add_pub(self):
        self.state_pub = rospy.Publisher("/SmartcarState", State, queue_size=10)

    def _add_sub(self):
        rospy.Subscriber("/UserCmd", UserCmd, self._cb_user_command, queue_size=5)
        # rospy.Subscriber("/detected_bounding_boxs", BoundingBoxArray, self._cb_lidar_cluster, queue_size=5)
        # rospy.Subscriber("/detected_bounding_nums", Int16, self._cb_cluster_nums, queue_size=1)
        rospy.Subscriber("/ultrasonic_detection", UltraSonicDetect, self._cb_ultrosonic_detect, queue_size=5)
        # rospy.Subscriber("/sensor_millimeter", hy_millimeterwave_frame, self._cb_radar_detect, queue_size=5)
        rospy.Subscriber("/pure_pursuit/task_finished", Bool, self._cb_task_finished, queue_size=5)
    
    def _add_srv(self):
        rospy.Service("/state_machine/service_solution_status", SmartcarSolution, self._cb_service_smartcar_solution)
    
    def _cb_service_smartcar_solution(self, req):
        if req.is_set:
            if req.set_solution == SmartcarSolutionResponse.SMARTCAR_LIDAR:
                rospy.loginfo("[state machine] Set solution to LIDAR mode")
            elif req.set_solution == SmartcarSolutionResponse.SMARTCAR_RTKGPS:
                rospy.loginfo("[state machine] Set solution to RTK-GPS mode")
            elif req.set_solution == SmartcarSolutionResponse.SMARTCAR_UNKNOWN:
                rospy.loginfo("[state machine] Set solution to UNKNOWN")
            else:
                rospy.logerr("[state machine] Unknown Solution code: {} !".format(req.set_solution))
                # solution值非法时,保持原来的不变
                res = SmartcarSolutionResponse()
                res.current_solution = self.current_solution
                return res

            self.current_solution = req.set_solution

        res = SmartcarSolutionResponse()
        res.current_solution = self.current_solution
        return res

    def _cb_user_command(self, msg):
        if msg.data == UserCmd.NORMAL_PAUSE:
            rospy.loginfo("[state-machine] Received user's pause command  --> Stop vehicle")
            self.state.set_state(SensorType.User, RState.FORBID)
        elif msg.data == UserCmd.NORMAL_RUN:
            rospy.loginfo("[state-machine] Received user's run command")
            self.state.set_state(SensorType.User, RState.ALLOW)
        else:
            rospy.logwarn("[state-machine] Unrecognized user command, id = {}".format(msg.data))

        # for user command, we conduct it immediately
        state = State()
        if self.state.is_state_ok() is RState.ALLOW:
            if msg.data == UserCmd.NORMAL_RUN:
                rospy.loginfo("[state-machine]     -------> set vehicle to run")
            state.main_state = State.RUN
        else:
            if msg.data == UserCmd.NORMAL_RUN:
                rospy.loginfo("[state-machine]     -------> vehicle will run when obstacle removed")
            state.main_state = State.PAUSE
        self.state_pub.publish(state)

    def update_state(self):
        state = State()
        now = rospy.Time.now().to_sec()
        if self.state.is_state_ok() is RState.ALLOW:
            if (now - self.last_lock_time) < LOCK_TIME:
                # stay still within LOCK_TIME after last obstacle detected
                return
            state.main_state = State.RUN
        else:
            state.main_state = State.PAUSE
            self.last_lock_time = now

        if self.last_state != state.main_state:
            if state.main_state is State.RUN:
                rospy.loginfo("[state_machine] set Current State: RUN")
            else:
                rospy.loginfo("[state_machine] set Current State: PAUSE")

        self.last_state = state.main_state
        self.state_pub.publish(state)

    """
    lidar cluster nums handler, msg.data > 1 means there is cluster in ROI
    """
    def _cb_cluster_nums(self, msg):
        if msg.data > 0:
            rospy.loginfo("[state machine] received lidar cluster")
            self.state.set_state(SensorType.Lidar, RState.FORBID)
        else:
            self.state.set_state(SensorType.Lidar, RState.ALLOW)

    """
    lidar cluster bounding box handler
    """
    def _cb_lidar_cluster(self, msg):
        if len(msg.boxes) > 1:
            rospy.loginfo("[state machine] received lidar cluster")
            self.state.set_state(SensorType.Lidar, RState.FORBID)
        else:
            self.state.set_state(SensorType.Lidar, RState.ALLOW)

    def _cb_ultrosonic_detect(self, msg):
        for i in range(0, msg.nums):
            id = int(msg.distance[2*i]) - 1
            dist = msg.distance[2*i+1]

            if (dist < 0.05):  # min fileter
                continue

            try:
                if dist > self.ultrasonic_distance_limit[id]:
                    self.ultrasonic_distance_bump[id] = 0
                else:
                    self.ultrasonic_distance_bump[id] += 1
            except Exception as e:
                print(msg)
                rospy.loginfo("id={}, dist={:.2f}".format(id, dist))
                print(e)

        for i in range(len(self.ultrasonic_distance_bump)):
            num = self.ultrasonic_distance_bump[i]
            if num > self.ultrasonic_distance_max_bump:
                self.state.set_ultrasonic_state(SensorType.UltroSonic, RState.FORBID, i)
                rospy.loginfo("[state machine] Ultrasonic id:{} triggered stop".format(i+1))
            else:
                self.state.set_ultrasonic_state(SensorType.UltroSonic, RState.ALLOW, i)

    def _cb_radar_detect(self, msg):
        pass

    def _cb_task_finished(self, msg):
        rospy.loginfo("[state machine] received task finished message")
        if msg.data is True:  # task finished
            self.state.set_state(SensorType.User, RState.FORBID)

if __name__ == '__main__':
    app = App()
    try:
        app.run()
    except rospy.ROSInterruptException as e:
        app.shutdown()
        rospy.loginfo(e)
