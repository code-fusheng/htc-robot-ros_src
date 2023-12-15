#!/usr/bin/env python2
#-*-coding:utf-8-*-

from math import fabs
import rospy
from geometry_msgs.msg import Twist
from can_msgs.msg import ecu

class App:
    def __init__(self):
        rospy.init_node("cmdvel_to_ecu", anonymous=False, log_level=rospy.INFO)
        self.speed_ratio = 3.6  # cmd_vel转ecu时, 速度的比率. /cmd_vel的速度为m, /ecu的速度为km, 因此首先要乘3.6
        self.angle_akm_ratio = -100.0  # cmd_vel转ecu时, 转角的比率
        self.angle_diff_ratio = -150.0
        self.steer_type = rospy.get_param("~steer_type", default="akm")

    def add_pub(self):
        self.pub_ecu = rospy.Publisher("ecu", ecu, queue_size=5)

    def add_sub(self):
        rospy.Subscriber("/cmd_vel", Twist, self.cb_cmdvel, queue_size=5)

    def add_service(self):
        pass

    def run(self):
        self.add_pub()
        self.add_sub()
        rospy.spin()

    #
    # CallBack functions
    #
    def cb_cmdvel(self, msg):
        ecu_msg = ecu()
        if self.steer_type == "akm":
            if fabs(msg.linear.x) < 0.0001:
                ecu_msg.motor = 0
                ecu_msg.shift = ecu.SHIFT_N
                ecu_msg.brake = True
            elif msg.linear.x < 0:
                ecu_msg.motor = fabs(msg.linear.x*self.speed_ratio)
                ecu_msg.steer = msg.angular.z*self.angle_akm_ratio  # TODO::设置rotate_to_steer
                ecu_msg.shift = ecu.SHIFT_R
                ecu_msg.brake = False
            else:
                ecu_msg.motor = fabs(msg.linear.x*self.speed_ratio)
                ecu_msg.steer = msg.angular.z*self.angle_akm_ratio
                ecu_msg.shift = ecu.SHIFT_D
                ecu_msg.brake = False

        elif self.steer_type == "diff":
            ecu_msg.motor = msg.linear.x*self.speed_ratio
            ecu_msg.steer = msg.angular.z*self.angle_diff_ratio
            ecu_msg.shift = ecu.SHIFT_D
            # 因为是低速, 因此不考虑刹车, 靠减速停车
            ecu_msg.brake = False
            
        else:
            rospy.logerr("[cmdvel_to_ecu] Unsupported steer type: {}!".format(self.steer_type))
            return
        self.pub_ecu.publish(ecu_msg)

    def shutdown(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()

if __name__ == "__main__":
    app = App()
    try:
        app.run()
    except Exception as e:
        print("Exception: [node name]", e)
    finally:
        app.shutdown()
