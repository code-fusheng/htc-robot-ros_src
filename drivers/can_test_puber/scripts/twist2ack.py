#!/usr/bin/env python2
#-*-coding:utf-8-*-
import rospy
from geometry_msgs.msg import TwistStamped
from can_msgs.msg import ecu

class twist2ack():
    def __init__(self):
        rospy.init_node('twist_to_ackermann',anonymous=True)
#        self.mov_cmd_vel = ctrl_cmd()
        self.mov_ecu = ecu()
        self.cmd_vel = TwistStamped()

        self.cmd_vel_sub = rospy.Subscriber('twist_cmd', TwistStamped, self.callback)
        rospy.sleep(2)
#        self.cmd_vel_pub = rospy.Publisher('ctrl_cmd', ctrl_cmd, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('ecu', ecu, queue_size=10)
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.transform()


    def callback(self,data):
        self.cmd_vel = data

    def transform(self):
# no       self.mov_cmd_vel.ctrl_cmd_gear = 4
#        self.mov_cmd_vel.ctrl_cmd_velocity = self.cmd_vel.twist.linear.x
#        self.mov_cmd_vel.ctrl_cmd_steering = self.cmd_vel.twist.angular.z
#        self.cmd_vel_pub.publish(self.mov_cmd_vel)
        self.mov_ecu.shift = self.mov_ecu.SHIFT_D                   #D(1) 前进  N(2) 停止   R(3) 后退
        self.mov_ecu.motor = self.cmd_vel.twist.linear.x *3.6           #motor的单位是km/h，0.0~10.0，精度为0.1     twist的单位是m/s
        if self.mov_ecu.motor>10:
            self.mov_ecu.motor = 10
        self.mov_ecu.steer = (self.cmd_vel.twist.angular.z * 180.0 /3.1415926) * 120.0 / 27.0           #steer是+-120.0，对应到+-27°               twist的单位是？假设是弧度制
        if self.mov_ecu.steer > 120:
            self.mov_ecu.steer = 120
        if self.mov_ecu.steer < -120:
            self.mov_ecu.steer = -120
        self.mov_ecu.brake = False

        if self.mov_ecu.motor<0.1:
            self.mov_ecu.shift = self.mov_ecu.SHIFT_N
            self.mov_ecu.brake = True
        self.cmd_vel_pub.publish(self.mov_ecu)
        self.rate.sleep()


if __name__ == '__main__':
    twist2ack()
