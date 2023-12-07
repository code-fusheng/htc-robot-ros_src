#!/usr/bin/env python2
#-*-coding:utf-8-*-

import rospy
from geometry_msgs.msg import TwistStamped
from can_msgs.msg import ecu
from math import fabs

class twist_to_ecu:

    def __init__(self):

        rospy.init_node("twist_to_ecu", anonymous=True)

        self.cmd_ecu = ecu()
        self.cmd_twist = TwistStamped()

        self.cmd_twist_sub = rospy.Subscriber("twist_cmd", TwistStamped, self.callback_twist)
        self.cmd_ecu_pub = rospy.Publisher("ecu", ecu, queue_size=10)

        rospy.spin()

    def callback_twist(self, msg):
        ecu_msg = ecu()
            
    
        self.cmd_ecu_pub.publish(ecu_msg)

    def shutdown(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()

if __name__ == "__main__":
    twist_to_ecu = twist_to_ecu()



