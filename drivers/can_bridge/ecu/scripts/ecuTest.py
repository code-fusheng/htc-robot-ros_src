#! /usr/bin/env python
import rospy
from can_msgs.msg import ecu

def talker():
    pub = rospy.Publisher('/ecu', ecu, queue_size=5)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    can_m = ecu()
    while not rospy.is_shutdown():
        can_m.shift = 1
        can_m.motor = 0
        can_m.steer = 20.0

        pub.publish(can_m)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


