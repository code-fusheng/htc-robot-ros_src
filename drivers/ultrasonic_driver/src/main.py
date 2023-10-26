#!/usr/bin/env python2
#-*-coding:utf-8-*-
import rospy
import struct
import serial
from ultrasonic_driver.msg import UltraSonicDetect

class App:

    def __init__(self):

        # Init node
        rospy.init_node('ultrasonic_driver', anonymous=False)
        self.node_name = rospy.get_name()
        self.rate = rospy.Rate(10)
        self.pub_ultrasonic_detection = rospy.Publisher("/ultrasonic_detection", UltraSonicDetect, queue_size=3)

        # Get ros params
        self.get_ros_params()

    def get_ros_params(self):
        self.serial_port = rospy.get_param('~serial_port','/dev/ttyUSB0')
        self.frame_id = rospy.get_param('~frame_id', 'ultrasonic_link')

    def run(self):
        try:
            self.serial_port = serial.Serial(port = self.serial_port,
                                            baudrate = 115200,
                                            timeout = 0.2,
                                            bytesize = serial.EIGHTBITS,
                                            parity=serial.PARITY_NONE,
                                            stopbits= serial.STOPBITS_ONE
                                            )
        except Exception as e:
            rospy.logerr("[Ultrasonic] Open serial failed, exit. {}".format(e))
            exit(1)
        data = bytearray()
        data.extend(struct.pack('>B', 0xb4))  # 根据协议指定是一直发，还是一收一发
        data.extend(struct.pack('>B', 0x10))
        data.extend(struct.pack('>B', 0xff))

        # for i in range(3):
        #     self.serial_port.write(data)
        #     rospy.sleep(1)

        self.buff = bytearray()
        while not rospy.is_shutdown():
            # self.rate.sleep()

            self.serial_port.write(data)  # 请求读取一次数据
            # self.serial_port.flush()
            # while self.serial_port.inWaiting() <= 0:
            #     rospy.loginfo("waiting")
            self.rate.sleep()
            self.buff = bytearray()
            line = self.serial_port.read_all()
            if ( len(line) != 28 ):
                rospy.loginfo("[ultrasonic] Received invalid data, lengh={} (!=28)".format(len(line)))
                continue
            self.buff.extend(line)
            if ( self.buff[0] != 0xff ):
                rospy.loginfo("[ultrasonic] Received invalid data, header={} (!=0xff <255>)".format(self.buff[0]))
                continue
            self.handle(self.buff)
            # if len(line) == 0:
            #     continue
            # self.buff.extend(line)
            # ind = 0
            # i = 0
            # while i < len(self.buff):
            #     if self.buff[i] == 0xff:
            #         ind = i
            #         if (i+2) >= len(self.buff):
            #             ind = i
            #             break
            #         length = self.buff[i+2]
            #         total_length = 3+length+1
            #         if (i+total_length) >= len(self.buff):
            #             break
            #         self.handle(self.buff[ind:i+total_length])

            #     i += 1

            # self.buff = self.buff[ind:]

    def handle(self, data):
        i = 3
        msg = UltraSonicDetect()
        while i+3 < len(data):
            # dist = data[i]*100+data[i+1]
            dist = data[i]/16*1000+data[i]%16*100+data[i+1]/16*10+data[i+1]%16
            id = data[i+2]
            msg.distance.append(id)
            if (dist > 4900):  # 超出4900即为无障碍物
                msg.distance.append(10000000)
            else:
                msg.distance.append(dist/1000.0)
            i += 3

        msg.nums = len(msg.distance)/2
        self.pub_ultrasonic_detection.publish(msg)

    def shutdown(self):
        return

if __name__ == '__main__':
    app = App()
    try:
        app.run()
    except rospy.ROSInterruptException:
        app.shutdown()
    except Exception as e:
        print(e)

