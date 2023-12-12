#!/usr/bin/env python2
#-*-coding:utf-8-*-

import serial
import time
import struct

serial_client = serial.Serial(port = "/dev/ttyUSB0",
                            baudrate = 115200,
                            timeout = 0.2,
                            bytesize = serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits= serial.STOPBITS_ONE
                            )

cmd1 = '\xb4\x10\xff'

cmd2 = bytearray()
cmd2.extend(struct.pack('>B', 0xb4))
cmd2.extend(struct.pack('>B', 0x10))
cmd2.extend(struct.pack('>B', 0xff))
print(cmd2, len(cmd2), cmd2[0], cmd2[1])

for i in range(10):
    print("write {} to serial".format(cmd2))
    serial_client.write(cmd2)
    time.sleep(0.1)
    line = bytearray()
    line = serial_client.read_all()
    print(line, len(line), line[0], line[1], line[2], line[3])
