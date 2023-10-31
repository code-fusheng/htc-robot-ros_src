#!/bin/bash
# ubuntu 18.04 noetic
# rsync -avz --progress ../../drivers code@192.168.1.130:/home/code/htc-robot-ros_ws
# rsync -avz --progress ../../data code@192.168.1.102:/home/code/htc-robot-ros_ws

# To 公司主机 VM 
# rsync -avz --progress ../drivers htc@192.168.1.130:/home/htc/htc-robot-ros_ws
# rsync -avz --progress ../drivers/wxPython-4.0.7.post2.tar.gz  htc@192.168.1.111:/home/htc/htc-robot-ros_ws
rsync -avz --progress ../drivers/Clash.tar.gz code@192.168.1.102:/home/code/htc-robot-ros_ws/drivers
# rsync -avz --progress ../../drivers/r8168-8.046.00.tar.bz2 code@192.168.1.130:/home/code/htc-robot-ros_ws/