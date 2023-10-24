#!/bin/bash
# ubuntu 18.04 noetic
rsync -avz --progress ../../drivers code@192.168.1.130:/home/code/htc-robot-ros_ws
rsync -avz --progress ../../data code@192.168.1.102:/home/code/htc-robot-ros_ws

# To 公司主机 VM 
# rsync -avz --progress ../drivers htc@192.168.1.130:/home/htc/htc-robot-ros_ws
# rsync -avz --progress ../data htc@192.168.1.130:/home/htc/htc-robot-ros_ws

rsync -avz --progress ../../drivers/r8168-8.046.00.tar.bz2 code@192.168.1.130:/home/code/htc-robot-ros_ws/