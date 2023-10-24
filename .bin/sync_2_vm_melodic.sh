#!/bin/bash
# ubuntu 18.04 melodic
# rsync -avz --progress . fusheng@192.168.1.180:/home/fusheng/htc-robot-ros_ws/src
rsync --delete -avz --progress ../ htc@192.168.1.111:/home/htc/htc-robot-ros_ws/src
# rsync --delete -avz --progress . code@192.168.2.5:/home/code/htc-robot-ros_ws/src