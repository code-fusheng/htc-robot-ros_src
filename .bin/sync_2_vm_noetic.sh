#!/bin/bash
# ubuntu 20.04 noetic
# rsync --delete -avz --progress . htc@192.168.1.134:/home/htc/htc-robot-ros_ws/src

# rsync --delete -avz --progress . code@192.168.2.102:/home/code/htc-robot-ros_ws/src

rsync --delete -avz --progress ../ htc@192.168.1.130:/home/htc/htc-robot-ros_ws/src

