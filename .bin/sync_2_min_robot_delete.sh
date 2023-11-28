#!/bin/bash
# ubuntu 20.04 noetic
rsync --delete -avz --progress --exclude='.git' . robot@192.168.1.111:/home/robot/htc-robot-ros_ws/src