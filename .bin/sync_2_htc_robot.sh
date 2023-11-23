#!/bin/bash
# ubuntu 18.04
rsync --delete -avz --progress --exclude='.git' . robot@192.168.1.140:/home/robot/htc-robot-ros_ws/src