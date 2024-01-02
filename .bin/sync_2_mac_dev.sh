#!/bin/bash
# rsync --delete -avz --progress --exclude='.git' . code@192.168.1.139:/home/code/htc-robot-ros_ws/src
rsync --delete -avz --progress --exclude='.git' . code@192.168.2.11:/home/code/htc-robot-ros_ws/src