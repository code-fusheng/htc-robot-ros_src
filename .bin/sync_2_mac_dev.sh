#!/bin/bash
rsync --delete -avz --progress --exclude='.git' . code@192.168.1.100:/home/code/htc-robot-ros_ws/src
# rsync --delete -avz --progress --exclude='.git' . code@192.168.2.12:/home/code/htc-robot-ros_ws/src