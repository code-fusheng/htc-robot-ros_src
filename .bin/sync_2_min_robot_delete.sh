#!/bin/bash
# ubuntu 20.04 noetic
rsync --delete -avz --progress --exclude='.git' . code@192.168.1.120:/home/code/htc-robot-ros_ws/src