#!/bin/bash
# ubuntu 20.04 noetic
rsync -avz --progress ../drivers code@192.168.1.102:/home/code/htc-robot-ros_ws/drivers
rsync -avz --progress ../data code@192.168.1.102:/home/code/htc-robot-ros_ws/data
