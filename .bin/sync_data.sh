#!/bin/bash
# ubuntu 18.04 noetic

rsync -avz --progress ../data  robot@192.168.1.140:/home/robot/htc-robot-ros_ws/

# rsync -avz --progress ../data  fusheng@192.168.1.109:/Users/fusheng/WorkSpace/CompanyWork/work-fusheng/robot-pro/htc-robot-ros_ws/