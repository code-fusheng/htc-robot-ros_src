#!/bin/bash
rsync --delete -avz --progress --exclude='.git' . code@192.168.1.131:/home/code/htc-robot-ros_ws/src