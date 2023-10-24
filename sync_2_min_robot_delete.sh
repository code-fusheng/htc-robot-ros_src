#!/bin/bash
# ubuntu 20.04 noetic
rsync --delete -avz --progress --exclude={'./CMakeLists.txt','.git'} . code@192.168.1.135:/home/code/htc-robot-ros_ws/src