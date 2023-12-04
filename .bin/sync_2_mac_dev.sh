#!/bin/bash
rsync -avz --progress --exclude='.git' . code@192.168.1.143:/home/code/htc-robot-ros_ws/src