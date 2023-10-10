#!/bin/bash

# 安装基础依赖

```shell
sudo apt install -y vim \
terminator \
libmetis-dev \
```

# 安装相机依赖

```shell
sudo apt install -y ros-$ROS_DISTRO-rgbd-launch \
ros-$ROS_DISTRO-libuvc \
ros-$ROS_DISTRO-libuvc-camera \
ros-$ROS_DISTRO-libuvc-ros
```
