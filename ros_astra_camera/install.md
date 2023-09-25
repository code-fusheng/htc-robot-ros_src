1. Git 地址
   https://github.com/orbbec/ros_astra_camera

2. 安装库&依赖

```shell
# V
sudo apt install ros-$ROS_DISTRO-rgbd-launch
# PS noetic 版本没有该 libuvc 依赖库 需要自行编译
sudo apt install ros-$ROS_DISTRO-libuvc
sudo apt install ros-$ROS_DISTRO-libuvc-camera
sudo apt install ros-$ROS_DISTRO-libuvc-ros
# V+
sudo apt install libgflags-dev \
ros-$ROS_DISTRO-image-geometry \
ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-image-transport \
ros-$ROS_DISTRO-image-publisher \
libgoogle-glog-dev \
libusb-1.0-0-dev \
libeigen3-dev
```

# Assuming you have sourced the ros environment, same below

sudo apt install libgflags-dev  
ros-$ROS_DISTRO-image-geometry 
ros-$ROS_DISTRO-camera-info-manager\
ros-$ROS_DISTRO-image-transport 
ros-$ROS_DISTRO-image-publisher
libgoogle-glog-dev
libusb-1.0-0-dev libeigen3-dev

3. 源码编译 libuvc(noetic 版本)

> 修改 CMakeLists.txt 中 libuvc 配置

```shell
find_package(libuvc REQUIRED)
set(libuvc_INCLUDE_DIRS "/usr/local/include/libuvc")
set(libuvc_LIBRARIES "/usr/local/lib/libuvc.so")
```

4. 编译

```shell
cd /home/code/htc-robot-ros_ws/src/astras
sudo chmod -R 755 ros_astra_camera
cd ../..
catkin_make
# catkin_make -DCATKIN_HTC-ROBOT-ROS_WS_PACKAGES="ros_astra_camera"
```

5. 创建设备规则

```shell
   cd ~/htc-robot-ros_ws/src/astras/ros_astra_camera/scripts
   sudo ./create_udev_rules
```

> 规则拓展说明

```shell
# 无需执行这部分
cat create_udev_rules
#!/bin/bash
echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo "up the astra usb connection as /dev/astra*"
echo ""
sudo cp `rospack find astra_camera`/56-orbbec-usb.rules /etc/udev/rules.d
echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
#sudo udevadm trigger --action=change
```
