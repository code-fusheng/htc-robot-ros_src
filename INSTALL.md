## 系统环境安装手册

### 安装基础依赖

```shell
sudo apt install -y vim \
terminator \
libmetis-dev \
libpcap-dev \
ros-$ROS_DISTRO-bfl \
ros-$ROS_DISTRO-serial \
ros-$ROS_DISTRO-tf2-sensor-msgs \
ros-$ROS_DISTRO-costmap-converter \
ros-$ROS_DISTRO-mbf_costmap_core \
ros-$ROS_DISTRO-rgbd-launch
```

### 安装相机依赖

```shell
sudo apt install -y ros-$ROS_DISTRO-rgbd-launch \
ros-$ROS_DISTRO-libuvc \
ros-$ROS_DISTRO-libuvc-camera \
ros-$ROS_DISTRO-libuvc-ros
```

###

```shell
# 安装依赖
sudo apt install -y libudev-dev \
pkg-config \
libgtk-3-dev \
libusb-1.0-0-dev \
libglfw3-dev \
libssl-dev \
# 下载 librealsense 源码
git clone https://github.com/IntelRealSense/librealsense.git
# git clone https://ghproxy.com/https://github.com/IntelRealSense/librealsense.git
cd librealsense
# 安装权限脚本
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
# 编译
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=ture
make
#
git clone https://gitcode.net/mirrors/curl/curl.git
cd build/third-party
sudo make install
#
git clone https://github.com/IntelRealSense/realsense-ros.git
git checkout 2.3.2
git checkout -b melodic_2.3.2 2.3.2
#
roslaunch realsense2_camera rs_camera.launch
```

```shell
# 打开imu
<arg name="enable_gyro"         default="true"/>
<arg name="enable_accel"        default="true"/>
# 联合方式copy或linear_interpolation
<arg name="unite_imu_method"          default="linear_interpolation"/>
# 时间戳同步
<arg name="enable_sync"               default="true"/>
```

### 安装激光雷达

```shell

```

```shell
# Wireshark 抓包调试雷达
```

### 安装 rtabmap

```shell
sudo apt install -y ros-$ROS_DISTRO-rtabmap*
```

```shell
# 编译 rtabmap_ros
git clone https://github.com/introlab/rtabmap_ros.git
# 切换分支
git checkout noetic-devel
```

```
git submodule add https://github.com/pal-robotics/ddynamic_reconfigure.git drivers/ddynamic_reconfigure
git submodule add https://github.com/code-fusheng/realsense-ros.git drivers/realsense-ros
```
