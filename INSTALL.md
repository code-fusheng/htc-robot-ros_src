## 系统环境安装手册

### 初始化系统环境

> 网络优化

```
# Github 仓库代理
https://ghproxy.com/
# 科学订阅地址
https://sub.wl-sub1.com/api/v1/client/subscribe?token=1ddb6feb800a114b7bdb3afc43373ddf
```

> 安装 ssh

```shell
# 安装 ssh
sudo apt install -y openssh-server
sudo vim /etc/ssh/sshd_config
sudo service ssh restart
```

### 安装 ROS

```shell
wget http://fishros.com/install -O fishros && . fishros
```

### 安装基础依赖

```shell
sudo apt install -y vim \
git \
terminator \
libmetis-dev \
libpcap-dev \
ros-$ROS_DISTRO-serial \
ros-$ROS_DISTRO-tf2-sensor-msgs \
ros-$ROS_DISTRO-costmap-converter \
ros-$ROS_DISTRO-mbf-costmap-core \
ros-$ROS_DISTRO-rgbd-launch \
ros-$ROS_DISTRO-nmea-msgs \
ros-$ROS_DISTRO-gps-common \
ros-$ROS_DISTRO-mbf-msgs \
ros-$ROS_DISTRO-gmapping

ros-$ROS_DISTRO-bfl \

```

### 安装 autoware 移植基础依赖

```shell
sudo apt install -y ros-$ROS_DISTRO-autoware-msgs \
ros-$ROS_DISTRO-autoware-config-msgs
ros-$ROS_DISTRO-rqt-runtime-monitor

```

### 安装相机依赖

```shell
sudo apt install -y ros-$ROS_DISTRO-rgbd-launch \
ros-$ROS_DISTRO-libuvc-camera \
ros-$ROS_DISTRO-libuvc-ros

ros-$ROS_DISTRO-libuvc \
```

> 编译 libuvc（noetic 环境）

PS: noetic 环境下 libuvc 需要拉源码进行编译 [https://github.com/libuvc/libuvc](https://github.com/libuvc/libuvc)

```shell
git clone https://ghproxy.com/https://github.com/libuvc/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install
# ros_astra_camera 功能包设置 libuvc 环境
find_package(libuvc QUIET)
if(EXISTS "/usr/local/include/libuvc")
  set(libuvc_INCLUDE_DIRS "/usr/local/include/libuvc")
  set(libuvc_LIBRARIES "/usr/local/lib/libuvc.so")
endif()
```

```shell
# 安装依赖
sudo apt install -y ros-$ROS_DISTRO-ddynamic-reconfigure \
libudev-dev \
pkg-config \
libgtk-3-dev \
libusb-1.0-0-dev \
libglfw3-dev \
libssl-dev
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
sudo make install
# 建议上科学
git clone https://gitcode.net/mirrors/curl/curl.git
cd build/third-party

#
git clone https://github.com/IntelRealSense/realsense-ros.git
git checkout 2.3.2
git checkout -b 2.3.2 2.3.2
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

### 安装 Rtapmap-Ros

```shell
# 编译 rtabmap_ros
git clone https://github.com/introlab/rtabmap_ros.git
# 不同的版本环境需要切换分支
git checkout noetic-devel
git checkout melodic-devel
```

```
git submodule add https://github.com/pal-robotics/ddynamic_reconfigure.git drivers/ddynamic_reconfigure
git submodule add https://github.com/code-fusheng/realsense-ros.git drivers/realsense-ros
```

### 安装 Lio-Sam

```shell
# 基础依赖
sudo apt-get install -y ros-$ROS_DISTRO-navigation \
ros-$ROS_DISTRO-robot-localization \
ros-$ROS_DISTRO-robot-state-publisher \
ros-$ROS_DISTRO-fake-localization \
libmetis-dev \
libtbb-dev
# gtsam method:1 (需要科学) # noetic 见下方问题处理
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev

# 拉取 lio-sam & 编译
git clone https://github.com/TixiaoShan/LIO-SAM.git
catkin_make -j1 -DCATKIN_WHITELIST_PACKAGES=lio_sam
```

> lio-sam 编译问题(noetic)

[https://github.com/TixiaoShan/LIO-SAM/issues/206](https://github.com/TixiaoShan/LIO-SAM/issues/206)

```shell
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
# 修改
#include <opencv/cv.h> => <opencv2/opencv.hpp>
#                      => <opencv2/imgproc.hpp>
#
# 修改 set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
# 移位置
#include <pcl/kdtree/kdtree_flann.h>
```

### 安装 LeGo-Loam

```shell
sudo apt-get install -y libmetis-dev
catkin_make -DCATKIN_WHITELIST_PACKAGES=cloud_msgs
catkin_make -j1 -DCATKIN_WHITELIST_PACKAGES=lego_loam
#include <opencv2/opencv.hpp>
```

### 问题处理

#### 1. 20.04 缺少 wifi 适配器 & 18.04 缺少 wifi 适配器

lspci -v

sudo apt install git
sudo apt install build-essential
sudo apt install dkms
git clone https://ghproxy.com/https://github.com/tomaspinho/rtl8821ce.git
cd rtl8821ce

> Intel AX201 => iwlwifi

sudo apt install git linux-headers-generic build-essential
git clone https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git

#### 1. noetic 缺少 bfl

sudo apt -y install liborocos-bfl-dev

#### melodic 缺少 sdl

sudo apt-get install libsdl-image1.2-dev libsdl-dev

### 编译运行

```shell
catkin_make -DCATKIN_WHITELIST_PACKAGES="lslidar_msgs;cloud_msgs;automotive_msgs;path_msgs;smartcar_msgs;lb_cloud_msgs;rtk_cloud_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_BUILD_TYPE=Release
```

### QT 问题

```
add_library(runtime_control SHARED IMPORTED)
set_target_properties(runtime_control PROPERTIES IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/lib/libruntime_control.so)

add_library(status_dashboard SHARED IMPORTED)
set_target_properties(status_dashboard PROPERTIES IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/lib/libstatus_dashboard.so)
```

### wxPython

```
https://pypi.org/project/wxPython/4.0.7.post2/#files
```
