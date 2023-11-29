# Autoware.AI 手册

### Autoware.AI 安装

官网&代码地址：[https://github.com/autowarefoundation/autoware/tree/autoware-ai](https://github.com/autowarefoundation/autoware/tree/autoware-ai)

> 下载依赖

```shell
$ sudo apt update
$ sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
$ sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
$ pip3 install -U setuptools
```

> 更新 elgen3.37

```shell
#Download Eigen
$ wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
#Decompress
$ tar -xzvf 3.3.7.tar.gz
#Build and install
$ cd eigen && mkdir build && cd build && cmake .. && make && sudo make install
#Remove downloaded and temporary files
$ cd && rm -rf 3.3.7.tar.gz && rm -rf eigen
```

> 创建工作空间

```shell
$ mkdir -p autoware.ai/src
$ cd autoware.ai
```

> 下载功能包

```shell
$ wget -O autoware.ai.repos "https://gitlab.com/autowarefoundation/autoware.ai/autoware/raw/1.14.0/autoware.ai.repos?inline=false"
$ vcs import src < autoware.ai.repos
```

> 鱼香 Ros 依赖更新

```
sudo apt install python-pip
sudo pip install rosdepc
sudo rosdepc init
首先用鱼香肉丝一键安装rosdepc
rosdepc update
rosdepc install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

```
sudo apt-get install -y \
ros-$ROS_DISTRO-automotive-navigation-msgs \
ros-$ROS_DISTRO-image-view2 \
ros-$ROS_DISTRO-uvc-camera \
ros-$ROS_DISTRO-velocity-controllers \
ros-$ROS_DISTRO-jsk-topic-tools \
ros-$ROS_DISTRO-effort-controllers \
ros-$ROS_DISTRO-gps-common \
ros-$ROS_DISTRO-jsk-rviz-plugins \
ros-$ROS_DISTRO-carla-msgs \
ros-$ROS_DISTRO-velodyne-description \
ros-$ROS_DISTRO-nmea-msgs \
ros-$ROS_DISTRO-jsk-recognition-msgs \
ros-$ROS_DISTRO-velodyne-pointcloud \
ros-$ROS_DISTRO-gscam \
ros-$ROS_DISTRO-lgsvl-msgs \
ros-$ROS_DISTRO-velodyne \
ros-$ROS_DISTRO-qpoases-vendor \
ros-$ROS_DISTRO-automotive-platform-msgs \
ros-$ROS_DISTRO-sound-play \
ros-$ROS_DISTRO-grid-map-ros \
ros-$ROS_DISTRO-geodesy \
ros-$ROS_DISTRO-jsk-topic-tools \
ros-$ROS_DISTRO-velodyne-gazebo-plugins \
ros-$ROS_DISTRO-rosbridge-server \
ros-$ROS_DISTRO-imu-tools \
ros-$ROS_DISTRO-nmea-navsat-driver \
libglew-dev \
ros-$ROS_DISTRO-lanelet2*
```

> 编译

```shell
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release（GPU版）
AUTOWARE_COMPILE_WITH_CUDA=0 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error（CPU版）
```

> 运行

```
source install/setup.bash
roslaunch runtime_manager runtime_manager.launch
```

### Autoware.AI 官方 demo 运行

[https://github.com/autowarefoundation/autoware_ai_documentation/wiki/ROSBAG-Demo](https://github.com/autowarefoundation/autoware_ai_documentation/wiki/ROSBAG-Demo)

> 启动 autoware

```
roslaunch runtime_manager runtime_manager.launch
```
