## 系统环境安装手册

### 安装基础依赖

```shell
sudo apt install -y vim \
terminator \
libmetis-dev \
```

### 安装相机依赖

```shell
sudo apt install -y ros-$ROS_DISTRO-rgbd-launch \
ros-$ROS_DISTRO-libuvc \
ros-$ROS_DISTRO-libuvc-camera \
ros-$ROS_DISTRO-libuvc-ros
```

```

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
