## rtabmap

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

### rtabmap 话题

输入：
① 里程计：
/odom (nav_msgs/Odometry)
② 雷 达：
/scan (sensor_msgs/LaserScan)
③ 摄像头：
/camera/rgb/image_raw (sensor_msgs/Image)
/camera/rgb/camera_info (sensor_msgs/CameraInfo)
/camera/depth/image (sensor_msgs/Image)

输出：
①2D 地图
/rtabmap/grid_map (nav_msgs/OccupancyGrid)
②3D 点云
/rtabmap/mapData (rtabmap_ros/MapData)
