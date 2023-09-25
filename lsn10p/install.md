1. 安装相关库
   sudo apt-get install libpcap-dev

2. 编译

拷贝 code 到工作空间 scp lsn10p => ws
catkin_make -DCATKIN_HTC-ROBOT-ROS_WS_PACKAGES="lsn10p"

3. 运行
   网口雷达运行：
   roslaunch lslidar_driver lslidar_net.launch
   串口雷达运行：
   roslaunch lslidar_driver lslidar_serial.launch

4. 查看 Rviz
   rviz
   frame_id: laser
   LaserScan: scan

5. 配置
   lsn10p/lslidar_driver/launch/lslidar_serial.launch

6. 雷达 Topic

```shell
$ rostopic info scan
Type: sensor_msgs/LaserScan

Publishers:
 * /lslidar_driver_node (http://code-AIxBoard:35333/)

Subscribers: None
```

7. 雷达 msgs

```shell
rosmsg show sensor_msgs/LaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```
