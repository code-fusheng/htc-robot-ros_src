<!--
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2023-09-21 17:28:23
-->

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
# rostopic echo /sacn --noarr
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
  string frame_id          # 激光雷达基准坐标系
float32 angle_min          # 扫描的起始角度(单位:弧度)
float32 angle_max          # 扫描的终止角度(单位:弧度)
float32 angle_increment    # 相邻两次测距的旋转角度(单位:弧度)
float32 time_increment     # 相邻两次测距的时间差(单位:秒)
float32 scan_time          # 两次扫描的起始时间差(单位:秒)
float32 range_min          # 有效测距范围的最小距离(单位:米)
float32 range_max          # 有效测距范围的最大距离(单位:米)
float32[] ranges           # 本次扫描的所有测距值(单位:米)
float32[] intensities      # 所有测距的返回信号强度
```
