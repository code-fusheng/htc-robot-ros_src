```shell
$ rosmsg show sensor_msgs/LaserScan
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

```shell
$ rostopic echo /sacn --noarr
header:
  seq: 410
  stamp:
    secs: 1695621405
    nsecs: 994232035
  frame_id: "laser"
angle_min: -3.1415927410125732
angle_max: 3.1415927410125732
angle_increment: 0.011877477169036865
time_increment: 0.0
scan_time: 0.0
range_min: 0.15000000596046448
range_max: 100.0
ranges: "<array type: float32, length: 529>"
intensities: "<array type: float32, length: 529>"
```
