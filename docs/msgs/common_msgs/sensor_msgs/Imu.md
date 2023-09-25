<!--
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2023-09-25 15:10:20
-->

<!--
    加速度的数值单位是 米/秒^2 , 旋转速度的数值单位为 弧度/秒

    如果协方差数值已知，就将其填充到协方差矩阵中。
    如果协方差数值未知，则将协方差矩阵全部置为零。

    若协方差矩阵对应的数值不存在（比如 IMU 没有输出 orientation 姿态数据），那么该协方差矩阵的第一个数值置为 -1。
    如果要使用这个消息包里的某个数据，需要先对协方差矩阵的第一个数值进行判断：
    如果数值为 -1 表明要使用的数据是不存在的，不要再去读取它。
 -->

```shell
$ rosmsg show sensor_msgs/Imu
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
# 融合数据
geometry_msgs/Quaternion orientation      # 四元数
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
# 裸数据
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance
```

```shell
$ rostopic echo /imu/data --noarr
^Cheader:
  seq: 8669
  stamp:
    secs: 1695626264
    nsecs: 215203049
  frame_id: "gyro_link"
orientation:
  x: -0.0020585227757692337
  y: -0.0002210639213444665
  z: -0.03965679183602333
  w: 0.9975232481956482
orientation_covariance: "<array type: float64[9, length: 9>"
angular_velocity:
  x: -0.0010657600359991193
  y: -0.0026644000317901373
  z: -0.0002664400089997798
angular_velocity_covariance: "<array type: float64[9, length: 9>"
linear_acceleration:
  x: -0.06818834692239761
  y: -0.020336873829364777
  z: 9.792803764343262
linear_acceleration_covariance: "<array type: float64[9, length: 9>"
```
