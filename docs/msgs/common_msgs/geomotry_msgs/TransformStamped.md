<!--
 * @Author: code-fusheng 2561035977@qq.com
 * @Date: 2023-09-25 15:10:20
-->

> 消息结构体格式

```shell
$ rosmsg show geometry_msgs/TransformStamped
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/Transform transform
  geometry_msgs/Vector3 translation     # 平移(三轴)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation     # 旋转(四元数)
    float64 x
    float64 y
    float64 z
    float64 w
```

```shell

```
