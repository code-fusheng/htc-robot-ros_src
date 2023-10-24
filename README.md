# robot-robot-ros_src

> ROS 机器人工程开发手册

### 一、实践部分

#### 1. 工程管理

##### 0. 系统初始化

```shell
# 安装中文输入法 ibus
sudo apt-get install ibus-pinyin
# 重启
ibus-setup
```

##### 1. 工程初始化

```shell
# 安装 ssh
sudo apt install -y openssh-server


# 安装ros环境
wget http://fishros.com/install -O fishros && . fishros
```

```shell
# 创建工作空间
$ mkdir -p ~/htc-robot-ros_ws/src
$ cd ~/htc-robot-ros_ws/src
# 初始化工作空间
$ catkin_init_workspace
# 返回上级目录(在src的同级目录进行编译)
$ cd ..
# 编译(先空编译一次,会生成devel等文件)
$ catkin_make
# 设置环境变量 source ~/htc-robot-ros_ws/devel/setup.bash
$ vim ~/.bashrc
# + source /opt/ros/noetic/setup.bash
# + source ~/htc-robot-ros_ws/devel/setup.bash
$ source ~/.bashrc
# 查看环境变量
$ echo $ROS_PACKAGE_PATH
# 编译单独功能包
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="package_name"
# --source src/simulation/
# 自定义编译线程
$ catkin_make -j -l
# 清除旧的构建文件
$ catkin_make clean
```

##### 2. 工程调试

```shell
# 安装依赖

$ rosdep install --from-paths src --ignore-src -r -y
# 单独编译包
catkin_make --pkg lslidar_msgs
# https://github.com/rst-tu-dortmund/teb_local_planner.git
# apt remove
sudo apt install ros-noetic-teb-local-planner
# 修改权限
sudo chmod 755 src/turn_on_robot_htc/scripts/send_mark.py
# rqt 参数修改工具
$ rosrun rqt_reconfigure rqt_reconfigure
```

##### 3. 工程开发

```cpp
// 节点句柄
ros::NodeHandle nh;
// 是否有对应参数服务器
nh.hasParam("<PARAM_NAME>")
// argv 默认会有程序名称参数 => 可以用来判断是否有参数

// 坐标转换 x、y、z 转换 四元数
tf2::Quaternion q;
q.setRPY(0, 0, msg->theta);
```

##### 4. 工程化备忘

- [ ] 通过 RQT 做界面配置，结合参数服务器

#### 2. 设备管理

> 设备适配清单

- IMU
  N100、MPU6050
- Lidar
  Ls_N10P、Ls_C16
- Camera
  Realsense_D435i、Astra_S
- Car
  wheeltec、

> 固定外设串口号

```shell
# 终端查看设备串口
$ ll /dev
```

1. 确保设备已经连接到计算机上，并且确定设备文件路径，例如 /dev/ttyUSB0

```shell
# Intel D435i 相机串口
$ lsusb
Bus 001 Device 006: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i
```

2. 创建规则文件，例如 htc-robot-camera.rules

3. 在规则文件中添加内容

模版

```shell
KERNEL=="ttyUSB*", ATTRS{idVendor}=="<YOUR_VENDOR_ID>", ATTRS{idProduct}=="<YOUR_PRODUCT_ID>", SYMLINK+="ttyACM%n", OWNER="<YOUR_USERNAME>"
```

参数说明

```shell
KERNEL=="ttyUSB*": 指定设备的内核名称以匹配所有 ttyUSB 开头的串口设备
ATTRS{idVendor}=="8086": 指定设备的供应商 ID 为"8086"，用于唯一标识设备的制造商
ATTRS{idProduct}=="0b3a": 指定设备的产品 ID 为"0b3a"，用于唯一标识设备的产品类型
ATTRS{serial}=="0001": 指定设备的序列号为"0001"，用于唯一标识设备的序列号
MODE:="0777": 设置设备文件权限为 0777，允许任何用户对设备进行读写操作
GROUP:="dialout": 将设备文件的所有权组设置为"dialout"，使"dialout"组的用户具有对设备的访问权限
SYMLINK+="htc_camera": 创建一个名为"htc_camera"的符号链接，指向与匹配的设备文件相关联
\> /etc/udev/rules.d/htc_camera.rules: 将规则写入/etc/udev/rules.d/目录下的 htc_camera.rules 文件中。
```

```shell
# 示例&解析
KERNEL=="ttyUSB*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", SYMLINK+="intel_d435i"
```

#### 3. 仿真测试 simulation

killall gzserver

> 常用传感器

- 里程计传感器 odom
- 姿态传感器 imu
- 摄像头 image/camera
- 激光雷达 scan

#### 4. 建图&保存&编辑

> 地图保存 map_server

http://wiki.ros.org/map_server

```shell
# 保存地图
rosrun map_server map_saver -f <MAP_NAME>
ll
# => test_map.pgm & test_map.yaml
# 加载地图
rosrun map_server map_server test_map.yaml
```

#### \*. 常用指令

##### topic 订阅常用指令

```shell
$ rostopic echo :	打印话题信息
$ rostopic hz		:	话题频率
$ rostopic info	:	话题信息
$ rostopic list	:	列举话题
$ rostopic pub	:	往话题输入信息
$ rostopic bw		:	话题带宽
$ rostopic find	: 从数据类型查找话题
$ rostopic type	: 查看话题的数据类型
```

> 查看 topic 的消息流程示例(eg:/odom)

```shell
$ rostopic list
# -> /odom
$ rostopic type /odom
# -> nav_msgs/Odometry
$ rosmsg info nav_msgs/Odometry
# -> ...
```

##### msg 消息常用指令

```shell
$ rosmsg list
$ rosmsg show
$ rosmsg type|topic_name|rosmsg show
```

##### pkg 包常用指令

```shell
# roscd <PACKAGE_NAME>
$ roscd roscpp
# - /opt/ros/noetic/share/roscpp
# 查找
rospack find <PACKAGE_NAME>
```

##### tf 树

```shell
$ rosrun rqt_tf_tree rqt_tf_tree
```

---

### 二、Ros 理论部分

#### 1. Node 节点

> 常用指令（ros1）

```
$ rosnode list
```

> 常用指令（ros2）

```shell
# 运行节点
$ ros2 run <package_name> <executable_name>
# 查看节点列表
$ ros2 node list
# 查看节点信息(常用)
$ ros2 node info <node_name>
# 重映射节点名称
$ ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
# 运行节点时设置参数
$ ros2 run example_parameters_rclcpp parameters_basic --ros-args -p rcl_log_level:=10
```

#### 2. package 功能包

> 创建功能包

```shell
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
$ catkin_create_pkg new_package std_msgs rospy roscpp
```

##### 2.1. CMakeLists.txt 编译规则文件

```
# 常用规则
cmake_minimum_required()	CMake版本
project()									项目名称
find_package()						添加依赖
catkin_package()					编译生成信息导出
add_executable()					生成可执行文件
target_link_libraries()		可执行文件链接到库
```

```
# 其他规则
add_message_files()				添加话题消息类型文件
add_service_files()				添加服务消息类型文件
add_action_files()				添加行为消息类型文件
generate_messages()				生成消息文件
include_directories()			添加头文件路径
add_dependencies()				添加依赖项
add_library()							生成自定义的库
install()									安装规则
```

##### 2.2. package.xml 功能包描述文件

##### 2.3. launch 脚本文件

> 根标签

```
<launch>
...
</launch>
```

> 嵌套

```
<includ>
...
</includ>
```

> 节点

模版

```xml
# rosrun <PACKAGE_NAME> <TYPE>
<node pkg="PACKAGE_NAME" type="TYPE" name="NODE_NAME"/>
```

属性

| 属性     | 作用                                                     |
| -------- | -------------------------------------------------------- |
| pkg      | 节点所在的包名                                           |
| type     | 可执行文件名                                             |
| name     | 重定义节点名                                             |
| args     | 传递参数                                                 |
| output   | 日志输出（log\|screen）                                  |
| respawn  | "true"，如果节点失效则重启（默认：false）                |
| required | "true"，如果节点失效则关闭整个 launch 文件（默认不设置） |
| ns       | 在命名空间中运行此节点                                   |

> 参数

参数区别

```xml
# 在参数服务器添加一个参数
<param>
# 从 ".yaml" 文件中一次性导入大量参数
<rosparam>
# 在 launch 中声明一个参数
<arg>
```

参数服务器常用指令

```shell
$ rosparam list			: 列出参数服务器中的参数
$ rosparam get			: 获取参数
$ rosparam set			: 设置参数
$ rosparam delete		: 删除参数
$ rosparam load			: 从文件中夹在参数到参数服务器
$ rosparam dump			: 将参数服务器的参数写入文件
```

节点分组

```xml
# 若干节点划分进同一命名空间
<group ns="group_name">
...
</group>
# 条件判断执行
<group if="condition">
...
</group>
```

重命名

```shell
<remap from="/old_topic" to="/new_topic">
```

#### 3. msgs 消息包

- std_msgs: 标准消息包 https://wiki.ros.org/std_msgs

- common_msg: 常规消息包 http://wiki.ros.org/common_msgs

  - sensor_msgs: 传感器消息包
  - geomotry_msgs: 几何消息包
    - Twist
  - nav_msgs: 导航消息包
  - stereo_msgs: 双目视觉消息包

#### 4. TF 坐标

TF 树
TF 的 broadcaster
TF 的 frame_id
TF 的基坐标
TF 的消息数据类型

> 常用的 TF 坐标系命名规则与约束

固定坐标系(World Frame): "world" 或 "map"， 表示机器人周围的固定参考坐标系
机器人坐标系(Robot Frame): "base_link" 或 "base_footprint" 或 <机器人名称>，表示机器人本身的坐标系
小车底盘坐标系(Base Frame): "base_link" 或 "base_footprint"，与机器人类似，表示机器人底盘或移动平台坐标

PS:在机器人系统中，同时存在 "base_link" 和 "base_footprint" 这两个名称的坐标系，通常是为了区分机器人底盘的不同表示方式或提供更灵活的坐标变换选项。
"base_link"：这个坐标系通常表示机器人底盘的中心或固定位置。它可能与机器人的物理结构直接相关，作为机器人坐标系的一部分，用于表示机器人整体的位置和姿态。在一些应用中，"base_link" 可以与其他感知设备（如雷达、相机）的坐标系进行转换，以便获取相对于机器人底盘的测量数据。
"base_footprint"：这个坐标系通常表示机器人底盘的地面投影。它可以看作是机器人底盘在地面上的二维表示，忽略了底盘的高度信息。使用 "base_footprint" 可以简化定位和避障等任务中的计算，并且在机器人移动时保持相对稳定。通常，"base_footprint" 与 "base_link" 之间的变换可以包括底盘高度的偏移。

查看 tf

```shell
rostopic type /tf
# tf2_msgs/TFMessage
```

TF 广播器 与 TF 监听器

```cpp
// TF 广播器
static tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStampled transformStampled;
...
transformStamped.xxx = xxx;
...
br.sendTransform(transformStamped);

// TF 监听器
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListenser tfListener(tfBuffer);
geometry_msgs::TransormStamped transformStamped;
transformStamped = tfBuffer.lookupTransform("target", "source", time);
```

#### 5. CMake

> REQUIRED`和`QUIET`是与`find_package

1. `REQUIRED`：这个选项用于指示指定的包对项目来说是必需的。如果找不到该包，CMake 将生成一个错误并停止配置过程。它确保该包必须可用以成功进行配置。
2. `QUIET`：这个选项用于抑制`find_package`命令在找不到包时可能生成的任何错误或警告消息。如果找不到该包，CMake 将只是将变量`<package>_FOUND`设置为`FALSE`，而不生成错误。

- 使用`REQUIRED`使包成为强制性的，如果找不到该包，CMake 将生成错误。
- 使用`QUIET`可以抑制找不到包时生成的任何错误或警告消息，让你自己处理这种情况。

---

### 三、机器人理论部分

**感知 决策 控制**
感知部分有：激光雷达、深度相机、IMU、里程计、碰撞感知、建图
决策部分有：路径规划（navigation）算法、定位算法
控制部分有：轮子驱动

#### 0. 底盘

目前机器人底盘一般都有 VCU(微控制器) 进行控制
一般用 stm32 的主板进行相关的控制
云乐的底盘应该是德州仪器的一款车规级板子

#### 1. 地图

MAP.pgm : 便携式灰度图(灰度图 0-255)

> costmap 代价地图

- footpinrt: 小车轮廓
- Inflation Layer 膨胀层
  根据 inflation_radiu(膨胀半径)参数，膨胀障碍物
- Static Layer 静态地图层
  接收 /map 话题信息，加载的地图文件
- Obstacle Layer 障碍物层
  接收雷达等信息，实时检测环境障碍物 observation_sources(观测源)

Master
^
Inflation
^
Obstacles
^
Static

> 占据栅格地图
> 占据点云地图
> 路标特征地图
> 语义地图 => 点云聚类分割

#### 2. SLAM

经典 SLAM 框架

1. 传感器信息读取
2. 里程计
3. 数据接收优化 => 全局一致的轨迹与地图
4. 回环检测
5. 建图

#### 3. 建图

> GMapping（Grid-based FastSLAM）：

优点：采用网格地图表示，适用于动态环境下的实时建图。具有良好的扩展性和鲁棒性。
缺点：对于大规模环境可能需要高计算资源，且在存在闭环时可能出现地图漂移。

> Hector SLAM：

优点：采用栅格地图表示，结合快速扫描匹配和惯性测量单元（IMU）信息，适用于实时性要求较高的应用。对运动畸变有较好的抵抗能力。
缺点：对于大规模环境建图可能会出现累积误差，局限于使用激光雷达作为主要传感器。

> Cartographer：

优点：采用分层位姿图表示，同时利用激光雷达和 IMU 数据，适用于大规模、复杂环境和高精度要求。具备全局一致性和高度优化的地图。
缺点：对计算资源要求较高，对传感器同步和校准要求较严格。

> Karto（OpenKarto）：

优点：采用栅格地图表示，轻量级算法，适用于小型机器人和嵌入式设备。具有快速建图和定位能力。
缺点：对环境的动态变化敏感，对于大规模环境和闭环检测支持较弱。

> ORB-SLAM：

优点：基于特征点的视觉 SLAM 算法，适用于单目、双目以及 RGB-D 相机。具备较好的实时性和鲁棒性，并支持闭环检测和重定位。
缺点：对于纹理缺乏明显的场景或低纯度特征存在挑战。

> LSD-SLAM：

优点：使用直接法（direct method）进行稀疏深度估计的视觉 SLAM 算法。能够处理纹理缺乏或运动模糊等情况下的建图与定位。
缺点：对计算资源要求较高，对于大规模环境可能会出现累积误差。

> RTAB-Map：

优点：基于图优化的 SLAM 算法，支持激光雷达、RGB-D 相机和双目相机等多种传感器。能够在室内和室外环境中进行建图和定位。
缺点：对计算资源和存储需求较高，对于大规模环境需要更多的计算时间。

> LIO-SLAM:

https://github.com/TixiaoShan/LIO-SAM

> Rtabmap_ros

http://wiki.ros.org/rtabmap_ros

```shell
# 官方示例
$ roslaunch rtabmap_demos demo_robot_mapping.launch rviz:=true rtabmapviz:=false
$ rosbag play --clock demo_mapping.bag
```

#### 4. 导航

##### 1. Navigation 工作框架

http://wiki.ros.org/navigation

> 功能包

amcl: 协助定位
move_base: 订阅目标位置、发布速度命令

###### caml 功能包

> caml 主要作用是补偿定位累计误差

订阅小车位置(TF 坐标 base_footprint)和雷达信息，计算小车在地图上的位置偏差，然后发布 map 到 odom_combined 的 TF 转换

acml 根据雷达信息计算 base_footprint 在 map 上的位置误差，再通过在 map 和 base_footprint 中添加一个 TF: odom_combined 来进行误差补偿

###### move_base 功能包

http://wiki.ros.org/move_base

###### 插件

> 路径规划器

base_global_planner
base_local_planner
recovery_behavior

> 代价地图

costmap

###### move_base

目标位置: simple_goal
传感器: scan、pointcloud
速度: cmd_vel
tf: map、base_footprint

> 参数配置文件

- move_base_params.yaml
  选择路径规划器
- dwa_local_planner_params.yaml、teb_local_planner_params.yaml
  不同局部路径规划器的参数配置文件
- base_global_planner_params.yaml、navfn_global_planner_params.yaml
  不同全局路径规划器的参数配置文件
- local_costmap_params.yaml
  局部代价地图配置文件: tf 树、地图大小、地图层插件配置
- global_costmap_params.yaml
  全局代价地图配置文件: tf 树、地图大小、地图层插件配置
- costmap_common_params.yaml
  公共代价地图配置文件: 小车外形、地图层详细配置

#### 5. 仿真

[CARLA simulator](https://github.com/carla-simulator/carla)

#### \*. 算法理论

##### 自适应蒙特卡洛定位 AMCL

##### 粒子滤波

初始化 => 计算权重 => 重采样 => 状态转移 => 计算权重 => ...

[B 站演示 粒子滤波](https://www.bilibili.com/video/BV1DW411h7yM/?p=2&spm_id_from=pageDriver&vd_source=929a35de55acb306c77f01d37d1507a1)

##### 局部路径规划 DWA 算法

```
# 算法流程伪代码
初始化 —— 小车最大、最小速度、加速度、评价函数权重等
循环
{
  判断是是否到达目地点
  计算当前采样的速度范围（动态窗口）
  遍历所有速度 v & w，根据模型模拟一段时间的路径
  根据评价函数打分（评价函数、归一化、权重）
  选取最优解 —— v & w，下发给运动底盘
  小车移动
}
```

**状态采样**
**速度采样**
速度如何采样：
速度范围？
限制？

1. 小车数独最大值、最小值
2. 电机性能影响加速度
3. 预留刹车距离

轨迹好坏的判断 —— 评价函数

##### 局部路径规划 Teb 算法

```shell
# 官方演示脚本
$ roslaunch teb_local_planner test_optim_node.launch
```

##### 通用图优化法 g2o

---

### 四、操作手册

> 基础底盘控制

```shell
# 好停车轮趣小车基础底盘控制指令
# IMU : /imu
# 电压 : /PowerVoltage
# 里程计 : /odom
$ rosrun turn_on_robot_htc htc_wheeltec_robot_node
$ roslaunch turn_on_robot_htc turn_on_robot_htc.launch
# 行驶控制 topic: cmd_vel
$ rosrun rqt_robot_steering rqt_robot_steering
$ rosrun turtlebot3_teleop turtlebot3_teleop_key
```
