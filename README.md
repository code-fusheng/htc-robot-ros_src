# robot-robot-ros_src

## ROS 机器人工程开发

### 一、实践部分

#### 1. 工程管理

> 工程初始化

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
$ catkin_make -DCATKIN_HTC-ROBOT-ROS_WS_PACKAGES="package_name"
# --source src/simulation/
# 自定义编译线程
$ catkin_make -j -l
# 清除旧的构建文件
$ catkin_make clean
```

> 工程开发

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

#### 2. 设备管理

> 固定外设串口号

```shell
# 终端查看设备串口
$ ll /dev
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
```

##### tf 树

```shell
$ rosrun rqt_tf_tree rqt_tf_tree
```

### 二、Ros 理论部分

#### 1. 节点 Node

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

#### 2. 功能包 package

> 创建功能包

```shell
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
$ catkin_create_pkg new_package std_msgs rospy roscpp
```

##### 2.1. 编译规则文件 CMakeLists.txt

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

#### 2.2. 功能包描述文件 package.xml

#### 2.3. launch 文件

##### 根标签

```
<launch>
...
</launch>
```

##### 嵌套

```
<includ>
...
</includ>
```

##### 节点

> 模版

```xml
# rosrun <PACKAGE_NAME> <TYPE>
<node pkg="PACKAGE_NAME" type="TYPE" name="NODE_NAME"/>
```

> 属性

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

##### 参数

> 参数区别

```xml
# 在参数服务器添加一个参数
<param>
# 从 ".yaml" 文件中一次性导入大量参数
<rosparam>
# 在 launch 中声明一个参数
<arg>
```

> 参数服务器常用指令

```shell
$ rosparam list			: 列出参数服务器中的参数
$ rosparam get			: 获取参数
$ rosparam set			: 设置参数
$ rosparam delete		: 删除参数
$ rosparam load			: 从文件中夹在参数到参数服务器
$ rosparam dump			: 将参数服务器的参数写入文件
```

##### 节点分组

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

##### 重命名

```shell
<remap from="/old_topic" to="/new_topic">
```

#### 2. 坐标 TF

TF 树
TF 的 broadcaster
TF 的 frame_id
TF 的基坐标
TF 的消息数据类型

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

### \*、机器人理论部分

**感知 决策 控制**
感知部分有：激光雷达、深度相机、IMU、里程计、碰撞感知、建图
决策部分有：路径规划（navigation）算法、定位算法
控制部分有：轮子驱动

**建图**

**SLAM**
经典 SLAM 框架

1. 传感器信息读取
2. 里程计
3. 数据接收优化 => 全局一致的轨迹与地图
4. 回环检测
5. 建图

### 工程化备忘

- [ ] 通过 RQT 做界面配置，结合参数服务器

### 操作手册

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
```
