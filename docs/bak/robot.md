# cd /home/htc/桌面/zh-ws

### 学习参考资料

https://fishros.com/d2lros2/#/

### 教程说明

python >> .py 文件能够直接运行
c++ >> .cpp 文件需要编译才能运行

#### Ros2 安装

> 1. 一键安装

```
wget http://fishros.com/install -O fishros && . fishros
```

> 2. 手动安装

```
# 添加源
echo "deb [arch=$(dpkg --print-architecture)] https://repo.huaweicloud.com/ros2/ubuntu/ $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# 添加源密钥
sudo apt install curl gnupg2 -y
curl -s https://gitee.com/ohhuo/rosdistro/raw/master/ros.asc | sudo apt-key add -
# 更新
sudo apt update
# 安装
sudo apt install ros-humble-desktop
# 安装依赖
sudo apt install python3-argcomplete -y
# 配置环境变量
source /opt/ros/humble/setup.bash
# 配置环境++
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

> 3. 卸载

```
sudo apt remove ros-humble-*
sudo apt autoremove
```

> 4. Ros 安装目录

```
/opt/ros/foxy
```

#### Ros 初体验

> 1. Listener & Talker

```
# 启动倾听者
ros2 run demo_nodes_py listener
# 启动说话者
ros2 run demo_nodes_py talker
```

> 2. 海龟

```
# 海龟模拟器
ros2 run turtlesim turtlesim_node
# 海龟遥控器
ros2 run turtlesim turtle_teleop_key
```

> 3. RQT 可视化

```
rqt
# Introspection / Node Graph
```

#### Ros 常用指令

> 1. Node 节点

```
# 运行节点
ros2 run <package_name> <executable_name>
# 查看节点列表
ros2 node list
# 查看节点信息(常用)
ros2 node info <node_name>
# 重映射节点名称
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
# 运行节点时设置参数
ros2 run example_parameters_rclcpp parameters_basic --ros-args -p rcl_log_level:=10

```

> 2. Pkg 功能包

```
create       Create a new ROS2 package
executables  Output a list of package specific executables
list         Output a list of available packages
prefix       Output the prefix path of a package
xml          Output the XML of the package manifest or a specific tag
```

> 3. interface 接口

```
ros2 interface proto geometry_msgs/msg/Twist
```

### 机器人核心

#### 感知 决策 控制

感知部分有：激光雷达、深度相机、IMU、里程计、碰撞感知、建图
决策部分有：路径规划（navigation）算法、定位算法
控制部分有：轮子驱动

#### Ros 通信机制

话题、服务、参数、动作

### DDS 数据分发服务

https://blog.csdn.net/DDS_CSIT/article/details/104607476

#### Ros2 客户端库 RCL

./ros2-rcl.jpg

### 基础 - 编程基础

#### 1. C++ 编译工具 CMake

```
# 1. 安装 make
sudo apt install make
# 2. 安装 cmake
```

#### 2. Python 打包工具 Setup

python 分发打包
https://packaging.python.org/en/latest/#

#### 3. CMake 依赖查找流程

```
cmake_minimum_required(VERSION 3.16.3)
project(first_node)
find_package(rclcpp REQUIRED)
add_executable(first_node first_ros2_node.cpp)
target_link_libraries(first_node rclcpp::rclcpp)
```

#### 4. Python 依赖查找流程

echo $PYTHONPATH
/opt/ros/foxy/lib/python3.8/site-packages

### 入门 - 使用 Ros2

#### 1. ROS2 节点与工作空间以及功能包

```
# 启动节点
ros2 run <package_name> <executable_name>
```

> Ros2/cli
> https://github.com/ros2/ros2cli.git

> 工作空间
> 注意：一个工作空间下可以有多个功能包，一个功能包可以有多个节点存在

```
mkidr -p demo_ws/src
```

> 功能包
>
> 1. 安装获取

```
sudo apt install ros-<version>-package_name
```

> 2. 手动编译
>    PS:手动编译之后，需要手动 source 工作空间的 install 目录。

> 3. 包相关指令

```
create       Create a new ROS2 package
executables  Output a list of package specific executables
list         Output a list of available packages
prefix       Output the prefix path of a package
xml          Output the XML of the package manifest or a specific tag
```

```
# 创建功能包
ros2 pkg create <package-name>  --build-type  {cmake,ament_cmake,ament_python}  --dependencies <依赖名字>
# 列出所有
ros2 pkg executables
# 列出功能包所有可执行文件
ros2 pkg executables turtlesim
# 列出所有的包
ros2 pkg list
# 输出某个包所在路径的前缀
ros2 pkg prefix  <package-name>
# 列出包的清单描述文件
ros2 pkg xml turtlesim
```

#### 2. ROS2 构建工具 Colcon

> colcon 功能包构建工具
> PS：相当于 ros1 的 catkin 工具；ROS2 默认是没有安装 colcon 的

```
# 安装工具
sudo apt-get install python3-colcon-common-extensions
```

> 验证测试

```
# 创建一个工作区文件夹colcon_test_ws
cd d2lros2/chapt2/
mkdir colcon_test_ws && cd colcon_test_ws
# 下载个ROS2示例源码测试一下
git clone https://github.com/ros2/examples src/examples -b foxy
# 编译
colcon build
#
├── build
├── install
├── log
└── src
# 运行自己编译的节点
pwd - ws
source install/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
source install/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function

# 关键指令
# 只编译一个包
colcon build --packages-select YOUR_PKG_NAME
# 不编译测试单元
colcon build --packages-select YOUR_PKG_NAME  --cmake-args -DBUILD_TESTING=0
# 运行编译的包的测试
colcon test
# 允许通过更改src下的部分文件来改变install（重要）
colcon build --symlink-install

```

colcon 官方文档 https://colcon.readthedocs.io/en/released/user/installation.html
ROS2 官网文档 https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html

#### 3. ROS2 客户端库

#### 4. RCLCPP 编写节点

> 创建包

```
ros2 pkg create example_cpp --build-type ament_cmake --dependencies rclcpp
```

- pkg create 创建包的意思
- --build-type 用来指定该包的编译类型: ament_python \ ament_cmake \ cmake
- --dependencies 指定功能包的依赖

> 创建 node 节点 编写代码

```
# 修改 CmakeLists

add_executable(node_01 src/node_01.cpp)
ament_target_dependencies(node_01 rclcpp)

install(TARGETS
  node_01
  DESTINATION lib/${PROJECT_NAME}
)

```

> 编译运行节点

```
# 编译节点
colcon build
# source 环境
source install/setup.bash
# 运行节点
ros2 run example_cpp node_01
```

#### 5. RCLPY 编写节点

> 创建包

```
ros2 pkg create example_py --build-type ament_python --dependencies rclpy
```

> 编写程序
> 一般步骤

- 1. 导入库文件
- 2. 初始化客户端库
- 3. 新建节点
- 4. spin 循环节点
- 5. 关闭客户端库

```
# 编写代码 修改 setup.py
    entry_points={
        'console_scripts': [
            "node_02 = example_py.node_02:main"
        ],
    },
```

> 编译运行节点

```
# 编译节点
colcon build
# source 环境
source install/setup.bash
# 运行节点
ros2 run example_py node_02
```

### 进阶 - ROS2 系统

#### 1.五种不同的方式编写节点

#### \* 构建系统与构建工具

> Colcon 构建进阶

```
# build 构建指令参数
--packages-select : 仅生成单个或选定的包
--packages-up-to : 构建选定的包，包括其依赖项
--packages-above : 整个工作区，对其一个包进行更改，重构（递归）依赖于此包的所有包
# 构建后安装目录
--build-base : 构建目录
--install-base : 安装目录
```

### 通信 - 话题与服务

#### 1. 话题理解与入门

> RQT 工具 rqt_graph

```
rqt_graph
```

> Cli 工具

```
ros2 topic -h
# 活动主题列表 -t [消息类型]
ros2 topic list -t
# 打印话题内容
ros2 topic echo /<MEG_NAME>
# 打印主题信息
ros2 topic info /<MSG_NAME>
- Type:
- P count:
- S count:
# 查看消息类型
ros2 interface show std_msgs/msg/String
# 手动发布命令
ros2 topic pub /chatter std_msgs/msg/String 'data: "123"'
```

#### 2. 话题与服务的实现

> rclcpp

```
# 创建工作空间
mkdir -p demo3_ws/src
# 创建包
ros2 pkg create example_topic_rclcpp --build-type ament_cmake --dependencies rclcpp
# 创建节点
touch example_topic_rclcpp/src/topic_publisher_01.cpp
```

编写代码

```
******
```

修改 CMakeLists.txt

```
add_executable(topic_publisher_01 src/topic_publisher_01.cpp)
ament_target_dependencies(topic_publisher_01 rclcpp)

install(
  TARGETS
  topic_publisher_01
  DESTINATION lib/${PROJECT_NAME}
)
```

编译运行

```
cd demo3_ws
colcon build --packages-select example_topic_rclcpp
source install/setup.bash
ros2 run example_topic_rclcpp topic_publisher_01
```

编写发布者
发布者 API https://docs.ros2.org/latest/api/rclcpp/
导入消息接口
ament_cmake 类型功能包导入消息接口的步骤：

1. CMakeLists.txt

```
find_package(std_msgs REQUIRED)
```

2. packages.xml

```
<depend>std_msgs</depend>
```

3. \*\*.cpp `#include "xxx/xxx.hpp"`

定时器 API 发布数据

运行测试

```
colcon build --packages-select example_topic_rclcpp
source install/setup.bash
ros2 run example_topic_rclcpp topic_publisher_01
```

编写订阅者

1. CMakeLists.txt

运行测试

```
colcon build --packages-select example_topic_rclcpp
source install/setup.bash
ros2 run example_topic_rclcpp topic_subscribe_01
```

> rclpy

```
# 创建功能包
ros2 pkg create example_topic_rclpy --build-type ament_python --dependencies rclpy
# 创建节点文件
touch example_topic_rclpy/example_topic_rclpy/topic_subscribe_02.py
touch example_topic_rclpy/example_topic_rclpy/topic_publisher_02.py
# 编写代码
****
# 发布节点
cd demo_ws
colcon build
source install/setup.bash
ros2 run example_topic_rclpy topic_subscribe_02
ros2 run example_topic_rclpy topic_publisher_02
```

#### 3.ROS2 服务

```
# 查看服务列表
ros2 service list
```

```cpp
# 创建功能包和节点
ros2 pkg create example_service_rclcpp --build-type ament_cmake --dependencies rclcpp
colcon build --packages-select example_service_rclcpp
source install/setup.bash
```

```python
# 创建功能包和节点
ros2 pkg create example_service_rclpy --build-type ament_python --dependencies rclpy example_interfaces  --node-name service_server_02
# PS --node-name 只支持创建一个节点文件
touch example_service_rclpy/example_service_rclpy/service_client_02.py
```

#### 4. ROS2 接口

> 激光雷达数据接口：sensor_msgs/msg/LaserScan

```
# 查看某一个接口包下所有的接口
ros2 interface package sensor_msgs
sensor_msgs/msg/JointState  #机器人关节数据
sensor_msgs/msg/Temperature #温度数据
sensor_msgs/msg/Imu #加速度传感器
sensor_msgs/msg/Image #图像
sensor_msgs/msg/LaserScan #雷达数据
```

> 自定义接口
> 话题、服务和动作(Action)都支持自定义接口

> 接口形式
> 话题接口格式：xxx.msg
> int64 num

服务接口格式：xxx.srv
int64 a
int64 b

---

int64 sum

动作接口格式：xxx.action
int32 order

---

## int32[] sequence

int32[] partial_sequence

> 接口数据类型
> 基础类型

```
bool
byte
char
float32,float64
int8,uint8
int16,uint16
int32,uint32
int64,uint64
string
```

包装类型

```
uint32 id
string image_name
sensor_msgs/Image
```

自定义接口场景

> 服务接口 MoveRobot.srv

```
# 前进后退的距离
float32 distance
---
# 当前的位置
float32 pose
```

> 话题接口（基础类型） RobotStatus.msg

```
uint32 STATUS_MOVEING = 1
uint32 STATUS_STOP = 1
uint32  status
float32 pose
```

> 话题接口（混合包装类型）RobotPose.msg

```
uint32 STATUS_MOVEING = 1
uint32 STATUS_STOP = 2
uint32  status
geometry_msgs/Pose pose
```

创建功能包接口

```
ros2 pkg create example_ros2_interfaces --build-type ament_cmake --dependencies rosidl_default_generators geometry_msgs
```

注意功能包类型必须为：ament_cmake

```
# 查看接口列表
ros2 interface list
# 查看接口详细
ros2 interface show std_msgs/msg/String
```

#### 自定义接口 RCLPY

example_interfaces_robot_02，机器人节点，对外提供控制机器人移动服务并发布机器人的状态。
example_interfaces_control_02，控制节点，发送机器人移动请求，订阅机器人状态话题。

```
ros2 pkg create example_interfaces_rclpy --build-type ament_python --dependencies rclpy example_ros2_interfaces --destination-directory src --node-name example_interfaces_robot_02 --maintainer-name "fishros" --maintainer-email "fishros@foxmail.com"
touch src/example_interfaces_rclpy/example_interfaces_rclpy/example_interfaces_control_02.py
```

#### 3. 数据与数据类型

原始的数据类型只有九类

```
bool
byte
char
float32, float64
int8, uint8
int16, uint16
int32, uint32
int64, uint64
string
```

QOS： https://mp.weixin.qq.com/s/J63fO4c_QIseLGQd5W2fAw

FastDDS

```
mkdir -p fastdds_ws/src
wget https://downloads.gradle-dn.com/distributions/gradle-6.4-bin.zip && unzip gradle-6.4-bin.zip
wget http://fishros.com/tools/files/fastrtps.repos && vcs import src < fastrtps.repos
```

### 控制概述

开关控制与闭环控制

### 参数与动作

```
# 查看所有节点的参数列表
ros2 param list
# 查看参数详细信息
ros2 param describe <node_name> <param_name>
ros2 param describe /turtlesim background_b
# 非存储修改
# 查看参数值
ros2 param get /turtlesim background_b
# 设置
ros2 param set /turtlesim background_b 10
# 参数存储
ros2 param dump <node_name>
# 参数加载
ros2 param load  /turtlesim ./turtlesim.yaml
# 启动节点时加载参数快照
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
```

#### 1. 参数 RCLCPP 实现

日志级别

```
enum RCUTILS_LOG_SEVERITY
{
  RCUTILS_LOG_SEVERITY_UNSET = 0,  ///< The unset log level
  RCUTILS_LOG_SEVERITY_DEBUG = 10,  ///< The debug log level
  RCUTILS_LOG_SEVERITY_INFO = 20,  ///< The info log level
  RCUTILS_LOG_SEVERITY_WARN = 30,  ///< The warn log level
  RCUTILS_LOG_SEVERITY_ERROR = 40,  ///< The error log level
  RCUTILS_LOG_SEVERITY_FATAL = 50,  ///< The fatal log level
};
```

#### 2. 参数 RCLPY

```
ros2 run example_parameters_rclpy parameters_basic --ros-args -p rcl_log_level:=10
```

### 动作 Action 通信与自定义

目标：即 Action 客户端告诉服务端要做什么，服务端针对该目标要有响应。解决了不能确认服务端接收并处理目标问题
反馈：即 Action 服务端告诉客户端此时做的进度如何（类似与工作汇报）。解决执行过程中没有反馈问题
结果：即 Action 服务端最终告诉客户端其执行结果，结果最后返回，用于表示任务最终执行情况。

参数是由服务构建出来了，而 Action 是由话题和服务共同构建出来的（一个 Action = 三个服务+两个话题） 三个服务分别是：1.目标传递服务 2.结果传递服务 3.取消执行服务 两个话题：1.反馈话题（服务发布，客户端订阅） 2.状态话题（服务端发布，客户端订阅）

```
# 查看系统 action 列表
# -t 显示参数类型
ros2 action list -t
- /turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
#
ros2 interface show turtlesim/action/RotateAbsolute
# 查看 action 信息 action客户端和服务段的数量以及名字
ros2 action info /turtle1/rotate_absolute
# action 模拟请求 action send_goal
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.6}" --feedback
```

> 自定义通信接口
> 控制机器人移动到点

```
# 创建功能包
ros2 pkg create robot_control_interfaces --build-type ament_cmake --destination-directory src --maintainer-name "fishros" --maintainer-email "fishros@foxmail.com"
# 创建action接口文件
mkdir -p src/robot_control_interfaces/action
touch src/robot_control_interfaces/action/MoveRobot.action
# 配置 + 编写代码
****
# 编译生成接口
colcon build --packages-select robot_control_interfaces
```

#### 动作 CPP 实现

https://fishros.com/d2lros2/#/humble/chapt4/get_started/5.%E5%8A%A8%E4%BD%9C%E4%B9%8BCPP%E5%AE%9E%E7%8E%B0

1. 创建功能包和节点

2. 编写机器人类

3. 编写机器人节点

4. 编写控制节点

5. 编译测试

### 生命周期节点介绍

ROS2 生命周期节点是利用状态机构成的，状态直接的转换依靠 ROS2 的通信机制完成。

生命周期节点主要有以下几个状态

未配置状态（Unconfigured） ，节点开始时的第一个状态，并在出现错误后结束。没有执行，其主要目的是错误恢复。
非活跃状态（Inactivate），节点持有资源（发布者、监听者等）和配置（参数、内部变量），但什么也不做。 没有执行，没有传输，传入的数据可以保存在缓冲区中，但不能读取， 主要目的是允许重新配置。
活跃状态（Activate）， 正常执行。
已完成状态（Finalized），节点已被销毁。

### 常用工具

#### Launch 启动工具

创建功能包和 launch

#### Ros2 命令行工具

ros2 -h

#### Rviz 数据可视化工具

https://fishros.com/d2lros2/#/humble/chapt5/get_started/3.%E6%95%B0%E6%8D%AE%E5%8F%AF%E8%A7%86%E5%8C%96%E5%B7%A5%E5%85%B7-RVIZ

rviz2

#### Rqt 调试工具

RQT 插件

#### rosbag 记录工具

#### 2.在同一个进程组织多个节点

#### 3.生命周期节点介绍

#### 4.ROS2 节点发现机制原理

#### 5.Colcon 编译原理与进阶使用

#### 6.ROS2 包运行原理

#### 7.ROS2 客户端库源码导读

## 运动学基础

一、矩阵与矩阵运算

二、miniconda 与 jupyer
https://docs.conda.io/en/latest/miniconda.html
https://repo.anaconda.com/miniconda/Miniconda3-py38_23.5.2-0-Linux-x86_64.sh

https://fishros.com/d2lros2/#/humble/chapt6/basic/2.MiniConda%E4%B8%8EJupyter%E4%BB%8B%E7%BB%8D%E5%AE%89%E8%A3%85

https://docs.conda.io/en/latest/miniconda.html
bash Miniconda3-py38_4.10.3-Linux-x86_64.sh
cd ~/miniconda3/bin
./conda init
conda create -n ros2 python=<3.X>
conda activate ros2

pip3 install jupyter -i https://pypi.tuna.tsinghua.edu.cn/simple
jupyter-notebook

三、矩阵计算 Numpy

### 机器人运动学

位姿
表示两个坐标系之间的位姿关系，比如位置可以表示坐标系{A}和坐标系{B}原点位置关系，姿态可以表示两个坐标系坐标轴的朝向关系

姿态的三种表示方式：

1. 旋转矩阵-在位姿描述一节中
2. 坐标轴旋转-绕 xyz 轴旋转不同的角度(欧拉角)
3. 四元数-ROS2 的 TF2 中的姿态描述

坐标描述的三类五种：

- 旋转矩阵 - 旋转矩阵
- 坐标轴旋转 - 固定轴欧拉角，非固定轴欧拉角

常用坐标转换：

- 固定角 与 四元数 互转
- 固定角 与 旋转矩阵 互转
- 四元数 与 旋转矩阵 互转

PS： 旋转矩阵一般记作 R\_\*

姿态转换实战 - transforms3d

```
pip install transforms3d -i https://pypi.tuna.tsinghua.edu.cn/simple
```

https://fishros.com/d2lros2/#/humble/chapt6/get_started/6.%E9%BD%90%E6%AC%A1%E5%9D%90%E6%A0%87%E5%8F%98%E6%8D%A2%E5%AE%9E%E6%88%98

角度:
影响机器人当前角度的因素只有一个，就是角速度。
某一时刻机器人转动的角度 = 这一时刻机器人的角速度\*这一时刻时长
假如我们认定初始时刻机器人的角度为 0,通过对机器人转动角度角度进行累加，即可获得机器人的当前角度。
上述过程其实就是对角速度进行积分得到角度。
位置:
通过对角速度积分，我们得到了角度。
机器人某一时刻自身方向上的前进速度可以分解为里程计坐标系中 x 轴和 y 轴方向上的速度。

### TF2 坐标变换工具

### 机器人建模

URDF 统一机器人建模语言

建立机器人描述功能包
建立 urdf 文件夹编写 urdf 文件
建立 launch 文件夹，编写 launch 文件
修改 setup.py 配置，编译测试

sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-robot-state-publisher

```
# 创建包
ros2 pkg create fishbot_description --build-type ament_python
# 创建 urdf
mkdir urdf
cd urdf
touch fishbot_base.urdf
# 创建 launch 文件
mkdir launch
cd launch
touch display_rviz2.launch.py
```

机器人 URDF 模型注入物理属性
有多重，
有多大的惯性
重心在哪
碰撞边界在哪
关节的上下界限
其他的一些必要信息等等

> 碰撞检测
> collision 可以包含的子标签如下：

origin，表示碰撞体的中心位姿
geometry，用于表示用于碰撞检测的几何形状
material，可选的，描述碰撞几何体的材料(这个设置可以在 gazebo 仿真时通过 view 选项看到碰撞包围体的形状)

```
    <collision>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.12" radius="0.10"/>
      </geometry>
      <material name="blue">
          <color rgba="0.1 0.1 1.0 0.5" />
      </material>
    </collision>
```

> 旋转惯量
> intertial 标签包含的子标签如下：

mass，描述 link 的质量
inertia，描述 link 的旋转惯量（该标签有六个属性值 ixx\ixy\ixz\iyy\iyz\izz）

```
   <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0122666" ixy="0" ixz="0" iyy="0.0122666" iyz="0" izz="0.02"/>
    </inertial>

```

URDF 仿真惯性参数不知道怎么配？快收藏，常见几何物体 URDF 分享
https://mp.weixin.qq.com/s/3L8Lilesy2W_WY5qup0gmA

> 摩擦力和刚性系数

>

### 控制指令

> 两轮差速控制器

两轮差速控制器默认通过订阅话题 cmd_vel 来获取目标线速度和角速度。该话题的类型为：geometry_msgs/msg/Twist

> 里程计
> 里程计信息默认的输出话题为 odom，其消息类型为：nav_msgs/msg/Odometry

```
ros2 interface show nav_msgs/msg/Odometry
```

header，表示该消息发布的时间
pose，表示当前机器人位置和朝向
twist，表示当前机器人的线速度和角速度

> IMU
> IMU 对应的消息类型为 sensor_msgs/msg/Imu

> 激光雷达

```
ros2 interface show  sensor_msgs/msg/LaserScan
```

cd ~/.gazebo && wget https://gitee.com/ohhuo/scripts/raw/master/gazebo_model.py && python3 gazebo_model.py

## SLAM 建图

地图分类

尺度地图
拓扑地图
语义地图

占据栅格地图就是一张写满占据率的格子组成的地图。
我们在做机器人的路径规划的时候，需要确定一个格子是有障碍物？没障碍物？还是未知呢？所以我们一般会设定两个阈值：

占据阈值（occupied_thresh），比如 0.65，则表示栅格占据率大于 0.65 的认为是有障碍物。
空闲阈值（free_thresh），比如 0.25，则表示栅格占据率小于 0.25 的认为没有障碍物。
那在 free_thresh 和 occupied_thresh 之间的则认为是未知区域（未探索）。

那又是如何解决的呢？SLAM 实现的方案很多，但是几个比较关键的技术如下：

传感器感知 通过各类传感器实现对环境的感知，比如通过激光雷达获取环境的深度信息。同时可以通过传感器融合来提高位置估计的精度，比如融合轮式里程计、IMU、雷达、深度相机数据等。

视觉/激光里程计 基本原理是通过前后数据之间对比得出机器人位置的变化。

回环检测 判断机器人是否到达之前到过的位置，可以解决位置估计误差问题，建图时可以纠正地图误差。

SLAM 算法分类
从算法的对数据的处理方式上看，目前常用的 SLAM 开源算法可以分为两类

1.基于滤波，比如扩展卡尔曼滤波（EKF: Extended Kalman Filter）、粒子滤波(PF: Particle Filter)等。

ROS 中的 gmapping、hector_slam 算法都是基于滤波实现的。

2.基于图优化，先通过传感器进行构图，然后对图进行优化。

目前比较主流的是图优化的方法，Cartographer 就是基于图优化实现的。图优化相对于滤波，不用实时的进行计算，效率更高，消耗的资源更少，所以在实际场景中使用的更多。

SLAM 开源库
4.1. Cartographer
github 地址：https://github.com/cartographer-project/cartographer

Cartographer 是一个可跨多个平台和传感器配置以 2D 和 3D 形式提供实时同时定位和建图（SLAM）的系统。

4.2. ORB_SLAM2(纯视觉)
github 地址：https://github.com/raulmur/ORB_SLAM2

ORB-SLAM2 是用于单目，双目和 RGB-D 相机的实时 SLAM 库，用于计算相机轨迹和稀疏 3D 重建

4.3 VINS
github 地址：https://github.com/HKUST-Aerial-Robotics/VINS-Mono

VINS-Mono 是单目视觉惯性系统的实时 SLAM 框架。它使用基于优化的滑动窗口配方来提供高精度的视觉惯性测距。

#### Carttographer

```
# 安装
sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros
# 源码安装
cd fishbot_ws/src
git clone https://ghproxy.com/https://github.com/ros2/cartographer.git -b ros2
git clone https://ghproxy.com/https://github.com/ros2/cartographer_ros.git -b ros2
# 安装依赖
wget http://fishros.com/install -O fishros && . fishros
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

```

```
# 创建 fishbot_cartographer

```

#### 地图

启用地图

```
ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=map/fishbot_map.yaml
```

### Nav2 导航框架

Nav2 项目继承并发扬 ROS 导航栈的精神。该项目力求以安全的方式让移动机器人从 A 点移动到 B 点。Nav2 也可以应用于其他应用，包括机器人导航，如下动态点跟踪，在这个过程中需要完成动态路径规划、计算电机的速度、避免障碍、恢复行为。

四个服务:
BT Navigator Server 导航行为树服务，通过这个大的服务来进行下面三个小服务组织和调用。
Planner Server，规划服务器，其任务是计算完成一些目标函数的路径。根据所选的命名法和算法，该路径也可以称为路线。说白了就是在地图上找路。
Controller Server，控制服务器，在 ROS 1 中也被称为局部规划器，是我们跟随全局计算路径或完成局部任务的方法。说白了就是根据找的路控制机器人走路。
Recovery Server，恢复服务器，恢复器是容错系统的支柱。恢复器的目标是处理系统的未知状况或故障状况并自主处理这些状况。说白了就是机器人可能出现意外的时候想办法让其正常，比如走着走着掉坑如何爬出来。

两大代价（成本）地图：
全局代价地图 （Global Costmap）
局部代价地图（Local Costmap）

航点跟随
nav2_waypoint_follower 软件包含一个航路点跟踪程序，

状态估计（重要组件）

> Nav2 中，默认进行状态估计的组件是 AMCL (Adaptive Monte Carlo Localization)自适应蒙特卡洛定位。nav2 中对应的功能包是 nav2_amcl。

根据 ROS 社区标准，在导航项目中，需要提供两个主要的坐标转换。 map 到 odom 的坐标变换由定位系统 (定位，建图，SLAM)提供， odom 到 base_link 的坐标转换由里程计系统提供。

REP-105 标准
REP 105 定义了导航和更大的 ROS 生态系统所需的框架和约定。应始终遵循这些约定，以利用社区中丰富的定位、里程计和 SLAM 项目。

简而言之，REP-105 至少必须为机器人构造一个包含 map -> odom -> base_link -> [sensorframes] 的完整 的 TF 树。TF2 是 ROS 2 中的时变坐标变换库，Nav2 使用 TF2 来表达和获取时间同步的坐标变换。全球定位系统 (GPS、SLAM、动作捕捉 Motion Capture) 的工作是至少要提供 map-> odom 的坐标转换。然后，里程计系统的作用是提供 odom -> base_link 的坐标转化。关于 base_link 的其余坐标转换应该是静态的，并应在 URDF 中定义。

全局定位: 定位与 SLAM

里程计（Odometry）

环境表达（建模）

可以使用相机或深度传感器创建代价地图层来检测和跟踪场景中的障碍物，以避免碰撞。此外，可以创建层来基于一些规则或启发式算法来改变基础成本图。最后，它们可用于将实时数据缓冲到 2D 或 3D 世界中，以进行障碍物的二值化标记。

成本地图过滤器

```
 sudo apt install ros-foxy-nav2-*
 git clone https://ghproxy.com/https://github.com/ros-planning/navigation2.git -b foxy-devel

```

#==============控制器及其实现相关功能包======================#
nav2_controller 　｜　控制器
nav2_dwb_controller | DWB 控制器，Nav2 控制器的一个实现
nav2_regulated_pure_pursuit_controller | 纯追踪控制器，Nav2 控制器的一个实现
nav2_constrained_smoother

#==============规划器及其实现相关功能包======================#
nav2_planner | Nav2 规划器
nav2_navfn_planner 　｜　 navfn 规划器，Nav2 规划器的一个实现
nav2_smac_planner | smac 规划器，Nav2 规划器的一个实现

#=====================恢复器==============================#
nav2_recoveries | Nav2 恢复器

#=====================行为树节点及其定义====================#
nav2_bt_navigator |　导航行为树
nav2_behavior_tree | 行为树节点插件定义

#=====================地图和定位===========================#
nav2_map_server 　｜　地图服务器
nav2_costmap_2d 　｜　 2D 代价地图
nav2_voxel_grid | 体素栅格
nav2_amcl | 自适应蒙特卡洛定位。　　状态估计，输入地图、激光、里程计数据，输出机器人 map 和 odom 之间的位资关系。

#=====================通用插件系统管理等====================#
nav2_bringup | 启动入口
nav2_common 　｜　公共功能包
nav2_msgs 　｜　通信相关消息定义
nav2_util | 常用工具
nav2_lifecycle_manager |节点生命周期管理器　
nav2_rviz_plugins | RVIZ 插件

#=====================核心定义============================#
nav2_core 　｜　 Nav2 核心包
navigation2 | nav2 导航汇总配置

#=====================应用================================#
nav2_waypoint_follower | 路点跟踪

#=====================测试=================================#
nav2_system_tests | 系统测试

# PIO

快捷键
Ctrl+Alt+B 编译工程
Ctrl+Alt+U 程序上传烧录到开发板
Ctrl+Alt+S 打开串口

Arduino 和其他单片机开发，一共分为四步。
编写代码，根据相关的 API 和 SDK 进行代码的编写。
编译工程，将工程的代码文件编译成二进制文件。
烧录二进制文件，将上一步生成的二进制文件通过工具烧录到开发板中。
运行测试，重启开发板，观察硬件执行情况（数据打印一般通过串口查看）

# GPIO

输出模式：指 GPIO 是可以通过程序控制其电压高低
输入模式：指 GPIO 可以读取其上的电压

> GPIO 控制 API

1. 引脚模式 pinMode

```
void pinMode(uint8_t pin, uint8_t mode);
```

- pin : 定义 GPIO 引脚编号
- mode : 设置操作模式
  模式:
  INPUT / OUTPUT / INPUT_PULLDOWN / INPUT_PULLUP

2. 数字输出 digitalWrite

```
void digitalWrite(uint8_t pin, uint8_t val);
```

- pin:
- val: HIGH / LOW

3. 数字输入 digitalRead

```
void digitalRead(uint8_t pin);
```

# ADC 点压测量

> Arduino ADC API

1. 设置 ADC 衰减系数 analogReadResolution

2. 读取 ADC 值 analogRead

```
uint16_t analogRead(uint8_t pin);
```

3. 读取电压值 analogReadMillivolts

```
uint32_t analogReadMillivolts(uint8_t pin);
```

> 欧拉角转四元数

```
typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}
```

### 控制速度-PID 控制器实现

Proportional（比例）、Integral（积分）、Derivative（微分）。
