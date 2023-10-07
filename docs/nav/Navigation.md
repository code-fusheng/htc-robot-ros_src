# Navigation 工作框架

> 功能包

amcl: 协助定位
move_base: 订阅目标位置、发布速度命令

### caml 功能包

> caml 主要作用是补偿定位累计误差

订阅小车位置(TF 坐标 base_footprint)和雷达信息，计算小车在地图上的位置偏差，然后发布 map 到 odom_combined 的 TF 转换

acml 根据雷达信息计算 base_footprint 在 map 上的位置误差，再通过在 map 和 base_footprint 中添加一个 TF: odom_combined 来进行误差补偿

### move_base 功能包

http://wiki.ros.org/move_base

#### 插件

> 路径规划器

base_global_planner
base_local_planner
recovery_behavior

> 代价地图

costmap

#### move_base

目标位置: simple_goal
传感器: scan、pointcloud
速度: cmd_vel
tf: map、base_footprint

> 参数配置文件

move_base_params.yaml
选择路径规划器

dwa_local_planner_params.yaml、teb_local_planner_params.yaml
不同局部路径规划器的参数配置文件

base_global_planner_params.yaml、navfn_global_planner_params.yaml
不同全局路径规划器的参数配置文件

local_costmap_params.yaml
局部代价地图配置文件: tf 树、地图大小、地图层插件配置

global_costmap_params.yaml
全局代价地图配置文件: tf 树、地图大小、地图层插件配置

costmap_common_params.yaml
公共代价地图配置文件: 小车外形、地图层详细配置
