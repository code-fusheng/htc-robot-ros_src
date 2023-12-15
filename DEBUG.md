# 设定场景 + 加载地图

```
type: automotive_msgs/ConfigMapManager
sub: /pcd_map_manager
rostopic echo /set_pcdmap_manager
dir_static_map: "/home/robot/htc-robot-ros_ws/data/test/t1/lidar_mode/pcd_map/static_map"
dir_dynamic_map: "/home/robot/htc-robot-ros_ws/data/test/t1/lidar_mode/pcd_map/dynamic_map"
```

# 雷达建图

> 开始

```
rostopic echo /ndtmapping/res
is_mapping: 1
info: ''

rostopic echo /ndtmapping/control
start_mapping: 1
save_dir: "/home/robot/htc-robot-ros_ws/data/test/t1/lidar_mode/pcd_map/static_map"
voxel_size: 1.0
step_size: 1.0
```

> 停止

```shell
# 停止建图
rostopic echo /ndtmapping/res
is_mapping: 2
info: ''
# 保存地图
rostopic echo /ndtmapping/control
start_mapping: 2
save_dir: "/home/robot/htc-robot-ros_ws/data/test/t1/lidar_mode/pcd_map/static_map"
voxel_size: 1.0
step_size: 1.0
```

```shell
# 保存地图
rostopic pub -1 /mapping/save_map automotive_msgs/SaveMap "file_res: 0.0
filename: '/home/robot/htc-robot-ros_ws/data/debug/t1/lidar_mode/pcd_map/static_map/static.pcd'"
```

# 地图分片

```shell
网格地图
rostopic pub -1 /gridmap_task_set

rostopic echo /gridmap_task_status

rostopic pub -1 /gridmap_task_set automotive_msgs/GridMapCreateConfig "source_dir: '/home/robot/htc-robot-ros_ws/data/test/t1/lidar_mode/pcd_map/static_map/'
dst_dir: '/home/robot/htc-robot-ros_ws/data/test/t1/lidar_mode/pcd_map/dynamic_map/'
voxel_size: 1.0
grid_size: 30.0"
```

https://github.com/LitoNeo/SmartCar-pcd-map-Tools.git

```
保存地图
rosservice call /online_mapping "start_mapping: 1
save_dir: '/home/robot/htc-robot-ros_ws/data/test/t1/lidar_mode/pcd_map/static_map'"
```

locazalier
rosservice call /function_switch/ndt_localization "switch_to: 0"
current_function_status: 0

rosservice call /ndt_localization/service_solution_status "is_set: true
set_solution: 1
lla_origin_latitude: 0.0
lla_origin_longitude: 0.0"
current_solution: 1

global_planner
rosservice call /function_switch/global_planning "switch_to: 0" 0

config_traj_path

```
加载路径
rostopic echo --noarr /config_traj_path
pilot_mode: 1
path_traj_path: "/home/robot/htc-robot-ros_ws/data/test/t1/lidar_mode/pathes"
```

开始 & 停止
rostopic echo --noarr /UserCmd
data: 1

data: 0



