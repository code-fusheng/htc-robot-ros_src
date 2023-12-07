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

```
rostopic echo /ndtmapping/res
is_mapping: 2
info: ''

rostopic echo /ndtmapping/control
start_mapping: 2
save_dir: "/home/robot/htc-robot-ros_ws/data/test/t1/lidar_mode/pcd_map/static_map"
voxel_size: 1.0
step_size: 1.0
```

```
保存地图
rostopic pub /mapping/save_map
/home/robot/htc-robot-ros_ws/data/test.pcd
1.0
```

```
网格地图
rostopic pub -1 /gridmap_task_set

rostopic echo /gridmap_task_status
```
