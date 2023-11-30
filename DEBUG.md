# 设定场景 + 加载地图

```
type: automotive_msgs/ConfigMapManager
sub: /pcd_map_manager
rostopic echo /set_pcdmap_manager
dir_static_map: "/home/htc/htc-robot-ros_ws/src/source/tjbhjc/e0904x1/lidar_mode/pcd_map/static_map"
dir_dynamic_map: "/home/htc/htc-robot-ros_ws/src/source/tjbhjc/e0904x1/lidar_mode/pcd_map/dynamic_map"
```

# 雷达建图

> 开始

```
rostopic echo /ndtmapping/res
is_mapping: 1
info: ''

rostopic echo /ndtmapping/control
start_mapping: 1
save_dir: "/home/data/1/2/lidar_mode/pcd_map/static_map"
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
save_dir: "/home/data/1/2/lidar_mode/pcd_map/static_map"
voxel_size: 1.0
step_size: 1.0
```
