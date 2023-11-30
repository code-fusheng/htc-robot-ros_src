NDT Mapping 三维点云地图建立
> Build pcd map with NDT algorithm

# ndt_mapping
Use pointCloud(must)、imu/odom/twist(optional) to create pcd map.

## params
`is_filter_before_add_to_map` defines whether filter transformed_scan_ptr before added to global map, and `voxel_size_filter_before_add_to_map` 
defines the voxel size of it.

*Note: `is_filter_before_add_to_map=true` is recommended, it can hugely accelerate the mapping process, and at my test,
this can even promote the final mapping result*

## Input and Output
### Subscribe
* lidar_topic
* _imu_topic
* vehicle_odom_topic
* vehicle_twist_topic
* automotive_msgs/SaveMap.msg: trigger map-saving process, params are filter_res and filename
    * if filter_res < 0.1, then save original map

### Publish
* "/ndt_map"
* "/current_pose"
* "/ndt/history_trajectory"

# ndt_mapping_pc_only
Use only pointCloud to create global map.

by default, it will save origin map cloud to pcd, which will make it much bigger and the calculate process is much slower than ndt_mapping node with filter_before_add_to_map on.

