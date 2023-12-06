NDT Mapping 三维点云地图建立
> Build pcd map with NDT algorithm

# ndt_mapping
使用`pointCloud(must)、imu/odom/twist(optional)` 创建三维点云地图，以用于支持NDT(激光雷达)模式的自动驾驶。

## 参数
1. `is_filter_before_add_to_map`：设置是否在将当前点云加入全局点云地图前进行voxel降采样， `voxel_size_filter_before_add_to_map` 
设置voxel的leaf size.

> 注意: 推荐使用`is_filter_before_add_to_map=true`, 这样可以加速建图时的速度，在实际使用过程中，这有时甚至可以提高建图的效果。

## 输入和输出
### Subscribe
* `lidar_topic`
* `imu_topic`
* `vehicle_odom_topic`
* `vehicle_twist_topic`
* `/mapping/save_map type:automotive_msgs/SaveMap.msg`: 触发地图保存操作:
  * `filter_res`: 在保存地图前进行voxel降采样。如果`filter_res<0.1`则保存原始点云地图
  * `filename`: 要保存的完整路径(**绝对路径**)+名称，以`.pcd`结尾，例如`/home/data/cao_chang_voxel_0.3.pcd`

### Publish
* `/ndt_map`: 生成的全局点云地图
* `/current_pose`: 建图过程中的当前位置信息
* `/ndt/history_trajectory`: 建图过程中的历史轨迹信息

# ndt_mapping_pc_only
Use only pointCloud to create global map.

by default, it will save origin map cloud to pcd, which will make it much bigger and the calculate process is much slower than ndt_mapping node with filter_before_add_to_map on.

## 使用
1. 检查`run_ndt_mapping.launch`中的参数：
2. 运行`roslaunch ndt_mapping run_ndt_mapping.launch`启动建图流程；
3. 使用`rosbag play --clock <your-bag-file>.bag`播放数据，开始建图；
   * 追加参数`-r 0.5`来以0.5倍的速率进行bag的播放(或者设置为其他参数)。在pc性能较低时，推荐使用慢速率进行bag的播放
   * 追加参数`-s 30`以跳过bag包开头30s的数据(或者设置为其他跳过时间)。如果起始位置不平整，或者开始时静止时间较长，推荐使用`-s`以跳过部分数据。
4. 观察RVIZ中位置和地图输出
   * 如果当前位姿数据产生明显抖动，或者地图产生模糊/折叠/平移，则表示此时建图已失败，需要结束此次建图并查找出错原因。可能的原因如：
     * pc性能低，bag播放过快导致计算跟不上，典型的现象为pause播放程序后，ndt_mapping建图程序仍在运行，此时表示ndt_mapping已积累了较多缓存且处于高负荷状态；
     * 录制时车速过快，导致ndt拟合不足。此时需要重新录制bag数据，控制车速，并增加imu数据。
     * 转弯时转弯半径过小，速度过快。ndt对角度变化较为敏感，过快的转弯会导致ndt匹配失败，进而引起定位丢失和建图漂移。转弯尤其是大转弯时，尽量减低车速。
     * 环境过于空旷，特征不足。ndt基于前后帧匹配进行定位，如果帧信息不足，则会造成匹配失败，典型如操场等空旷区域。应尽量避免在空旷场景中使用单纯的ndt进行建图定位。
     * “隧道效应”，即环境相似度过高，如很长的走廊等。ndt在这种场景下无法获得足够的特征信息，因此也会定位失败。可以通过在场景中添加特征标志物来尝试解决该问题。
     * 参数设置不合理，如voxel大小，迭代次数设置等等。这一些参数的设置需要ndt算法相关知识的理解。
5. 观察RVIZ中的地图，待地图达到期望要求或者建图结束后，发送`/mapping/save_map`指令进行地图的保存。
   `rostopic pub -1 /mapping/save_map automotive_msgs/SaveMap "file_res: <0.3> filename: '/path/to/map.pcd'" `
6. 使用`pcl_viewer <your-map>.pcd`查看建好的地图。
