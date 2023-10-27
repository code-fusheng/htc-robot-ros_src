```shell
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── docs
│   ├── bak
│   ├── debug.md
│   ├── DOC_DESC.md
│   ├── drivers
│   ├── install_env.md
│   ├── install.md
│   ├── map
│   ├── msgs
│   ├── nav
│   ├── sensor
│   └── tf
├── drivers
│   ├── amathutils_lib  # 数学运算函数整合 (autoware - yunle)
│   ├── astar_search    # (autoware - yunle)
│   ├── can_bridge  # 底盘 CAN 控制功能包
│   ├── can_*  # CAN 相关适配
│   ├── ddynamic_reconfigure  # 激光雷达点云相关
│   ├── fdilink_ahrs    # FDI 惯导单元功能包
│   ├── lio_sam_master  # lio_sam 三维重建算法
│   ├── lslidar_ros # 镭神智能激光雷达
│   ├── nmea_msgs   # GPS 相关消息包
│   ├── pointcloud_to_laserscan # 点云转激光扫描线
│   ├── realsense-ros   # Intel 深度相机功能包
│   ├── ros_astra_camera    # Astra 奥比中光相机功能包
│   ├── rtabmap_ros # RTABMAP 激光雷达+视觉导航功能包
│   ├── rtabmap_ros_noetic.zip
│   ├── simulation  # 仿真
│   ├── grid_map_generator  # (yunle)
│   ├── waypoint_planner    # 航迹点跟随(autoware - yunle) astar_search
│   ├── libwaypoint_follower    #  (yunle)
│   ├── ultrasonic_driver    #  超声波驱动
│   └── wheeltec_gps_driver # GNSS - GPS 驱动 ultrasonic_driver
├── INSTALL.md
├── msgs
│   ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
│   ├── demo_msgs
│   └── htc_robot_msgs
├── navigation
│   ├── amcl    # ACML 辅助定位
│   ├── base_local_planner  # 局部规划
│   ├── carrot_planner
│   ├── clear_costmap_recovery
│   ├── costmap_2d  # 代价地图
│   ├── dwa_local_planner   # DWA 算法的局部路径规划
│   ├── fake_localization   # 自定位
│   ├── global_planner  # 全局路径规划
│   ├── map_server  # 地图存储加载功能包
│   ├── move_base   # 控制(决策)功能包
│   ├── move_slow_and_clear
│   ├── nav_core    # 导航核心
│   ├── navfn
│   ├── navigation
│   ├── README.md
│   ├── rotate_recovery
│   ├── teb_local_planner   # TEB 算法的局部路径规划
│   └── voxel_grid  # 网格地图
├── PLAN.md
├── README.md
├── robot_pose_ekf  # 里程计优化功能包(卡尔曼滤波)
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── example_with_gps.launch
│   ├── include
│   ├── package.xml
│   ├── plotekf.m
│   ├── README.md
│   ├── robot_pose_ekf.launch
│   ├── scripts
│   ├── src
│   ├── srv
│   └── test
├── sync_2_mac.sh
├── sync_2_min_robot_delete.sh
├── sync_2_min_robot.sh
├── sync_2_vm_melodic.sh
├── sync_2_vm_noetic.sh
├── sync_src.sh
└── turn_on_robot_htc   # 工程脚本核心包
    ├── CMakeLists.txt
    ├── costmap_common_params   # 代价地图公共参数
    │   └── param_wheeltec_mini_akm
    ├── include
    │   ├── debug_robot.h
    │   ├── htc_robot.h
    │   ├── htc_wheeltec_robot.h
    │   ├── htc_yunle_robot.h
    │   └── Quaternion_Solution.h
    ├── launch
    │   ├── camera_htc.launch           # 相机控制节点
    │   ├── gnss_htc.launch             # GNSS 控制节点
    │   ├── hello_robot.launch
    │   ├── imu_htc.launch              # IMU 惯导控制节点
    │   ├── include
    │   ├── lidar_htc.launch            # 雷达控制节点
    │   ├── mapping_htc.launch          # 建图节点
    │   ├── navigation_htc.launch       # 导航节点
    │   ├── param
    │   ├── robot_cmd_htc.launch        # 终端控制节点
    │   ├── robot_model_htc.launch      # 机器人模型可视化节点
    │   └── turn_on_robot_htc.launch    # 机器人启动脚本
    ├── map     # 地图存储目录
    │   ├── MAP-default.yaml
    │   ├── MAP.pgm
    │   ├── map_saver_htc.launch
    │   └── MAP.yaml
    ├── package.xml
    ├── param_common    # 导航公共参数配置
    │   ├── base_global_planner_param.yaml
    │   ├── dwa_local_planner_params.yaml
    │   ├── global_costmap_params.yaml
    │   ├── local_costmap_params.yaml
    │   └── move_base_params.yaml
    ├── rviz    # rviz 参数配置
    │   ├── cartographer.rviz
    │   ├── lio_sam.rviz
    │   ├── nav.rviz
    │   └── slam.rviz
    ├── scripts
    │   ├── send_mark.py
    │   └── turtlebot_teleop_key.py
    ├── src
    │   ├── hello_robot.cpp
    │   ├── htc_wheeltec_robot.cpp
    │   └── Quaternion_Solution.cpp
    ├── udev    # 设备串口配置
    │   ├── debug_udev.sh
    │   ├── imu_udev.sh
    │   ├── README.md
    │   └── rule.d
    └── urdf    # 机器人模型 URDF
        ├── wheeltec
        └── yunle
```
