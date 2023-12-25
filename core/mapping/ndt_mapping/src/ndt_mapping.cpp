#include "ndt_mapping.h"

ndt_mapping::ndt_mapping(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{}

ndt_mapping::~ndt_mapping() {};

void ndt_mapping::run() {

  ros::CallbackQueue control_queue;

  init_param();

  switch_sub = nh.subscribe("/htcbot/mode_switch", 10, &ndt_mapping::swtich_callback, this);
  map_path_conf_sub = nh.subscribe("/htcbot/map_path_conf", 10, &ndt_mapping::path_conf_callback, this);

  if (mode_switch) {
    // sub datasource input
    points_sub = nh.subscribe("points_raw", 100000, &ndt_mapping::points_callback, this);
    imu_sub = nh.subscribe("imu_raw", 100000, &ndt_mapping::imu_callback, this);
  }

  // output_sub = nh.subscribe("mapping/save_map", 10, &ndt_mapping::output_callback, this);
  ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
  history_trajectory_pub = nh.advertise<nav_msgs::Path>("/ndt/history_trajectory", 10);
  // control save

  ros::spin();

}

void ndt_mapping::init_param() {

  // init params
  private_nh.param<int>("max_iter", max_iter, 30);
  private_nh.param<double>("setp_size", step_size, 0.5);
  private_nh.param<float>("ndt_res", ndt_res, 5.0);
  private_nh.param<double>("trans_eps", trans_eps, 0.01);
  private_nh.param<double>("voxel_leaf_size", voxel_leaf_size, 1.0);
  private_nh.param<double>("scan_rate", scan_rate, 10.0);
  private_nh.param<double>("min_scan_range", min_scan_range, 1.0);
  private_nh.param<double>("max_scan_range", max_scan_range, 200.0);
  private_nh.param<double>("min_add_scan_shift", min_add_scan_shift, 1.5);
  private_nh.param<bool>("use_imu", use_imu, false);
  private_nh.param<bool>("use_odom", use_odom, false);
  private_nh.param<std::string>("imu_topic", _imu_topic, "/imu_raw");
  private_nh.param<std::string>("lidar_topic", _lidar_topic, "/points_raw");
  private_nh.param<std::string>("odom_topic", _odom_topic, "/odom");
  private_nh.param<std::string>("map_path", map_path, "");
  
  // init tf params
  private_nh.param<double>("x", _tf_x, 0.0);
  private_nh.param<double>("y", _tf_y, 0.0);
  private_nh.param<double>("z", _tf_z, 0.0);
  private_nh.param<double>("roll", _tf_roll, 0.0);
  private_nh.param<double>("pitch", _tf_pitch, 0.0);
  private_nh.param<double>("yaw", _tf_yaw, 0.0);

  private_nh.param<int>("initial_scan_loaded", initial_scan_loaded, 0);
  private_nh.param<bool>("incremental_voxel_update", incremental_voxel_update, false);  

  std::cout << "incremental_voxel_update: " << incremental_voxel_update << std::endl;
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;

  /**
   * 计算激光雷达相对于车身底盘的初始变换矩阵 激光雷达 localizer => 车身底盘 base_link 坐标系
   * 对应的关系由 tf_x, tf_y, tf_z 给出
  */

  // 初始化平移向量
  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  // 初始化旋转向量，分别绕着 x、y、z 轴旋转
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  tf_ltob = tf_btol.inverse();

   // previous: 前一帧点云车辆的位置
  previous_pose.x = 0.0;
  previous_pose.y = 0.0;
  previous_pose.z = 0.0;
  previous_pose.roll = 0.0;
  previous_pose.pitch = 0.0;
  previous_pose.yaw = 0.0;
  // ndt_pose: NDT 配准算法得到的车辆位置
  ndt_pose.x = 0.0;
  ndt_pose.y = 0.0;
  ndt_pose.z = 0.0;
  ndt_pose.roll = 0.0;
  ndt_pose.pitch = 0.0;
  ndt_pose.yaw = 0.0;
  // current_pose: 当前帧点云车辆位置
  current_pose.x = 0.0;
  current_pose.y = 0.0;
  current_pose.z = 0.0;
  current_pose.roll = 0.0;
  current_pose.pitch = 0.0;
  current_pose.yaw = 0.0;
  // current_pose_imu: 当前帧imu位置
  current_pose_imu.x = 0.0;
  current_pose_imu.y = 0.0;
  current_pose_imu.z = 0.0;
  current_pose_imu.roll = 0.0;
  current_pose_imu.pitch = 0.0;
  current_pose_imu.yaw = 0.0;
  // guess_pose: NDT 配准算法所需的初始位置
  guess_pose.x = 0.0;
  guess_pose.y = 0.0;
  guess_pose.z = 0.0;
  guess_pose.roll = 0.0;
  guess_pose.pitch = 0.0;
  guess_pose.yaw = 0.0;
  // added: 用于计算地图更新的距离变化
  added_pose.x = 0.0;
  added_pose.y = 0.0;
  added_pose.z = 0.0;
  added_pose.roll = 0.0;
  added_pose.pitch = 0.0;
  added_pose.yaw = 0.0;
  // diff: 前后两次接收到传感器(IMU或者odom)消息时位姿的变化
  diff_x = 0.0;
  diff_y = 0.0;
  diff_z = 0.0;
  diff_yaw = 0.0;

  // offset: 位姿的偏差矫正

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  // 发布和订阅相关消息
  map.header.frame_id = "map";

  history_trajectory.header.frame_id = "map";

  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);  

  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);

  is_first_map = true;

  mode_switch = false;

  std::cout << "ndt_res: " << ndt_res << std::endl;
  std::cout << "step_size: " << step_size << std::endl;
  std::cout << "trans_epsilon: " << trans_eps << std::endl;
  std::cout << "max_iter: " << max_iter << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size << std::endl;
  std::cout << "min_scan_range: " << min_scan_range << std::endl;
  std::cout << "max_scan_range: " << max_scan_range << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
  std::cout << "use_odom: " << use_odom << std::endl;
  std::cout << "use_imu: " << use_imu << std::endl;
  std::cout << "odom_topic: " << _odom_topic << std::endl;
  std::cout << "lidar_topic: " << _lidar_topic << std::endl;
  std::cout << "imu_topic: " << _imu_topic << std::endl;

}

void ndt_mapping::swtich_callback(const htcbot_msgs::ModeSwitch::ConstPtr& input) {
  ROS_INFO("[htc_ndt_mapping] ==> mode: %s switch: %s", input->mode.c_str(), input->switch_to ? "true" : "false");
  if (input->mode == "Mapping") {
    mode_switch = input->switch_to ? true : false;
    if (mode_switch) {
      // sub datasource input
      ROS_INFO("xxxx");
      points_sub = nh.subscribe("points_raw", 100000, &ndt_mapping::points_callback, this);
      imu_sub = nh.subscribe("imu_raw", 100000, &ndt_mapping::imu_callback, this);
    } else {
      ROS_INFO("yyyy");
      // save map
      pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
      map_ptr->header.frame_id = "map";
      sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*map_ptr, *map_msg_ptr);
      std::string filename = map_path + "/static.map";
      pcl::io::savePCDFileASCII(filename, *map_ptr);
    }
  }
}

void ndt_mapping::path_conf_callback(const htcbot_msgs::MapPathConf::ConstPtr& input) {
  ROS_INFO("[htc_ndt_localizer] ==> path_static: %s", input->path_static.c_str());
  map_path = input->path_static;
}

void ndt_mapping::output_callback(const automotive_msgs::SaveMap::ConstPtr& input) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  map_ptr->header.frame_id = "map";
  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  std::string filename = map_path + "/static.map";
  pcl::io::savePCDFileASCII(filename, *map_ptr);
}

void ndt_mapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr& input) 
{
  // r 表示激光点云到激光雷达的距离
  double r;
  pcl::PointXYZI p;
  // tmp 为原始点云转换的 PCL 点云数据
  // scan 为 tmp 过滤后的 PCL 点云数据
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  tf::Quaternion q;  
  
  // 分别表示激光雷达与车体 相对于 map 的坐标系变换矩阵，并且初始化为 4 阶单位矩阵 
  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  // 将点云数据转换为 PCL 使用的数据类型
  pcl::fromROSMsg(*input, tmp);

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
  {
    // 将 tmp 点云容器中的点进行逐一处理、去除不符合距离范围的点云数据
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;
    // 计算点雨激光雷达的欧式距离 r
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    // 判断: 若小于最小距离或者大于最大距离，则滤除该点
    if (min_scan_range < r && r < max_scan_range)
    {
      // 满足的数据逐一插入至 scan 点云，完成原始点云的过滤
      scan.push_back(p);
    }
  } 

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

  // Add initial point cloud to velodyne_map
  // 如果点云地图没有初始化载入
  if (initial_scan_loaded == 0)
  { 
    // 将初始化点云加入至地图
    // 通过 tf_btol 变换矩阵 和 scan 点云数据 作为输入 将点云进行转化
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    // 将转换后的点云加入 map 进行拼接，实际上是作为第一帧点云图像
    map += *transformed_scan_ptr;
    // 标记初始化载入状态 1: 成功
    initial_scan_loaded = 1;
  }

  // Apply voxelgrid filter
  // 对 scan 输入点云进行体素过滤 并将结果保存至 filtered_scan
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size,
                                  voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  // 设置转换参数 Epsilon、最大步长、网格大小、最大迭代次数 以及设置输入数据为 已过滤点云 filtered_scan_ptr
  ndt.setTransformationEpsilon(trans_eps);
  ndt.setStepSize(step_size);
  ndt.setResolution(ndt_res);
  ndt.setMaximumIterations(max_iter);
  ndt.setInputSource(filtered_scan_ptr);

  // 将第一张地图 map_ptr 设置输入 NDT 输入点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
  if (is_first_map == true){
    ndt.setInputTarget(map_ptr);
    is_first_map = false;
  }

  // NDT 目标点云为 map 全局地图，NDT 源点云为每一次接收到降采样过滤原始点云 filtered_scan_ptrs
  // guess_pose: 初始位置 = 前一帧位置 + 位置的变化
  // 初始位置的偏航角与转弯有关，为前一帧的偏航角 + 偏航角的变化
  guess_pose.x = previous_pose.x + diff_x;
  guess_pose.y = previous_pose.y + diff_y;
  guess_pose.z = previous_pose.z + diff_z;
  guess_pose.roll = previous_pose.roll;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.yaw = previous_pose.yaw + diff_yaw;
  // 1. 使用 imu + odom 融合
  if (use_imu == true && use_odom == true)
    imu_odom_calc(current_scan_time);
  // 2. 单独使用 imu 求初值
  if (use_imu == true && use_odom == false)
    imu_calc(current_scan_time);
  // 3. 单独使用 odom 里程计求初值
  if (use_imu == false && use_odom == true)
    odom_calc(current_scan_time);
  // 声明 NDT 初值 => 根据方法赋初值
  pose guess_pose_for_ndt;
  
  if (use_imu == true && use_odom == true)
    guess_pose_for_ndt = guess_pose_imu_odom;
  else if (use_imu == true && use_odom == false)
    guess_pose_for_ndt = guess_pose_imu;
  else if (use_imu == false && use_odom == true)
    guess_pose_for_ndt = guess_pose_odom;
  else
    // 使用原始初值
    guess_pose_for_ndt = guess_pose;

  // 利用 current_pose 位置的位姿旋转量 来初始化关于xyz轴的旋转向量
  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
  // 利用 current_pose 位置的三维坐标 来初始化平移向量
  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // 根据选择类型，进行 NDT 配准
  if (method_type == MethodType::PCL_GENERIC)
  {
    // 开始 NDT 配准，ndt.align 以 init_guess 为初值进行迭代优化 => 然后将配准结果保存在 output_cloud 点云中
    ndt.align(*output_cloud, init_guess);
    // 计算目标点云与源点云之间的欧式距离平方和作为适应分数
    fitness_score = ndt.getFitnessScore();
    // 得到最终的激光雷达相对于 map 坐标系的变换矩阵 t_localizer
    t_localizer = ndt.getFinalTransformation();
    // 判断是否收敛
    has_converged = ndt.hasConverged();
    // 得到最后的迭代次数
    final_num_iteration = ndt.getFinalNumIteration();
    transformation_probability = ndt.getTransformationProbability();
  }
  else if (method_type == MethodType::PCL_ANH)
  {
    anh_ndt.align(init_guess);
    fitness_score = anh_ndt.getFitnessScore();
    t_localizer = anh_ndt.getFinalTransformation();
    has_converged = anh_ndt.hasConverged();
    final_num_iteration = anh_ndt.getFinalNumIteration();
  }

  t_base_link = t_localizer * tf_ltob;
  // 将原始图像经过 NDT 变换之后输出转换点云 transformed_scan_ptr
  // 注意scan_ptr保存的几乎是原始点云(只过滤了最近最远点)，因此用于匹配的地图会很大
  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_l, mat_b;

  // 前三行 前三列 表示旋转矩阵
  // 第四列前三行表示的是平移向量

  mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                 static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                 static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                 static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                 static_cast<double>(t_localizer(2, 2)));

  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update localizer_pose.
  localizer_pose.x = t_localizer(0, 3);
  localizer_pose.y = t_localizer(1, 3);
  localizer_pose.z = t_localizer(2, 3);
  mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

  // Update ndt_pose
  ndt_pose.x = t_base_link(0, 3);
  ndt_pose.y = t_base_link(1, 3);
  ndt_pose.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);//mat2rpy
  
  // 将 NDT 配准之后的位置作为当前位置
  current_pose.x = ndt_pose.x;
  current_pose.y = ndt_pose.y;
  current_pose.z = ndt_pose.z;
  current_pose.roll = ndt_pose.roll;
  current_pose.pitch = ndt_pose.pitch;
  current_pose.yaw = ndt_pose.yaw;

  // 以当前位置作为坐标原点
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  // 以当前位置旋转角度 rpy，设置旋转四元素 q
  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  // 利用 q 来设置旋转
  transform.setRotation(q);
  // 发布坐标变换信息
  br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "base_link"));

  // 计算激光雷达扫描间隔时间
  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();

  // 计算相邻帧位姿偏差
  diff_x = current_pose.x - previous_pose.x;
  diff_y = current_pose.y - previous_pose.y;
  diff_z = current_pose.z - previous_pose.z;
  diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
  diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  // 利用前后两帧扫描位置偏差与扫描时间间隔计算此时的瞬时速度
  current_velocity_x = diff_x / secs;
  current_velocity_y = diff_y / secs;
  current_velocity_z = diff_z / secs;
  // 当前位姿 current_pose 赋予 imu 当前位姿，更新矫正
  current_pose_imu.x = current_pose.x;
  current_pose_imu.y = current_pose.y;
  current_pose_imu.z = current_pose.z;
  current_pose_imu.roll = current_pose.roll;
  current_pose_imu.pitch = current_pose.pitch;
  current_pose_imu.yaw = current_pose.yaw;

  current_pose_odom.x = current_pose.x;
  current_pose_odom.y = current_pose.y;
  current_pose_odom.z = current_pose.z;
  current_pose_odom.roll = current_pose.roll;
  current_pose_odom.pitch = current_pose.pitch;
  current_pose_odom.yaw = current_pose.yaw;

  current_pose_imu_odom.x = current_pose.x;
  current_pose_imu_odom.y = current_pose.y;
  current_pose_imu_odom.z = current_pose.z;
  current_pose_imu_odom.roll = current_pose.roll;
  current_pose_imu_odom.pitch = current_pose.pitch;
  current_pose_imu_odom.yaw = current_pose.yaw;

  current_velocity_imu_x = current_velocity_x;
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;

  // 最后将 current_pose 赋值前一帧位姿 previous_pos
  previous_pose.x = current_pose.x;
  previous_pose.y = current_pose.y;
  previous_pose.z = current_pose.z;
  previous_pose.roll = current_pose.roll;
  previous_pose.pitch = current_pose.pitch;
  previous_pose.yaw = current_pose.yaw;

  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  offset_imu_x = 0.0;
  offset_imu_y = 0.0;
  offset_imu_z = 0.0;
  offset_imu_roll = 0.0;
  offset_imu_pitch = 0.0;
  offset_imu_yaw = 0.0;

  offset_odom_x = 0.0;
  offset_odom_y = 0.0;
  offset_odom_z = 0.0;
  offset_odom_roll = 0.0;
  offset_odom_pitch = 0.0;
  offset_odom_yaw = 0.0;

  offset_imu_odom_x = 0.0;
  offset_imu_odom_y = 0.0;
  offset_imu_odom_z = 0.0;
  offset_imu_odom_roll = 0.0;
  offset_imu_odom_pitch = 0.0;
  offset_imu_odom_yaw = 0.0;

  // Calculate the shift between added_pos and current_pos
  // 计算 added_pose 与 current_pose 之间的距离
  // added_pose 为上一次更新地图的位姿信息
  double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) +
                        pow(current_pose.y - added_pose.y, 2.0));
  double _diff_angle = getAbsoluteAngleDiff(current_pose.yaw, added_pose.yaw) ; 
  if (shift >= min_add_scan_shift || _diff_angle > 7)
  {
    map += *transformed_scan_ptr;

    added_pose.x = current_pose.x;
    added_pose.y = current_pose.y;
    added_pose.z = current_pose.z;
    added_pose.roll = current_pose.roll;
    added_pose.pitch = current_pose.pitch;
    added_pose.yaw = current_pose.yaw;

    if (method_type == MethodType::PCL_GENERIC)
      ndt.setInputTarget(map_ptr);
    else if (method_type == MethodType::PCL_ANH)
    {
      if (incremental_voxel_update == true)
        anh_ndt.updateVoxelGrid(transformed_scan_ptr);
      else
        anh_ndt.setInputTarget(map_ptr);
    }

    // 声明 ROS 可用的点云对象
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    // 将 PCL 使用的 map_ptr 数据转换为 ROS 类型的 map_msg_ptr
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    // 发布 ndt_map 地图数据
    ndt_map_pub.publish(*map_msg_ptr);
  }

  q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  current_pose_msg.pose.position.x = current_pose.x;
  current_pose_msg.pose.position.y = current_pose.y;
  current_pose_msg.pose.position.z = current_pose.z;
  current_pose_msg.pose.orientation.x = q.x();
  current_pose_msg.pose.orientation.y = q.y();
  current_pose_msg.pose.orientation.z = q.z();
  current_pose_msg.pose.orientation.w = q.w();

  // 发布当前位姿
  current_pose_pub.publish(current_pose_msg);

  if (shift > 0.1) {
    history_trajectory.poses.push_back(current_pose_msg);
    history_trajectory_pub.publish(history_trajectory);
  }

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
  std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
  std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
            << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

}

void ndt_mapping::imu_callback(const sensor_msgs::Imu::Ptr& input)
{
  // 当接收到 imu 的消息的时候，获取 imu 当前的时间戳 => 作为当前时间 current_time
  const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  // 计算前后两次接收到消息的微小时间差
  const double diff_time = (current_time - previous_time).toSec();
  double imu_roll, imu_pitch, imu_yaw;
  // 声明用于表示旋转的四元数
  tf::Quaternion imu_orientation;
  // 将 imu 采集的旋转四元数消息转化为 TF 类型的旋转四元数存入 imu_orientation
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  // 利用 imu_orientation 旋转变量 初始化一个 3*3 的旋转矩阵 然后通过 imu_roll, imu_pitch, imu_yaw 获取 imu 此时的旋转角度
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);
  // 将角度转化为弧度
  imu_roll = wrapToPmPi(imu_roll);
  imu_pitch = wrapToPmPi(imu_pitch);
  imu_yaw = wrapToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  // 将角度的变化转换为弧度
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

  sensor_msgs::Imu imu;

  imu.header = input->header;
  // 获取 imu x 方向上的线性加速度
  imu.linear_acceleration.x = input->linear_acceleration.x;
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0)
  {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  }
  else
  {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }
}

/**
 * odom_callback 函数
 * 以接收到的里程计信息为输入参数 调用 odom_calc 计算求得 NDT 的初始位姿估计
*/
void ndt_mapping::odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
  odom = *input;
  odom_calc(input->header.stamp);
}

double ndt_mapping::wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

double ndt_mapping::wrapToPmPi(double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

double ndt_mapping::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

/**
 * imu 配准初值计算函数
*/
void ndt_mapping::imu_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
                 std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
                 std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;

  offset_imu_x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;
  current_velocity_imu_y += accY * diff_time;
  current_velocity_imu_z += accZ * diff_time;

  offset_imu_roll += diff_imu_roll;
  offset_imu_pitch += diff_imu_pitch;
  offset_imu_yaw += diff_imu_yaw;

  guess_pose_imu.x = previous_pose.x + offset_imu_x;
  guess_pose_imu.y = previous_pose.y + offset_imu_y;
  guess_pose_imu.z = previous_pose.z + offset_imu_z;
  guess_pose_imu.roll = previous_pose.roll + offset_imu_roll;
  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pitch;
  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_yaw;

  previous_time = current_time;
}

void ndt_mapping::odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  // 获取前后两帧时间差
  double diff_time = (current_time - previous_time).toSec();

  // 计算两帧时间间隔内的里程计旋转角度
  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;
  // 更新当前里程计位置的角度
  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;
  // diff_distance 表示在 x 方向的变化距离
  // offset 表示车身不稳定造成的计算偏差
  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;

  // 对初始位置进行修正 = 前一帧位置 + 偏差位置
  guess_pose_odom.x = previous_pose.x + offset_odom_x;
  guess_pose_odom.y = previous_pose.y + offset_odom_y;
  guess_pose_odom.z = previous_pose.z + offset_odom_z;
  guess_pose_odom.roll = previous_pose.roll + offset_odom_roll;
  guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pitch;
  guess_pose_odom.yaw = previous_pose.yaw + offset_odom_yaw;

  previous_time = current_time;
}

/**
 * 里程计 + imu 联合初值计算函数
*/
void ndt_mapping::imu_odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_imu_odom_x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_roll += diff_imu_roll;
  offset_imu_odom_pitch += diff_imu_pitch;
  offset_imu_odom_yaw += diff_imu_yaw;

  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_yaw;

  previous_time = current_time;
}

double ndt_mapping::getAbsoluteAngleDiff(const double &current_yaw, const double &added_yaw) 
{
  double diff = (current_yaw - added_yaw) * 180.0 / PI;
  diff = diff + 360*4;
  int res = int(diff) % 360;
  if ( res > 180 ) res = 360 -res;
  return res;
}