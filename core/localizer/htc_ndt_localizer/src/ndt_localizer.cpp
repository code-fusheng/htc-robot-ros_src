#include "ndt_localizer.h"

ndt_localizer::ndt_localizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh):nh(nh), private_nh(private_nh), tf2_listener(tf2_buffer) 
{

    // init params
    init_params();

    // def subscribers
    initial_pose_sub = nh.subscribe("initial_pose", 100, &ndt_localizer::callback_init_pose, this);
    map_points_sub = nh.subscribe("points_map", 1, &ndt_localizer::callback_pointsmap, this);
    sensor_points_sub = nh.subscribe("filtered_points", 1, &ndt_localizer::callback_pointcloud, this);

    // def publishers
    sensor_aligned_pose_pub = nh.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
    ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
    exe_time_pub = nh.advertise<std_msgs::Float32>("exe_time_ms", 10);
    transform_probability_pub = nh.advertise<std_msgs::Float32>("transform_probability", 10);
    iteration_num_pub = nh.advertise<std_msgs::Float32>("iteration_num", 10);
    diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    diagnostic_thread = std::thread(&ndt_localizer::timer_diagnostic, this);
    diagnostic_thread.detach();

};

void ndt_localizer::init_params() {

    private_nh.getParam("base_frame", base_frame);
    ROS_INFO("base_frame: %s", base_frame.c_str());

    double trans_epsilon = ndt.getTransformationEpsilon();
    double step_size = ndt.getStepSize();
    double resolution = ndt.getResolution();
    int max_iterations = ndt.getMaximumIterations();

    private_nh.getParam("trans_epsilon", trans_epsilon);
    private_nh.getParam("step_size", step_size);
    private_nh.getParam("resolution", resolution);
    private_nh.getParam("max_iterations", max_iterations);

    map_frame = "map";

    ndt.setTransformationEpsilon(trans_epsilon);
    ndt.setStepSize(step_size);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(max_iterations);

    ROS_INFO(
        "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", 
        trans_epsilon, step_size, resolution, max_iterations);

    private_nh.getParam(
        "converged_param_transform_probability", 
        converged_param_transform_probability);

};

void ndt_localizer::callback_init_pose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr
) {
    if (initial_pose_msg_ptr->header.frame_id == map_frame) {
        initial_pose_cov_msg = *initial_pose_msg_ptr;
    } else {
        // get TF from pose_frame to map_frame
        geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
        get_transform(map_frame, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);
        // transform pose_frame to map_frame
        geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(
        new geometry_msgs::PoseWithCovarianceStamped);
        tf2::doTransform(*initial_pose_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
        // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
        initial_pose_cov_msg = *mapTF_initial_pose_msg_ptr;
    }
    init_pose = false;
};

void ndt_localizer::callback_pointsmap(
    const sensor_msgs::PointCloud2::ConstPtr & map_points_msg_ptr
) {
    const auto trans_epsilon = ndt.getTransformationEpsilon();
    const auto step_size = ndt.getStepSize();
    const auto resolution = ndt.getResolution();
    const auto max_iterations = ndt.getMaximumIterations();
    
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;

    ndt_new.setTransformationEpsilon(trans_epsilon);
    ndt_new.setStepSize(step_size);
    ndt_new.setResolution(resolution);
    ndt_new.setMaximumIterations(max_iterations);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
    ndt_new.setInputTarget(map_points_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_new.align(*output_cloud, Eigen::Matrix4f::Identity());

    ndt_map_mtx.lock();
    ndt = ndt_new;
    ndt_map_mtx.unlock();

};

void ndt_localizer::callback_pointcloud(
    const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr
) {
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx);

  const std::string sensor_frame = pointcloud2_msg_ptr->header.frame_id;
  const auto sensor_ros_time = pointcloud2_msg_ptr->header.stamp;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*pointcloud2_msg_ptr, *sensor_points_sensorTF_ptr);
  // get TF base to sensor
  geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
  get_transform(base_frame, sensor_frame, TF_base_to_sensor_ptr);

  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(
    *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
  
  // set input point cloud
  ndt.setInputSource(sensor_points_baselinkTF_ptr);

  if (ndt.getInputTarget() == nullptr) {
    ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
    return;
  }
  // align
  Eigen::Matrix4f initial_pose_matrix;
  if (!init_pose){
    Eigen::Affine3d initial_pose_affine;
    tf2::fromMsg(initial_pose_cov_msg.pose.pose, initial_pose_affine);
    initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
    // for the first time, we don't know the pre_trans, so just use the init_trans, 
    // which means, the delta trans for the second time is 0
    pre_trans = initial_pose_matrix;
    init_pose = true;
  }else
  {
    // use predicted pose as init guess (currently we only impl linear model)
    initial_pose_matrix = pre_trans * delta_trans;
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const auto align_start_time = std::chrono::system_clock::now();
  key_value_stdmap["state"] = "Aligning";
  ndt.align(*output_cloud, initial_pose_matrix);
  key_value_stdmap["state"] = "Sleeping";
  const auto align_end_time = std::chrono::system_clock::now();
  const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() /1000.0;

  const Eigen::Matrix4f result_pose_matrix = ndt.getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

  const float transform_probability = ndt.getTransformationProbability();
  const int iteration_num = ndt.getFinalNumIteration();

  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  if (
    iteration_num >= ndt.getMaximumIterations() + 2 ||
    transform_probability < converged_param_transform_probability) {
    is_converged = false;
    ++skipping_publish_num;
    std::cout << "Not Converged" << std::endl;
  } else {
    skipping_publish_num = 0;
  }
  // calculate the delta tf from pre_trans to current_trans
  delta_trans = pre_trans.inverse() * result_pose_matrix;

  Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
  std::cout<<"delta x: "<<delta_translation(0) << " y: "<<delta_translation(1)<<
             " z: "<<delta_translation(2)<<std::endl;

  Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
  Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2,1,0);
  std::cout<<"delta yaw: "<<delta_euler(0) << " pitch: "<<delta_euler(1)<<
             " roll: "<<delta_euler(2)<<std::endl;

  // pseudo odom
  nav_msgs::Odometry odom;
  odom.header.stamp = sensor_ros_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  // set the position
  odom.pose.pose = result_pose_msg;
  // set the velocity
  odom.twist.twist.linear.x = delta_translation(0)/exe_time;
  odom.twist.twist.linear.y = delta_translation(1)/exe_time;
  odom.twist.twist.linear.z = delta_translation(2)/exe_time;
  odom.twist.twist.angular.x = delta_euler(1)/exe_time;
  odom.twist.twist.angular.y = delta_euler(0)/exe_time;
  odom.twist.twist.angular.z = delta_euler(2)/exe_time;
  //publish the message
  odom_pub.publish(odom);
  // publish tf(odom frame to base frame)
  geometry_msgs::PoseStamped odom_pose_stamped_msg;
  odom_pose_stamped_msg.header.stamp = sensor_ros_time;
  odom_pose_stamped_msg.pose = result_pose_msg;
  publish_tf("odom", base_frame, odom_pose_stamped_msg);

  pre_trans = result_pose_matrix;
  
  // publish
  geometry_msgs::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;

  if (is_converged) {
    ndt_pose_pub.publish(result_pose_stamped_msg);
  }

  // publish tf(map frame to base frame) ********* ATTENTION*********
  // publish_tf(map_frame_, base_frame_, result_pose_stamped_msg);

  // publish aligned point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
  sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame;
  sensor_aligned_pose_pub.publish(sensor_points_mapTF_msg);


  std_msgs::Float32 exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_pub.publish(exe_time_msg);

  std_msgs::Float32 transform_probability_msg;
  transform_probability_msg.data = transform_probability;
  transform_probability_pub.publish(transform_probability_msg);

  std_msgs::Float32 iteration_num_msg;
  iteration_num_msg.data = iteration_num;
  iteration_num_pub.publish(iteration_num_msg);

  key_value_stdmap["seq"] = std::to_string(pointcloud2_msg_ptr->header.seq);
  key_value_stdmap["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap["skipping_publish_num"] = std::to_string(skipping_publish_num);

  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "align_time: " << align_time << "ms" << std::endl;
  std::cout << "exe_time: " << exe_time << "ms" << std::endl;
  std::cout << "trans_prob: " << transform_probability << std::endl;
  std::cout << "iter_num: " << iteration_num << std::endl;
  std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;

};