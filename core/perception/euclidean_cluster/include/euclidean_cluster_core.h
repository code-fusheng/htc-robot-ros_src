#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <smartcar_msgs/DetectedObject.h>
#include <smartcar_msgs/DetectedObjectArray.h>

class EuClusterCore
{

private:
  struct Detected_Obj
  {
    jsk_recognition_msgs::BoundingBox bounding_box_;
    std_msgs::Header header;
    geometry_msgs::Pose pose;
    geometry_msgs::Vector3 dimensions;
    std_msgs::Float32 value;
    std_msgs::UInt32 label;

    pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;
    pcl::PointXYZ centroid_;
  };

  ros::Subscriber sub_point_cloud_;

  ros::Publisher pub_bounding_boxs_;
  ros::Publisher pub_bounding_nums;
  ros::Publisher pub_polyon;
  ros::Publisher pub_detected_obj_array;

  std::vector<double> seg_distance_, cluster_distance_;

  std_msgs::Header point_cloud_header_;

  double lidar_height, down_height, up_height; // 雷达高度, 往下截取点云高度, 往上截取点云高度
  double x_low, x_high, y_low, y_high;         // 水平方向截取方形ROI区域
  std::string in_cloud_topic;
  double leaf_size;

  void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);

  void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list, smartcar_msgs::DetectedObjectArray &msg_objs);

  void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                       double in_max_cluster_distance, std::vector<Detected_Obj> &obj_list,
                       smartcar_msgs::DetectedObjectArray &msg_objs);

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);

  void extract_roi(pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_pc_ptr);

public:
  EuClusterCore(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~EuClusterCore();
};