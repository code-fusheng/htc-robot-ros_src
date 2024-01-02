#include "euclidean_cluster_core.h"

// #define LEAF_SIZE 0.3 //定义降采样的leaf size，聚类是一个费时运算，为了减少计算量，我们通常先进行降采样
#define MIN_CLUSTER_SIZE 10
#define MAX_CLUSTER_SIZE 1000
#define max_radius 20 //检测半径

EuClusterCore::EuClusterCore(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    pnh.param<double>("leaf_size", leaf_size, 0.1);
    pnh.param<double>("lidar_height", lidar_height, 1.0);
    pnh.param<double>("down_height", down_height, 0.2);
    pnh.param<double>("up_height", up_height, 1.2);
    pnh.param<double>("low_x", x_low, 0.3);
    pnh.param<double>("high_x", x_high, 3.0);
    pnh.param<double>("low_y", y_low, -0.5);
    pnh.param<double>("high_y", y_high, 0.5);
    pnh.param<std::string>("in_cloud_topic", in_cloud_topic, "/points_raw");

    seg_distance_ = {15, 30, 45, 60};
    cluster_distance_ = {0.3, 0.6, 0.9, 2.0, 2.5};

    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);
    pub_bounding_nums = nh.advertise<std_msgs::Int16>("/detected_bounding_nums", 5);
    pub_polyon = nh.advertise<geometry_msgs::PolygonStamped>("/detection/debug_polygon", 10);
    pub_detected_obj_array = nh.advertise<smartcar_msgs::DetectedObjectArray>("/detection/objs", 10);

    sub_point_cloud_ = nh.subscribe(in_cloud_topic, 5, &EuClusterCore::point_cb, this);
    ros::spin();
}

EuClusterCore::~EuClusterCore() {}

void EuClusterCore::publish_cloud(const ros::Publisher &in_publisher,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                                  const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void EuClusterCore::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
}

void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, double in_max_cluster_distance, std::vector<Detected_Obj> &obj_list, smartcar_msgs::DetectedObjectArray &msg_objs)
{

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //create 2d pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    //make it flat
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0; //放到 pcl::EuclideanClusterExtraction 是一个已经平面化的二维点云，这种做法能够带来速度的提升
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud_2d); //放到 pcl::EuclideanClusterExtraction 是一个已经平面化的二维点云，这种做法能够带来速度的提升
    euclid.setClusterTolerance(in_max_cluster_distance);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);

    for (size_t i = 0; i < local_indices.size(); i++)
    {
        // the structure to save one detected object
        Detected_Obj obj_info;

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y;
            p.z = in_pc->points[*pit].z;

            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;

            if (p.x < min_x)
                min_x = p.x;
            if (p.y < min_y)
                min_y = p.y;
            if (p.z < min_z)
                min_z = p.z;
            if (p.x > max_x)
                max_x = p.x;
            if (p.y > max_y)
                max_y = p.y;
            if (p.z > max_z)
                max_z = p.z;
        }

        //min, max points
        obj_info.min_point_.x = min_x;
        obj_info.min_point_.y = min_y;
        obj_info.min_point_.z = min_z;

        obj_info.max_point_.x = max_x;
        obj_info.max_point_.y = max_y;
        obj_info.max_point_.z = max_z;

        //calculate centroid, average  ???
        if (local_indices[i].indices.size() > 0)
        {
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();
        }

        //calculate bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

        obj_info.bounding_box_.header = point_cloud_header_;

        obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2; //几何中心
        obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
        obj_info.bounding_box_.pose.position.z = obj_info.min_point_.z + height_ / 2;

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

        obj_list.push_back(obj_info);

        // > 提取indices点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud (cloud_2d);
        extract.setIndices(boost::make_shared<std::vector<int>>(local_indices[i].indices));
        extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
        extract.filter (*cluster_points);

        // > 提取外轮廓点
        pcl::ConvexHull<pcl::PointXYZ> cHull;
        pcl::PointCloud<pcl::PointXYZ> cHull_points;
        cHull.setInputCloud(cluster_points);
        cHull.reconstruct (cHull_points);

        // if (cHull_points.size() < 3) continue;

        geometry_msgs::PolygonStamped msg_polygon;
        msg_polygon.header = point_cloud_header_;
        // double min_x = 999999999999.0, max_x = -999999999999.0;
        // double min_y = 999999999999.0, max_y = -999999999999.0;
        // double sum_x = 0, sum_y = 0;
        for (const auto& p: cHull_points.points) {
            geometry_msgs::Point32 p32;
            p32.x = p.x;
            p32.y = p.y;
            p32.z = p.z;
            msg_polygon.polygon.points.push_back(p32);
            // if (p.x < min_x) min_x = p.x;
            // if (max_x < p.x) max_x = p.x;
            // if (p.y < min_y) min_y = p.y;
            // if (max_y < p.y) max_y = p.y;
            // sum_x += p.x;
            // sum_y += p.y;
        }

        smartcar_msgs::DetectedObject obj;
        obj.header = point_cloud_header_;
        // 障碍物轮廓点
        obj.convex_hull = msg_polygon;
        // 障碍物中心点
        // obj.pose.position.x = sum_x / cHull_points.size();
        // obj.pose.position.y = sum_y / cHull_points.size();
        obj.pose.position.x = obj_info.centroid_.x;
        obj.pose.position.y = obj_info.centroid_.y;

        // 障碍物尺度
        obj.dimensions.x = (max_x - min_x) > 0.1 ? (max_x - min_x) : 0.1;
        obj.dimensions.y = (max_y - min_y) > 0.1 ? (max_y - min_y) : 0.1;
        obj.dimensions.z = 1;

        obj.id = i;

        msg_objs.objects.push_back(obj);
        // this->pub_polyon.publish(msg_polygon);
        // ROS_INFO("cluster points size: %d, convex points size: %d", cluster_points->size(), cHull_points.size());
    }
    // if (msg_objs.objects.empty()) {
    //     // ROS_WARN("msg_objs.objects is empty! ");
    //     return;
    // }
}

void EuClusterCore::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list, smartcar_msgs::DetectedObjectArray &msg_objs)
{
    //cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    //in this way, the points farther in the pc will also be clustered

    //0 => 0-15m d=0.5
    //1 => 15-30 d=1
    //2 => 30-45 d=1.6
    //3 => 45-60 d=2.1
    //4 => >60   d=2.6

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(5);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        // 如果点的距离大于120m, 忽略该点
        if (origin_distance >= max_radius)
        {
            continue;
        }

        if (origin_distance < seg_distance_[0]) //15
        {
            segment_pc_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1]) //30
        {
            segment_pc_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2]) //45
        {
            segment_pc_array[2]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[3]) //60
        {
            segment_pc_array[3]->points.push_back(current_point);
        }
        else
        {
            segment_pc_array[4]->points.push_back(current_point);
        }
    }

    //std::vector<pcl::PointIndices> final_indices;
    //std::vector<pcl::PointIndices> tmp_indices;

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        cluster_segment(segment_pc_array[i], cluster_distance_[i], obj_list, msg_objs);
    }
}

void EuClusterCore::extract_roi(pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_pc_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr outClouds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> cliper;

    cliper.setInputCloud(filtered_pc_ptr);
    pcl::PointIndices indices;
#pragma omp for // #pragma omp for语法OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速的效果。
    for (size_t i = 0; i < filtered_pc_ptr->points.size(); i++)
    {
        double x = filtered_pc_ptr->points[i].x;
        double y = filtered_pc_ptr->points[i].y;
        double z = filtered_pc_ptr->points[i].z;
        if (x_low < x && x < x_high && y_low < y && y < y_high && down_height < z && z < up_height)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false); //false to save the points
    cliper.filter(*outClouds);
    filtered_pc_ptr->clear();
    filtered_pc_ptr = outClouds;
}

void EuClusterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    smartcar_msgs::DetectedObjectArray msg_objs;
    msg_objs.header = point_cloud_header_;

    point_cloud_header_ = in_cloud_ptr->header;

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    // down sampling the point cloud before cluster
    voxel_grid_filer(current_pc_ptr, filtered_pc_ptr, leaf_size);

    std::vector<Detected_Obj> global_obj_list;

    // 高度以及范围截取
    extract_roi(filtered_pc_ptr);

    // 聚类
    cluster_by_distance(filtered_pc_ptr, global_obj_list, msg_objs);

    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for (size_t i = 0; i < global_obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    }
    bbox_array.header = point_cloud_header_;

    pub_bounding_boxs_.publish(bbox_array);

    // 
    this->pub_detected_obj_array.publish(msg_objs);

    std_msgs::Int16 msg;
    msg.data = bbox_array.boxes.size();
    pub_bounding_nums.publish(msg);
}
