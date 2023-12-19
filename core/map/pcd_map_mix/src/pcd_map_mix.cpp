#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_map_mix");
    ros::NodeHandle nh;
    
    // 初始化 PCL 可视化工具
    pcl::visualization::PCLVisualizer viewer("Global Map");

    // 创建一个空的点云表示全局地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>);

    bool is_first_iteration = true;

    // 加载并配准多个 PCD 文件
    for (char key = 'A'; key <= 'B'; ++key) {
        // 构建 PCD 文件路径
        std::string file_path = "/home/code/htc-robot-ros_ws/data/map/static_" + std::string(1, key) + ".pcd";

        // 加载当前区域的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *local_map) == -1) {
            PCL_ERROR("Couldn't read file %s\n", file_path.c_str());
            continue;
        }

        // 在 PCL 可视化工具中显示当前区域的点云
        viewer.addPointCloud(local_map, "local_map_" + std::string(1, key));
        viewer.spinOnce();

        // 将当前区域的点云与全局地图配准
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setStepSize(0.1);
        ndt.setResolution(1.0);
        ndt.setMaximumIterations(50);
        if (is_first_iteration) {
            // 设置当前区域的点云作为全局地图
            global_map = local_map;
            is_first_iteration = false;
        } else {
            // 将当前区域的点云与全局地图配准
            ndt.setInputSource(local_map);
            ndt.setInputTarget(global_map);

            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
            ndt.align(*aligned);

            // 将配准后的点云融合到全局地图中
            *global_map += *aligned;
        }

        // 在 PCL 可视化工具中显示全局地图
        viewer.updatePointCloud(global_map, "global_map");
        viewer.spinOnce();
    }

    // 可以在这里保存最终的全局地图到文件
    pcl::io::savePCDFileASCII("/home/code/htc-robot-ros_ws/data/map/final_global_map.pcd", *global_map);

    ros::spin();
    return 0;
}