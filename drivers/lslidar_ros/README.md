# lslidar_c16


### 编译与运行：

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
把驱动压缩包拷贝到src目录下，并解压
cd ~/lslidar_ws
catkin_make
source devel/setup.bash
roslaunch lslidar_driver lslidar_c16.launch 


~~~

### 





launch 文件说明：

~~~xml
  <arg name="device_ip" default="192.168.1.200" />  //雷达ip
  <arg name="msop_port" default="2368" />   //数据包目的端口号
  <arg name="difop_port" default="2369" />  //设备包目的端口号
  <arg name="use_gps_ts" default="false" /> //是否使用gps授时
  <arg name="lidar_type" default="c16"/>    //选择雷达型号

  //true：雷达零度角对应点云x轴，false：雷达零度角对应点云y轴
  <param name="coordinate_opt" value="false"/>  //默认 雷达零度角对应点云y轴

  <!-- 1表示xyzirt格式，2表示xyzi格式 -->
  <arg name= "pcl_type" default="2"/>
 


~~~



