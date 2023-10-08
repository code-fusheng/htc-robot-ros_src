最新版 git 获取：https://github.com/orbbec/ros_astra_camera
适配使用环境：Ubuntu18.04，其它环境不保证支持

1.自行安装 ROS

2.安装依赖
sudo apt install ros-$ROS_DISTRO-rgbd-launch
sudo apt install ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros

3.编译功能包
创建文件夹：mkdir ~/catkin_wk/src
把压缩包解压到文件夹~/catkin_wk/src
赋可执行权限提前避免错误：sudo chmod -R 777 ~/catkin_wk/src/ros_astra_camera
编译，在文件夹~/catkin_wk 下运行：catkin_make
等待编译完成

4.创建设备规则
cd ~/catkin_wk/src/ros_astra_camera/scripts
sudo ./create_udev_rules
拔插摄像头

5.启动相机
AstraProPlus 相机：roslaunch astra_camera astraproplus.launch
AstraPro 相机：roslaunch astra_camera astrapro.launch
AstraS 相机：roslaunch astra_camera astra.launch
Dabai 相机：roslaunch astra_camera dabai_u3.launch
Gemini 相机：roslaunch astra_camera gemini.launch
