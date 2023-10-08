基础控制

启动初始化节点
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch

按键控制
roslaunch wheeltec_robot_rc keyboard_teleop.launch

启动相机
AstraS 相机：roslaunch astra_camera astra.launch

catkin_make -DCATKIN_WHITSLIST_NOETIC_PACKAGES="ros_astra_camera"

查包
rospack find rtabmap_ros
