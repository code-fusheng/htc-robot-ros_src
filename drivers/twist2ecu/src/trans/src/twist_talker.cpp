// 这个程序会持续发布数值递增的twist数据, node名为twist_talker, 发布topic名为chatter_twist
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>


int main(int argc, char *argv[])
{
    // 保证编码格式符合本机格式, 不太清楚具体用途
    setlocale(LC_ALL, "");
    
    // 1. 初始化ROS节点
    ros::init(argc, argv, "twist_talker");
    
    // 2. 创建ROS句柄
    ros::NodeHandle nh;
 
    // 3. 创建发布者对象
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("chatter_twist", 1000);

    // 4. 组织发布的信息, 编写发布逻辑
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.twist.linear.x = 0;
    twist_msg.twist.linear.y = 0;
    twist_msg.twist.linear.z = 0;
    twist_msg.twist.angular.x = 0;
    twist_msg.twist.angular.y = 0;
    twist_msg.twist.angular.z = 0;

    ros::Rate r(1);
    while (ros::ok()) 
    {
        twist_pub.publish(twist_msg);
	twist_msg.twist.linear.x += 1;
        twist_msg.twist.linear.y += 1;
	twist_msg.twist.linear.z += 1;
	twist_msg.twist.angular.x += 1;
	twist_msg.twist.angular.y += 1;
	twist_msg.twist.angular.z += 0.001;
	
   	r.sleep();
        ros::spinOnce();
    }


    return 0;
}
