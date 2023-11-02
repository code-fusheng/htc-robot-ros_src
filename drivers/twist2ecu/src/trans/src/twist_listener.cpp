// 这个程序会持续订阅twist数据, node名为twist_listener, 订阅topic名为chatter_twist
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

void doTwist_callback(const geometry_msgs::TwistStamped twist_msg)
{
    
}

int main(int argc, char *argv[])
{
    // 保证编码格式符合本机格式, 不太清楚具体用途
    setlocale(LC_ALL, "");
    
    // 1. 初始化ROS节点
    ros::init(argc, argv, "twist_listener");
    
    // 2. 创建ROS句柄
    ros::NodeHandle nh;
 
    // 3. 创建订阅对象
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("chatter_twist", 1000, doTwist_callback);

    // 4. 回调函数处理
    ros::spin();


    return 0;
}
