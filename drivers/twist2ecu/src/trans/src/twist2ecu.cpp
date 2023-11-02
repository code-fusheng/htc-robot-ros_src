// 这个程序会完成twist数据格式到ecu数据格式的转换并再发布, node名为twist2ecu, 订阅topic名为chatter_twist, 发布topic名为chatter_ecu
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <trans/ecu.h> // 第三方库, 由编译器根据ecu.msg自动生成
#define _USE_MATH_DEFINES // 使用M_PI宏定义3.1415926

// 负责构建回调函数和节点的订阅与发布
class Sub_And_Pub
{
public:
    Sub_And_Pub()
    {
        // 该节点要发布的话题
        pub = n.advertise<trans::ecu>("/published_topic_ecu", 1);

	// 该节点要订阅的话题
        sub = n.subscribe("/chatter_twist", 1, &Sub_And_Pub::twist2ecu_callback, this);
    }

    void twist2ecu_callback(const geometry_msgs::TwistStamped twist_input)
    {
        trans::ecu ecu_output;
        
        //换算操作
        /* 需要说明的
           ecu格式中最重要的两个数据：motor——目标速度 m/s, steer——转向角度 度
           twist格式中最重要的两个数据: linear.x——车辆速度 m/s, angular.z——偏航角转向速度 rad/s
           我重点是根据这两个数据进行twist2ecu换算
        */
	ecu_output.motor = twist_input.twist.linear.x;
	ecu_output.steer += (twist_input.twist.angular.z * 180 / M_PI) * 1; // 频率为1Hz
	if (ecu_output.steer > 120)
            ecu_output.steer = 30;
        if (ecu_output.steer < -120)
            ecu_output.steer = -30;
        
	//发布节点
        pub.publish(ecu_output);
    }
    
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
};


// 主函数 
int main(int argc, char *argv[])
{
    //保证编码格式符合本机格式, 不太清楚具体用途
    setlocale(LC_ALL, "");
    
    // 1. 初始化ROS节点, node名为twist2ecu
    ros::init(argc, argv, "twist2ecu");

    // 2. 创建Sub_And_Pub这个类, 其中会处理好订阅->处理->再发布的操作
    Sub_And_Pub SAPObject;

    // 4. 回调函数处理, 进入ros::spin()所创建的回调函数循环
    ros::spin();
    
    return 0;
}

