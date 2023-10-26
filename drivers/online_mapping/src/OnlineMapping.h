#ifndef GENERATOR_H
#define GENERATOR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>
#include <string>
#include <vector>
#include <algorithm>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QGroupBox>
#include <QSpinBox>
#include <QString>
#include "automotive_msgs/OnlineMapping.h"
#include <automotive_msgs/NDTMappingRes.h>

namespace online_mapping{
    
class OnlineMapping: public rviz::Panel
{
    Q_OBJECT
public:
    // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
    OnlineMapping( QWidget* parent = 0 );

    // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin
    // 中，数据就是topic的名称
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;


//     // 公共槽.
// public Q_SLOTS:
//     // 当用户输入topic的命名并按下回车后，回调用此槽来创建一个相应名称的topic publisher
//     void setTopic( const QString& topic );

//     // 内部槽.
protected Q_SLOTS:
    void onCheckValidate();
    void onStartMapping();
    void onStopMapping();
    void onForceStopMapping();

    // 内部变量.
protected:
    // 加载map部分
    QGroupBox*   mapping_box;

    // QLabel*      label_map_save_dir;
    // QLineEdit*   input_map_save_dir;
    // QPushButton* btn_map_save_dir_choose;

    QLabel*      label_motor_name;
    QLineEdit*   input_motor_name;

    QLabel*      label_carriage_name;
    QLineEdit*   input_carriage_name;

    QPushButton* btn_force_stop;
    QPushButton* btn_check_validate;
    QPushButton* btn_stop_mapping;
    QPushButton* btn_start_mapping;

    QLabel *label_info;
    
    // The ROS node handle.
    ros::NodeHandle nh_;

    // publisher
    ros::Publisher pub_mapping_cmd, pub_force_stop;

    // service client
    // ros::ServiceClient mapping_client;
     
    // subscriber
    ros::Subscriber sub_ndt_mapping_status;

private:
    const QString base_dir = "/home/data";

    void cb_ndt_mapping_status(const automotive_msgs::NDTMappingResConstPtr &msg);

    bool check_dir_validate(const QString motor, const QString carriage);

    void enter_mapping_status();

    void leave_mapping_status();
};

} // end namespace online_mapping

#endif  // GENERATOR_H