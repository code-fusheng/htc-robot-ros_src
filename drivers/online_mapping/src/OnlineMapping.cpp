#include <stdio.h>
#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QDebug>
#include "OnlineMapping.h"
#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <automotive_msgs/NDTMappingReq.h>
#include <automotive_msgs/NDTMappingRes.h>

namespace online_mapping
{

// 构造函数，初始化变量
OnlineMapping::OnlineMapping( QWidget* parent )
  : rviz::Panel( parent )
{
    QVBoxLayout* main_layout = new QVBoxLayout;

    // 地图加载
    this->mapping_box = new QGroupBox("在线建图");
    QVBoxLayout* mappingBoxLayout = new QVBoxLayout();

    QHBoxLayout* motorNameLayout = new QHBoxLayout();
    this->label_motor_name = new QLabel("地铁名: ");
    this->input_motor_name = new QLineEdit();
    motorNameLayout->addWidget(this->label_motor_name);
    motorNameLayout->addWidget(this->input_motor_name);
    mappingBoxLayout->addLayout(motorNameLayout);

    QHBoxLayout* carriageNameLayout = new QHBoxLayout();
    this->label_carriage_name = new QLabel("车厢号: ");
    this->input_carriage_name = new QLineEdit();
    carriageNameLayout->addWidget(this->label_carriage_name);
    carriageNameLayout->addWidget(this->input_carriage_name);
    mappingBoxLayout->addLayout(carriageNameLayout);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    this->btn_check_validate = new QPushButton("检查");
    this->btn_force_stop = new QPushButton("强制停止");
    this->btn_stop_mapping = new QPushButton("停止");
    this->btn_start_mapping = new QPushButton("开始");
    // buttonLayout->addWidget(this->btn_check_validate);
    buttonLayout->addWidget(this->btn_force_stop);
    buttonLayout->addWidget(this->btn_stop_mapping);
    buttonLayout->addWidget(this->btn_start_mapping);
    mappingBoxLayout->addLayout(buttonLayout);

    this->mapping_box->setLayout(mappingBoxLayout);
    main_layout->addWidget(this->mapping_box);

    this->label_info = new QLabel();
    main_layout->addWidget(this->label_info);

    setLayout( main_layout );

    // // 创建一个定时器，用来定时发布消息
    // QTimer* output_timer = new QTimer( this );

    // 设置信号与槽的连接
    // connect(this->btn_check_validate, SIGNAL(clicked()), this, SLOT(onCheckValidate()));
    connect(this->btn_force_stop, SIGNAL(clicked()), this, SLOT(onForceStopMapping()));
    connect(this->btn_stop_mapping, SIGNAL(clicked()), this, SLOT(onStopMapping()));
    connect(this->btn_start_mapping, SIGNAL(clicked()), this, SLOT(onStartMapping()));

    // // 设置定时器的回调函数，按周期调用sendVel()
    // connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

    this->pub_mapping_cmd = nh_.advertise<automotive_msgs::NDTMappingReq>("/ndtmapping/control", 3);
    this->pub_force_stop = nh_.advertise<std_msgs::String>("/ndtmapping/force_stop", 3);
    this->leave_mapping_status();
    // this->mapping_client = nh_.serviceClient<automotive_msgs::OnlineMapping>("online_mapping");
    this->sub_ndt_mapping_status = nh_.subscribe<automotive_msgs::NDTMappingRes>("/ndtmapping/res", 1, boost::bind(&OnlineMapping::cb_ndt_mapping_status, this, _1));
    // this->reset();
}

void OnlineMapping::onCheckValidate() {
    QString motor = this->input_motor_name->text();
    QString carriage = this->input_carriage_name->text();
    if (check_dir_validate(motor, carriage)) {
        QMessageBox::information(this, "检查地图数据目录", "Success!");
    }
}

bool OnlineMapping::check_dir_validate(const QString motor, const QString carriage) {
    if (motor.isEmpty()) {
        QMessageBox::warning(this, "检查地图目录设置", "地铁名不可为空");
        return false;
    }
    if (carriage.isEmpty()) {
        QMessageBox::warning(this, "检查地图目录设置", "车厢号不可为空");
        return false;
    }

    QString dir_str = this->base_dir+"/"+motor+"/"+carriage;
    QDir dir = QDir(dir_str);
    if (!dir.exists()) {
        dir.mkpath(dir_str);
        return true;
    }

    // 若该目录已存在, 则检查其是否为空
    QStringList files = dir.entryList();
    // for (int i = 0; i < files.size(); i++){
    //     ROS_INFO("file: %s", files[i].toStdString().c_str());
    // }
    if (files.size() > 2) {
        int clicked = QMessageBox::warning(this, "检查地图目录设置", "当前目录非空,请确认是否删除目录下所有文件?", QMessageBox::Yes, QMessageBox::No);
        if (clicked == QMessageBox::Yes){
            // 清空目录
            if (!dir.removeRecursively()){
                QMessageBox::warning(this, "检查地图目录设置", "权限不足, 无法删除当前目录文件, 请尝试手动删除");
                return false;
            }
            dir.mkpath(dir_str);  // removeBecursively会同时删除当前文件夹,因此重建一次
            return true;
        }else{
            QMessageBox::warning(this, "检查地图目录设置", "请选择其他目录, 或手动清空当前目录.");
            return false;
        }
    }

    return true;
}

void OnlineMapping::onForceStopMapping() {
    std_msgs::String msg;
    msg.data = "force stop ndt mapping";
    this->pub_force_stop.publish(msg);
}

void OnlineMapping::onStopMapping() {
    QString motor = this->input_motor_name->text();
    QString carriage = this->input_carriage_name->text();
    // automotive_msgs::OnlineMapping srv;
    // srv.request.start_mapping = false;
    // srv.request.save_dir = (this->base_dir + "/" + motor + "/" + carriage).toStdString();
    // if (!this->mapping_client.call(srv)){
    //     QMessageBox::warning(this, "开始建图", "无法找到建图服务, 请确认已连接到车辆并开机.");
    //     return;
    // }
    // if (srv.response.confirmed == false) {
    //     QMessageBox::warning(this, "开始建图", "建图服务不可用, 请重启车辆并检查网络和车辆设置.");
    //     return; 
    // }
    // this->leave_mapping_status();
    automotive_msgs::NDTMappingReq msg;
    msg.start_mapping = msg.STOP_MAPPING;
    msg.save_dir = (this->base_dir + "/" + motor + "/" + carriage).toStdString();
    this->pub_mapping_cmd.publish(msg);
    this->label_info->setText("正在停止建图...");
}

void OnlineMapping::onStartMapping() {
    // QString motor = this->input_motor_name->text();
    // QString carriage = this->input_carriage_name->text();
    // if (check_dir_validate(motor, carriage)) {
    //     // 发布开始建图指令
    //     automotive_msgs::OnlineMapping srv;
    //     srv.request.start_mapping = true;
    //     srv.request.save_dir = (this->base_dir + "/" + motor + "/" + carriage).toStdString();
    //     if (!this->mapping_client.call(srv)){
    //         QMessageBox::warning(this, "开始建图", "无法找到建图服务, 请确认已连接到车辆并开机.");
    //         return;
    //     }
    //     if (srv.response.confirmed == false) {
    //         QMessageBox::warning(this, "开始建图", "建图服务不可用, 请重启车辆并检查网络和车辆设置.");
    //         return; 
    //     }
        // this->enter_mapping_status();
    QString motor = this->input_motor_name->text();
    QString carriage = this->input_carriage_name->text();
    if ( check_dir_validate(motor, carriage) ) {
        automotive_msgs::NDTMappingReq msg;
        msg.start_mapping = msg.START_MAPPING;
        msg.save_dir = (this->base_dir + "/" + motor + "/" + carriage).toStdString();
        this->pub_mapping_cmd.publish(msg);
        this->label_info->setText("正在开启建图...");
    }
}

void OnlineMapping::enter_mapping_status(){
    this->btn_start_mapping->setDisabled(true);
    this->btn_stop_mapping->setDisabled(false);
    this->btn_check_validate->setDisabled(true);
    this->input_motor_name->setDisabled(true);
    this->input_carriage_name->setDisabled(true);
}

void OnlineMapping::leave_mapping_status(){
    this->btn_start_mapping->setDisabled(false);
    this->btn_stop_mapping->setDisabled(true);
    this->btn_check_validate->setDisabled(false);
    this->input_motor_name->setDisabled(false);
    this->input_carriage_name->setDisabled(false);
}

void OnlineMapping::cb_ndt_mapping_status(const automotive_msgs::NDTMappingResConstPtr &msg){
    if ( msg->is_mapping == msg->MAPPING_ON ) {
        this->enter_mapping_status();
        this->label_info->setText(QString::fromStdString("正在建图... "+msg->info));
    }
    if ( msg->is_mapping == msg->MAPPING_OFF ) {
        this->leave_mapping_status();
        this->label_info->setText(QString::fromStdString("建图已停止. "+msg->info));
    }
}

// 重载父类的功能
void OnlineMapping::save( rviz::Config config ) const
{
    rviz::Panel::save( config );
}

// 重载父类的功能，加载配置数据
void OnlineMapping::load( const rviz::Config& config )
{
    rviz::Panel::load( config );
}

} // end online_mapping

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(online_mapping::OnlineMapping,rviz::Panel)
// END_TUTORIAL