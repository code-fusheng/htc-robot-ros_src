#include "HtcbotControl.h"

namespace htcbot_control
{

HtcbotControl::HtcbotControl(QWidget *parent)
  : rviz::Panel(parent)
{

  main_layout_ = new QVBoxLayout;

  // 创建地图场景配置输入 Box 
  map_box_ = new QGroupBox("场景设置");
  map_layout_ = new QVBoxLayout();
  
  // 创建输入框和按钮
  parent_layout_ = new QHBoxLayout();
  parent_path_label_ = new QLabel("场景名称: ");
  parent_path_input_ = new QLineEdit(this);
  parent_layout_->addWidget(parent_path_label_);
  parent_layout_->addWidget(parent_path_input_);
  map_layout_->addLayout(parent_layout_); 

  child_layout_ = new QHBoxLayout();
  child_path_label_ = new QLabel("场景编号: ");
  child_path_input_ = new QLineEdit(this);
  child_layout_->addWidget(child_path_label_);
  child_layout_->addWidget(child_path_input_);
  map_layout_->addLayout(child_layout_); 

  QHBoxLayout *buttonLayout = new QHBoxLayout();  
  reset_button_ = new QPushButton("重置", this);
  confirm_button_ = new QPushButton("确定", this);
  buttonLayout->addWidget(reset_button_);
  buttonLayout->addWidget(confirm_button_);
  map_layout_->addLayout(buttonLayout);

  map_box_->setLayout(map_layout_);

  // 创建tab切换面板
  tab_widget_ = new QTabWidget(this);
  switchHomePanel();
  switchMappingPanel();
  switchNavigationPanel();
  switchRoutePanel();

  // 设置布局
  main_layout_->addWidget(map_box_);
  main_layout_->addWidget(tab_widget_);
  setLayout(main_layout_);

  // 连接按钮点击事件到槽函数
  connect(confirm_button_, SIGNAL(clicked()), this, SLOT(confirmTopic()));

  // 创建ROS话题发布者
  topic_publisher_ = nh_.advertise<std_msgs::String>("my_topic", 1);
  map_path_conf_pub_ = nh_.advertise<htcbot_msgs::MapPathConf>("htcbot/map_path_conf", 1);
  mode_switch_pub_ = nh_.advertise<htcbot_msgs::ModeSwitch>("htcbot/mode_switch", 1);
}

void HtcbotControl::confirmTopic()
{
  // 获取输入框中的文本
  QString parent_text = parent_path_input_->text();
  QString child_text = child_path_input_->text();
  if (checkDirValidate(parent_text, child_text)) {
    // 发布消息到ROS话题
    htcbot_msgs::MapPathConf conf_msg;
    QString dir_str = base_dir+"/"+parent_text+"/"+child_text;
    conf_msg.path_static = (dir_str+"/lidar_mode/pcd_map/static_map").toStdString();
    conf_msg.path_dynamic = (dir_str+"/lidar_mode/pcd_map/dynamic_map").toStdString();
    map_path_conf_pub_.publish(conf_msg);
  } 
}

void HtcbotControl::pubMappingSwitch()
{
    mapping_switch_value_ = !mapping_switch_value_;
    QMessageBox::information(this, "当前建图开关状态", QString(mapping_switch_value_ ? "true" : "false"));
    htcbot_msgs::ModeSwitch switch_msg;
    switch_msg.mode = "Mapping";
    switch_msg.switch_to = mapping_switch_value_;
    mode_switch_pub_.publish(switch_msg);
}

void HtcbotControl::pubLocalizerSwitch()
{
    localizer_switch_value_ = !localizer_switch_value_;
    QMessageBox::information(this, "当前定位开关状态", QString(localizer_switch_value_ ? "true" : "false"));
    htcbot_msgs::ModeSwitch switch_msg;
    switch_msg.mode = "Localizer";
    switch_msg.switch_to = localizer_switch_value_;
    mode_switch_pub_.publish(switch_msg);
}

void HtcbotControl::switchMappingPanel()
{
  mapping_panel_ = new QWidget(tab_widget_);
  tab_widget_->addTab(mapping_panel_, "在线建图");
  // 在这里添加建图面板的控件和逻辑

}

void HtcbotControl::switchNavigationPanel()
{
  navigation_panel_ = new QWidget(tab_widget_);
  tab_widget_->addTab(navigation_panel_, "导航");
  // 在这里添加导航面板的控件和逻辑
}

void HtcbotControl::switchRoutePanel()
{
  route_panel_ = new QWidget(tab_widget_);
  tab_widget_->addTab(route_panel_, "轨迹");
  // 在这里添加路线面板的控件和逻辑
}

void HtcbotControl::switchHomePanel()
{
  home_panel_ = new QWidget(tab_widget_);
  tab_widget_->addTab(home_panel_, "主界面");
  // 在这里添加主页面板的控件和逻辑
  QVBoxLayout *layout = new QVBoxLayout(home_panel_);

  // 建图开始
  mapping_switch_button_ = new QPushButton("建图模块");
  layout->addWidget(mapping_switch_button_);
  connect(mapping_switch_button_, SIGNAL(clicked()), this, SLOT(pubMappingSwitch()));
  
  // 开启定位
  localizer_switch_button_ = new QPushButton("定位模块");
  layout->addWidget(localizer_switch_button_);
  connect(localizer_switch_button_, SIGNAL(clicked()), this, SLOT(pubLocalizerSwitch()));
  
  // 加载地图

  // 加载路径
}

bool HtcbotControl::checkDirValidate(const QString parent_text, const QString child_text)
{
    if (parent_text.isEmpty()) {
        QMessageBox::warning(this, "检查场景配置", "场景名称不能为空");
        return false;
    }
    if (child_text.isEmpty()) {
        QMessageBox::warning(this, "检查场景配置", "场景编号不能为空");
        return false;
    }
    QString dir_str = base_dir+"/"+parent_text+"/"+child_text;
    QDir dir = QDir(dir_str);
    if (!dir.exists()) {
        dir.mkpath(dir_str);
        return true;
    }
    return true;
}

}

// 导出插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(htcbot_control::HtcbotControl, rviz::Panel)