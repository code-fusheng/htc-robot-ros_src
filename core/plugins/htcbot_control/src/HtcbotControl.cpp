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
  mapping_conf_pub_ = nh_.advertise<htcbot_msgs::MappingConf>("htcbot/mapping_conf", 1);
  grid_map_build_pub_ = nh_.advertise<htcbot_msgs::GridMapBuild>("htcbot/grid_map_build", 1);

}

void HtcbotControl::confirmTopic()
{

  map_static_str_ = "";
  map_dynamic_str_ = "";
  pathes_str_ = "";

  // 获取输入框中的文本
  QString parent_text = parent_path_input_->text();
  QString child_text = child_path_input_->text();
  if (checkDirValidate(parent_text, child_text)) {
    // 发布消息到ROS话题
    htcbot_msgs::MapPathConf conf_msg;
    QString dir_str = base_dir+"/"+parent_text+"/"+child_text;
    map_static_str_ = dir_str+"/lidar_mode/pcd_map/static_map";
    QDir static_dir = QDir(map_static_str_);
    if (!static_dir.exists()) {
        static_dir.mkpath(map_static_str_);
    }
    map_dynamic_str_ = dir_str+"/lidar_mode/pcd_map/dynamic_map";
    QDir dynamic_dir = QDir(map_dynamic_str_);
    if (!dynamic_dir.exists()) {
        dynamic_dir.mkpath(map_dynamic_str_);
    }
    pathes_str_ = dir_str+"/lidar_mode/pathes";
    QDir pathes_dir = QDir(pathes_str_);
    if (!pathes_dir.exists()) {
        pathes_dir.mkpath(pathes_str_);
    }
    conf_msg.map_static_path = (dir_str+"/lidar_mode/pcd_map/static_map").toStdString();
    conf_msg.map_dynamic_path = (dir_str+"/lidar_mode/pcd_map/dynamic_map").toStdString();
    conf_msg.route_path = (dir_str+"/lidar_mode/pathes").toStdString();
    map_path_conf_pub_.publish(conf_msg);  
  } 
}  

void HtcbotControl::pubMappingConf()
{
    mapping_switch_value_ = !mapping_switch_value_;
    QMessageBox::information(this, "当前建图开关状态", QString(mapping_switch_value_ ? "true" : "false"));
    htcbot_msgs::ModeSwitch switch_msg;
    switch_msg.mode = "Mapping";
    switch_msg.switch_to = mapping_switch_value_;
    mode_switch_pub_.publish(switch_msg);

    htcbot_msgs::MappingConf conf_msg;
    conf_msg.mapping_state = mapping_switch_value_ ? 1 : 0;
    conf_msg.save_dir = map_static_str_.toStdString();
    conf_msg.voxel_size = mapping_voxel_input_->text().toFloat();
    conf_msg.step_size = mapping_step_input_->text().toFloat();
    mapping_conf_pub_.publish(conf_msg);
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

void HtcbotControl::pubRunningManualSwitch()
{
  manual_running_switch_value_ = !manual_running_switch_value_;
  manual_running_label_->setText(manual_running_switch_value_ ? "true" : "false");
  QMessageBox::information(this, "当前手动驾驶开关状态", QString(manual_running_switch_value_ ? "true" : "false"));
}

void HtcbotControl::pubRunningAutoSwitch()
{
  auto_running_switch_value_ = !auto_running_switch_value_;
  auto_running_label_->setText(auto_running_switch_value_ ? "true" : "false");
  QMessageBox::information(this, "当前自动驾驶开关状态", QString(auto_running_switch_value_ ? "true" : "false"));
}

void HtcbotControl::pubRunningAvoidSwitch()
{
  avoid_running_switch_value_ = !avoid_running_switch_value_;
  avoid_running_label_->setText(avoid_running_switch_value_ ? "true" : "false");
  QMessageBox::information(this, "当前自主避障开关状态", QString(avoid_running_switch_value_ ? "true" : "false"));
}

void HtcbotControl::pubBuildGridMap() 
{
  // /htcbot/grid_map_build
  htcbot_msgs::GridMapBuild build_msg;
  build_msg.input_dir = map_static_str_.toStdString();
  build_msg.output_dir = map_dynamic_str_.toStdString();
  build_msg.voxel_size = grid_voxel_input_->text().toFloat();
  build_msg.side_length = grid_side_input_->text().toFloat();
  grid_map_build_pub_.publish(build_msg);
}

void HtcbotControl::pubCarTaskStart()
{
  htcbot_msgs::ModeSwitch switch_msg;
  switch_msg.mode = htcbot_msgs::ModeSwitch::CAR_TASK;
  switch_msg.switch_to = htcbot_msgs::ModeSwitch::ON;
  mode_switch_pub_.publish(switch_msg);
}

void HtcbotControl::pubCarTaskPuse()
{
  htcbot_msgs::ModeSwitch switch_msg;
  switch_msg.mode = htcbot_msgs::ModeSwitch::CAR_TASK;
  switch_msg.switch_to = htcbot_msgs::ModeSwitch::OFF;
  mode_switch_pub_.publish(switch_msg);
}

void HtcbotControl::switchMappingPanel()
{
  mapping_panel_ = new QWidget(tab_widget_);
  tab_widget_->addTab(mapping_panel_, "在线建图");

  // 在这里添加建图面板的控件和逻辑
  QVBoxLayout *layout = new QVBoxLayout(mapping_panel_);

  QGroupBox *conf_box = new QGroupBox("建图控制");

  QVBoxLayout *conf_layout = new QVBoxLayout();

  // 创建降采样和步长输入框和按钮
  QHBoxLayout *voxel_layout = new QHBoxLayout();
  QLabel *voxel_label = new QLabel("voxel size");
  mapping_voxel_input_ = new QLineEdit(this);
  mapping_voxel_input_->setText("1.0");
  voxel_layout->addWidget(voxel_label);
  voxel_layout->addWidget(mapping_voxel_input_);
  conf_layout->addLayout(voxel_layout); 
  QHBoxLayout *step_layout = new QHBoxLayout();
  QLabel *step_label = new QLabel("step size");
  mapping_step_input_ = new QLineEdit(this);
  mapping_step_input_->setText("1.0");
  step_layout->addWidget(step_label);
  step_layout->addWidget(mapping_step_input_);
  conf_layout->addLayout(step_layout);

  QHBoxLayout *mapping_op_layout = new QHBoxLayout();
  QPushButton *mapping_start_button = new QPushButton("开始");
  QPushButton *mapping_end_button = new QPushButton("结束");
  mapping_op_layout->addWidget(mapping_start_button);
  mapping_op_layout->addWidget(mapping_end_button);
  conf_layout->addLayout(mapping_op_layout);

  connect(mapping_start_button, SIGNAL(clicked()), this, SLOT(pubMappingConf()));
  connect(mapping_end_button, SIGNAL(clicked()), this, SLOT(pubMappingConf()));

  conf_box->setLayout(conf_layout);

  QGroupBox *grid_box = new QGroupBox("分片控制");

  QVBoxLayout *grid_layout = new QVBoxLayout();

  QHBoxLayout *grid_voxel_layout = new QHBoxLayout();
  QLabel *grid_voxel_label = new QLabel("grid voxel size");
  grid_voxel_input_ = new QLineEdit(this);
  grid_voxel_input_->setText("1.0");
  grid_voxel_layout->addWidget(grid_voxel_label);
  grid_voxel_layout->addWidget(grid_voxel_input_);
  grid_layout->addLayout(grid_voxel_layout); 
  
  QHBoxLayout *grid_side_layout = new QHBoxLayout();
  QLabel *grid_side_label = new QLabel("grid side length");
  grid_side_input_ = new QLineEdit(this);
  grid_side_input_->setText("1.0");
  grid_side_layout->addWidget(grid_side_label);
  grid_side_layout->addWidget(grid_side_input_);
  grid_layout->addLayout(grid_side_layout);

  QHBoxLayout *grid_op_layout = new QHBoxLayout();
  QPushButton *build_button = new QPushButton("创建");
  grid_op_layout->addWidget(build_button);
  grid_layout->addLayout(grid_op_layout);

  connect(build_button, SIGNAL(clicked()), this, SLOT(pubBuildGridMap()));

  grid_box->setLayout(grid_layout);

  layout->addWidget(conf_box);
  layout->addWidget(grid_box);
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

  // 模块开关
  QGroupBox *mode_switch_box = new QGroupBox("模块开关");
  QVBoxLayout *mode_switch_layout = new QVBoxLayout();

  // 建图开始
  mapping_switch_button_ = new QPushButton("建图模块");
  mode_switch_layout->addWidget(mapping_switch_button_);
  connect(mapping_switch_button_, SIGNAL(clicked()), this, SLOT(pubMappingSwitch()));
  
  // 开启定位
  localizer_switch_button_ = new QPushButton("定位模块");
  mode_switch_layout->addWidget(localizer_switch_button_);
  connect(localizer_switch_button_, SIGNAL(clicked()), this, SLOT(pubLocalizerSwitch()));

  mode_switch_box->setLayout(mode_switch_layout);

  // 加载地图

  // 加载路径

  layout->addWidget(mode_switch_box);

  // 运行模式

  QGroupBox *running_type_box = new QGroupBox("运行模式");
  QVBoxLayout *running_type_layout = new QVBoxLayout();

  // 手动驾驶
  QHBoxLayout *mannual_running_layout = new QHBoxLayout();
  manual_running_button_ = new QPushButton("手动驾驶");
  manual_running_label_ = new QLabel();
  manual_running_label_->setText(manual_running_switch_value_ ? "true" : "false");
  mannual_running_layout->addWidget(manual_running_button_);
  mannual_running_layout->addWidget(manual_running_label_);
  mannual_running_layout->addStretch();
  connect(manual_running_button_, SIGNAL(clicked()), this, SLOT(pubRunningManualSwitch()));
  
  running_type_layout->addLayout(mannual_running_layout);

  // 自动驾驶
  QHBoxLayout *auto_running_layout = new QHBoxLayout();
  auto_running_button_ = new QPushButton("自动驾驶");
  auto_running_label_ = new QLabel();
  auto_running_label_->setText(auto_running_switch_value_ ? "true" : "false");
  auto_running_layout->addWidget(auto_running_button_);
  auto_running_layout->addWidget(auto_running_label_);
  auto_running_layout->addStretch();
  connect(auto_running_button_, SIGNAL(clicked()), this, SLOT(pubRunningAutoSwitch()));

  running_type_layout->addLayout(auto_running_layout);

  // 自主避障
  QHBoxLayout *avoid_running_layout = new QHBoxLayout();
  avoid_running_button_ = new QPushButton("自主避障");
  avoid_running_label_ = new QLabel();
  avoid_running_label_->setText(avoid_running_switch_value_ ? "true" : "false");
  avoid_running_layout->addWidget(avoid_running_button_);
  avoid_running_layout->addWidget(avoid_running_label_);
  avoid_running_layout->addStretch();
  connect(avoid_running_button_, SIGNAL(clicked()), this, SLOT(pubRunningAvoidSwitch())); 

  running_type_layout->addLayout(avoid_running_layout);

  running_type_box->setLayout(running_type_layout);

  layout->addWidget(running_type_box);

  // 任务控制

  QGroupBox *task_control_box = new QGroupBox("任务控制");
  QHBoxLayout *task_control_layout = new QHBoxLayout();

  task_start_button_ = new QPushButton("开始");
  task_puse_button_ = new QPushButton("暂停");
  task_end_button_ = new QPushButton("结束");
  task_control_layout->addWidget(task_start_button_);
  task_control_layout->addWidget(task_puse_button_);
  task_control_layout->addWidget(task_end_button_);

  task_control_box->setLayout(task_control_layout);

  connect(task_start_button_, SIGNAL(clicked()), this, SLOT(pubCarTaskStart()));
  connect(task_puse_button_, SIGNAL(clicked()), this, SLOT(pubCarTaskPuse()));
  connect(task_end_button_, SIGNAL(clicked()), this, SLOT(pubCarTaskPuse()));

  layout->addWidget(task_control_box);
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