#ifndef HTCBOT_CONTROL_H
#define HTCBOT_CONTROL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>
#include <QDir>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QGroupBox>
#include <QTabWidget>
#include <QComboBox>
#include <QStringList>
#include <std_msgs/String.h>
#include <htcbot_msgs/MapPathConf.h>
#include <htcbot_msgs/ModeSwitch.h>

namespace htcbot_control
{
class HtcbotControl : public rviz::Panel
{
  Q_OBJECT
public:
  HtcbotControl(QWidget *parent = 0);

public Q_SLOTS:
  void confirmTopic();
  void pubMappingSwitch();
  void pubLocalizerSwitch();

private:

  const QString base_dir = "/home/data";

  QVBoxLayout *main_layout_;

  QGroupBox *map_box_;
  QVBoxLayout *map_layout_;

  QHBoxLayout *parent_layout_;
  QLabel *parent_path_label_;
  QLineEdit *parent_path_input_;

  QHBoxLayout *child_layout_;
  QLabel *child_path_label_;
  QLineEdit *child_path_input_;

  QPushButton *reset_button_;
  QPushButton *confirm_button_;

  QPushButton *mapping_switch_button_;
  bool mapping_switch_value_ = false;

  QPushButton *localizer_switch_button_;
  bool localizer_switch_value_ = false;

  ros::Publisher topic_publisher_;
  // 场景/地图路径配置发布者
  ros::Publisher map_path_conf_pub_;
  ros::Publisher mode_switch_pub_;
  ros::NodeHandle nh_;

  QTabWidget *tab_widget_;
  QWidget *mapping_panel_;
  QWidget *navigation_panel_;
  QWidget *route_panel_;
  QWidget *home_panel_;

  void switchMappingPanel();
  void switchNavigationPanel();
  void switchRoutePanel();
  void switchHomePanel();

  bool checkDirValidate(const QString parent_text, const QString child_text);

};
}

#endif // HTCBOT_CONTROL_H
