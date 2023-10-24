#pragma once
#ifndef USB2CAN_CORE_H
#define USB2CAN_CORE_H

#include <can_msgs/battery.h>
#include <can_msgs/ecu.h>
#include <can_msgs/vehicle_status.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "MsgJD03.h"
#include "MsgNWD.h"
#include "MsgJD01.h"
#include "MsgMMWare.h"
#include "unistd.h"
#include <cstdlib>
#include <ctime>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <thread>

namespace USB2CAN
{
struct Param
{
  uint8_t mrun_num;
  std::string name;
  int debug_can_port;
  int main_can_port;
  ros::Publisher battery_status_publisher;
  ros::Publisher vehicle_status_publisher;
  ros::Publisher vehicle_status_low_rate_publisher;
};

enum CARTYPE{
  NWD = 0,
  JD03 = 1,
  JD01 = 2,
  JD02 = 3
};

enum CanType{
  USB2CAN = 0,
  EthCAN_UDP = 1
};

// static DWORD CAN_device = VCI_USBCAN2; // 如果变量要写到头文件.h中,则最好使用static,以避免multi
//                                        // definitation问题
static DWORD CAN_id = 0; // 默认采用第一个CAN设备, (目前也只有一个CAN设备,即USBCAN2)
// static DWORD CAN_port = 0;

static VCI_BOARD_INFO pInfo; // 获取/存储设备信息
static int count = 0;        // 数据列表中,泳衣存储列表序号

static VCI_INIT_CONFIG config;

static pthread_t threadid;
static pthread_t thread_can_recv;
static Param *param;

static double wheel_radius = 0;  // 车轮半径, 差速底盘中用以通过车轮rpm计算轮速
static double left_wheel_torque, left_wheel_speed, left_wheel_dc;
static double right_wheel_torque, right_wheel_speed, right_wheel_dc;

// void ExitHandler(int sig);

class CAN_app
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle p_nh;

  ros::Subscriber sub_ecu, sub_imu;
  ros::Publisher pub_vehicle_status, pub_vehicle_status_low_rate;
  ros::Publisher pub_battery_status;

  CanType can_type;  // 0: usb2can  1: Eth Can (UDP)
  // 0: usb2can
  int debug_can_port;
  int main_can_port;

  // 1: Eth Can (UDP)
  std::string can1_remote_ip, can2_remote_ip;
  int can1_remote_port, can2_remote_port;
  std::string local_ip;
  int local_port;
  int sock_fd;
  std::string can_eth_card;
  struct sockaddr_in can1_addr_serv, can2_addr_serv;
  int can1_addr_serv_len, can2_addr_serv_len;

  bool param_debug;
  bool param_show_sending_msg;

  double pre_steer;
  double vehicle_weight;

  CARTYPE car_type;

  std::vector<std::thread> tasks;

  /* mmWare */
  mmware_msgs::ObjectList object_list_;
  mmware_msgs::ClusterList cluster_list;
  //create map container for object list
  std::map<int,mmware_msgs::Object> object_map_;
  std::map<int,mmware_msgs::Cluster> cluster_map_;

  ros::Publisher pub_cluster;
  ros::Publisher pub_cluster_list;

  void initROS();

  void ecu_cb(const can_msgs::ecu::ConstPtr &msg);

  void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);

  void convertStringToType(std::string str);

  void publish_mmware_cluster_map();

public:
  CAN_app();
  ~CAN_app();

  void init_usb2can();
  void run_usb2can();

  void init_eth_can();
  void run_eth_can();
  void can_recv_func(int id);

  void run();

  // void ExitHandler(int sig);
};

} // namespace USB2CAN
#endif