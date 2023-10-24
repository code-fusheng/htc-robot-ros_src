#include <MsgBase.h>

namespace USB2CAN
{

/* Common functions */
static inline void Quaternion2Euler(const geometry_msgs::Quaternion quat_msg, double *roll, double *pitch, double *yaw)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quat_msg, quat);
    tf::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);
    *roll = *roll * 180/M_PI;
    *pitch = *pitch * 180/M_PI;
    *yaw = *yaw * 180/M_PI;
    // printf("pitch=%.6f\n", *pitch);
}


/* For 发送车身姿态到vcu */
void SendPostureMsg::initImuMsg() {
    bzero(canMsg.Data, BUFFER_SIZE);
    canMsg.ID = 0x124;
    canMsg.SendType = 0x01;   // 0:正常发送,发送失败则重发  1:只发送一次
    canMsg.RemoteFlag = 0x00; // 0:数据帧  1:远程帧(数据段空)
    canMsg.ExternFlag = 0x00; // 0:标准帧  1:扩展帧
    canMsg.DataLen = 0x08;
}

void SendPostureMsg::setAngle(double angle) {
    int value = int(angle*10);
    writeInt(canMsg.Data, SCU_POSTURE_ANGLE_OFFSET, SCU_POSTURE_ANGLE_OFFSET, value);
}

void SendPostureMsg::setWeight(double weight) {
    int value = int(weight*10);
    writeInt(canMsg.Data, SCU_POSTURE_WEIGHT_OFFSET, SCU_POSTURE_WEIGHT_OFFSET, value);
}

void SendPostureMsg::rosMsg2canMsg() {
    double roll, pitch, yaw;
    Quaternion2Euler(this->imuMsg.orientation, &roll, &pitch, &yaw);
    this->setAngle(-pitch);  // IMU右手系, x前y左z上, 上坡时计算得到pitch为负数(TODO::换IMU验证). 对于vcu来讲, 上坡发送坡度为正值.
    this->setWeight(this->weight);
}

VCI_CAN_OBJ SendPostureMsg::getMessage() {
    this->rosMsg2canMsg();
    VCI_CAN_OBJ return_msg = this->canMsg;
    return return_msg;
}


// -*-*-*-*- Received canMsg: wheel status -*-*-*-*-

void WheelStatus::init() {
    this->torque = _torque();
    this->rpm = _rpm();
    this->dc = _dc();
}

int WheelStatus::_torque() {
    return readAsInt(this->canMsg.Data, CCU_TORQUE_MEASURED_OFFSET, CCU_TORQUE_MEASURED_LENGTH);
}

int WheelStatus::_rpm() {
    int ret = readAsInt(this->canMsg.Data, CCU_RPM_MEASURED_OFFSET, CCU_RPM_MEASURED_LENGTH);
    if (ret > 32767)
        ret -= 65536;
    return ret*this->lr;
}

int WheelStatus::_dc() {
    return readAsInt(this->canMsg.Data, CCU_DC_MEASURED_OFFSET, CCU_DC_MEASURED_LENGTH);
}

int WheelStatus::get_torque() {
    return this->torque;
}

int WheelStatus::get_rpm() {
    return this->rpm;
}

int WheelStatus::get_dc() {
    return this->dc;
}

} // end namespace USB2CAN