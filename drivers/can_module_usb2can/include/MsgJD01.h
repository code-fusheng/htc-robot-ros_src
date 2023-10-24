#ifndef MSGJD01_H
#define MSGJD01_H

#include "MsgBase.h"

namespace USB2CAN
{

// send vehicle control msg protocol
#define JD01_SCU_SHIFT_LEVEL_OFFSET 0
#define JD01_SCU_SHIFT_LEVEL_LENGTH 2 // 车辆档位请求： 0 为检测到或初始状态  1：D 2：N 3：R
#define JD01_SCU_ACC_MODE_OFFSET 2
#define JD01_SCU_ACC_MODE_LENGTH 2 // 加速模式  0不影响 1：加速1 2：加速2 3：加速3
#define JD01_SCU_BRAKE_MODE_OFFSET 4
#define JD01_SCU_BRAKE_MODE_LENGTH 2 // 制动模式 0：不影响 1：制动1 2：制动2 3：制动3
#define JD01_SCU_DRIVE_MODE_REQ_OFFSET 6
#define JD01_SCU_DRIVE_MODE_REQ_LENGTH 2 // 驾驶模式请求： 0：不影响 1：自动驾驶模式请求  2：驾驶员PAD模式请求  3：驾驶员方向盘模式请求
#define JD01_SCU_STEER_WHEEL_F_OFFSET 8
#define JD01_SCU_STEER_WHEEL_F_LENGTH 8 // Left wheel moter speed use to change direction
#define JD01_SCU_STEER_WHEEL_R_OFFSET 16
#define JD01_SCU_STEER_WHEEL_R_LENGTH 8 // Right wheel moter speed use to change direction
#define JD01_SCU_TARGET_SPEED_OFFSET 24
#define JD01_SCU_TARGET_SPEED_LENGTH 9
#define JD01_SCU_EBRAKE_OFFSET 33
#define JD01_SCU_EBRAKE_LENGTH 1
#define JD01_SCU_DOWNACC_SPEED_OFFSET 34
#define JD01_SCU_DOWNACC_SPEED_LENGTH 3
#define JD01_SCU_UPACC_SPEED_OFFSET 37
#define JD01_SCU_UPACC_SPEED_LENGTH 3


class MsgJD01: public SendMsgBase{
protected:
    void initCanMsg();
    void setDriveMode(DriveMode driveMode);
    void setBrakeMode(int v);
    void setAccMode(int v);
    void setShiftLevel(ShiftLevel shiftLevel);
    void setEBrake(bool need = false);
    void setSpeed(double v);
    void setWheelAngle(double angle);
    void rosMsg2canMsg();

public:
    MsgJD01():SendMsgBase()
    {
        initCanMsg();
    }

    MsgJD01(VCI_CAN_OBJ in_canMsg):SendMsgBase(in_canMsg){}

    MsgJD01(can_msgs::ecu ecuMsg, double pre_steer):SendMsgBase(ecuMsg, pre_steer)
    {
        initCanMsg();
    }

    ~MsgJD01(){}
    
    VCI_CAN_OBJ getMessage();
};


}

#endif