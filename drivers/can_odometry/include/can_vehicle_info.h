#ifndef CAN_VEHICLE_INFO_H
#define CAN_VEHICLE_INFO_H

namespace can_odometry
{
inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

// convert degree to radian
inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

// convert degree to radian
inline double rad2deg(double rad)
{
  return rad * 180 / M_PI;
}

struct VehicleInfo
{
  bool is_stored;
  double wheel_base;  // 轮距
  double minimum_turning_radius;
  double maximum_steering_wheel_angle_deg;
  bool is_akm;  // 是否为阿克曼车型

  VehicleInfo()
  {
    is_stored = false;
    wheel_base = 0.0;
    minimum_turning_radius = 0.0;
    maximum_steering_wheel_angle_deg = 0.0;
  }
  double convertSteeringAngleToAngularVelocity(const double cur_vel_mps, const double cur_angle_rad)  // rad/s
  {
    return is_stored ? tan(cur_angle_rad) * cur_vel_mps / wheel_base : 0;
  }
  double getCurrentSteeringAngle(const double steering_wheel_angle_rad)  // steering wheel [rad] -> steering [rad]
  {
    return is_stored ?
               steering_wheel_angle_rad * getMaximumSteeringWheelAngle() / deg2rad(maximum_steering_wheel_angle_deg) :
               0;
  }
  double getMaximumSteeringWheelAngle()  // radian
  {
    return is_stored ? asin(wheel_base / minimum_turning_radius) : 0;
  }
};
}
#endif  // CAN_VEHICLE_INFO_H
