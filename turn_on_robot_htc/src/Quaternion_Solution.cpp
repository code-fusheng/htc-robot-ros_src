#include "Quaternion_Solution.h"
#define SAMPLING_FREQ 20.0f     // 采购频率

/**
 * 平方根倒数 求解四元数
 * 该函数使用了一种快速计算倒数平方根的方法，通常称为 "魔数" 方法。
 * 1. 首先，将输入的参数 number 乘以 0.5，并将结果保存在变量 x 中。
 * 2. 然后，将 number 的值保存在变量 y 中。
 * 3. 接下来，通过将 y 的内存表示转换为 long 类型，并使用特定的魔数（0x5f375a86）对其进行操作，实现对 y 的倒数平方根的近似计算。这个步骤使用了位运算和数值操作。
 * 4. 然后，将经过处理的 i 的内存表示转换回 float 类型，得到最终的倒数平方根近似值，保存在变量 y 中。
 * 5. 最后，使用牛顿迭代法对近似值进行一次修正，以提高精度。具体地，用 f - (x * y * y) 计算一个修正因子，然后将其与 y 相乘，得到更准确的倒数平方根值。
 * 6. 返回最终计算得到的倒数平方根值 y。
*/
float InvSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

/**
 * 四元数解算
*/
volatile float twoKp = 1.0f;     // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;     // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;      
    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    //误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // 计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / SAMPLING_FREQ);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ);
      integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ);
      gx += integralFBx;        // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;       // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / SAMPLING_FREQ));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / SAMPLING_FREQ));
  gz *= (0.5f * (1.0f / SAMPLING_FREQ));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx); 
  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  Mpu6050.orientation.w = q0;
  Mpu6050.orientation.x = q1;
  Mpu6050.orientation.y = q2;
  Mpu6050.orientation.z = q3;
}