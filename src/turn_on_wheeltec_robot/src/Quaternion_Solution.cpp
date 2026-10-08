#include "turn_on_wheeltec_robot/Quaternion_Solution.h"

/**************************************
Date: May 31, 2020
Function: 平方根倒数 求四元数用到
***************************************/
float QuaternionSolution::InvSqrt(float number)
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

// 冻结姿态解算标志，用于在静止状态时使用最后陀螺仪漂移后计算的禁止姿态，矫正静止时的漂移问题
void QuaternionSolution::SetFreeze(bool flag){
  imu_freeze = flag;
}

/**************************************
Date: May 31, 2020
Function: 四元数解算
  将IMU的原始xyz三轴线速度和三轴角速度计算姿态四元数
  原理是采用Mahony互补滤波（AHRS）算法融合底盘上传的IMU芯片的陀螺仪和加速度计分别测得的三轴角速度和三轴加速度计算姿态角
  基本过程是利用陀螺仪测的三轴角速度及上次姿态四元数进行积分计算当前姿态四元数，然后将垂直重力加速度方向在该姿态四元数下旋转，
  比较旋转后的重力加速度是否和加速度计测的三轴加速度合并后的加速度一致，并根据两者向量叉积误差去修正当前姿态四元数。
***************************************/
sensor_msgs::msg::Imu QuaternionSolution::operator()(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  
  sensor_msgs::msg::Imu Mpu6050;   // 移到开头（或冻结分支里构造）
  if(imu_freeze){
    RCLCPP_INFO(rclcpp::get_logger("total"),"Robot stop! IMU data freeze!\n");
    // 冻结时：保持上次姿态，返回当前 q0~q3 构造的消息
    Mpu6050.orientation.w = q0;
    Mpu6050.orientation.x = q1;
    Mpu6050.orientation.y = q2;
    Mpu6050.orientation.z = q3;
    return Mpu6050;
  }
  
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
  return Mpu6050;
}

void QuaternionSolution::operator()(sensor_msgs::msg::Imu& Imu_Data_Pub){
  Imu_Data_Pub.orientation = operator()(
    Imu_Data_Pub.angular_velocity.x, Imu_Data_Pub.angular_velocity.y, Imu_Data_Pub.angular_velocity.z,
    Imu_Data_Pub.linear_acceleration.x, Imu_Data_Pub.linear_acceleration.y, Imu_Data_Pub.linear_acceleration.z
  ).orientation;
}
