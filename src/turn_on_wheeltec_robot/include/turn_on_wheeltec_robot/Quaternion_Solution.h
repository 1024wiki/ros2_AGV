#ifndef __QUATERNION_SOLUTION_H_
#define __QUATERNION_SOLUTION_H_
#include <sensor_msgs/msg/imu.hpp> 
#include "rclcpp/rclcpp.hpp"
#define SAMPLING_FREQ 20.0f // 采样频率


// 采用Mahony互补滤波（AHRS）算法融合底盘上传的IMU芯片的陀螺仪和加速度计分别测得的三轴角速度和三轴加速度计算姿态角
class QuaternionSolution{
private:
    uint8_t imu_freeze = 0;         // 机器人静态冻结标志，防止陀螺仪在静止状态下的零点漂移问题
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;           // 上次IMU原始数据计算的姿态四元数历史值
    float twoKp = 1.0f;             // 2 * proportional gain (Kp)
    float twoKi = 0.0f;             // 2 * integral gain (Ki)
    float integralFBx = 0.0f;
    float integralFBy = 0.0f;
    float integralFBz = 0.0f;   // integral error terms scaled by Ki

    // 平方根倒数计算
    float InvSqrt(float number);
    
public:
    // 构造函数
    QuaternionSolution() = default;
    ~QuaternionSolution() = default;

    // Mahony互补滤波（AHRS）算法，输入IMU采集的xyz三轴角速度和xyz三轴加速度，得到姿态四元数
    sensor_msgs::msg::Imu operator()(float gx, float gy, float gz, float ax, float ay, float az);
    void operator()(sensor_msgs::msg::Imu& Imu_Data_Pub);

    // 冻结姿态解算标志，用于在静止状态时使用最后陀螺仪漂移后计算的禁止姿态，矫正静止时的漂移问题
    void SetFreeze(bool flag);
};

#endif


