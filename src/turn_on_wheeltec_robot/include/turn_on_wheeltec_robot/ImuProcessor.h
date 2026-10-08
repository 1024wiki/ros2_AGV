#ifndef _IMUPROCESSOR_H_
#define _IMUPROCESSOR_H_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "turn_on_wheeltec_robot/Quaternion_Solution.h"

// 用于处理原始IMU数据在静止时，陀螺仪出现的零点漂移问题，进行过滤发布矫正的IMU数据
class ImuProcessor : public rclcpp::Node
{
public:
    ImuProcessor();

private:
    // 保存/odom话题订阅的里程计数据，用于判读是否静止
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;
    const double x_vel_threshold=0.05;  // x线速度静止状态阈值
    const double z_vel_threshold=0.05;  // z角速度静止状态阈值
    bool odom_vaild = false; // 标识是否接收到odom数据

    // 判断是否处于静止状态，累积次数超过static_threshold阈值时，判断为静止状态
    int static_count = 0;
    const int static_threshold = 10; // 静止次数阈值，也为静止冻结前的漂移次数

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    QuaternionSolution Quaternion_Solution;     // IMU数据姿态四元数计算

    // 订阅函数用于获取里程计数据用于判断是否处于静止状态
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 订阅函数用于在获取到里程计数据下，进行矫正静止状态的IMU数据
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
};



#endif