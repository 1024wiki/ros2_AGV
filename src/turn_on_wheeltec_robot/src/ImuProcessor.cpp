#include "turn_on_wheeltec_robot/ImuProcessor.h"

// 用于处理原始IMU数据在静止时，陀螺仪出现的零点漂移问题，进行过滤发布矫正的IMU数据
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); 
  auto node = std::make_shared<ImuProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;  
} 

ImuProcessor::ImuProcessor() : Node("imu_processor")
{
    // 接收运动底盘节点发布的/imu/data_raw话题数据进行处理，矫正静止时的零点漂移问题
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/stm32_raw", 2,std::bind(&ImuProcessor::imuCallback, this, std::placeholders::_1));
    // 接收运动底盘节点发布的/odom话题数据判断是否处于静止状态
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom_raw", 2,std::bind(&ImuProcessor::odomCallback, this, std::placeholders::_1));
    // 发布矫正静止时的零点漂移问题后的IMU话题/imu/data_filtered数据
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/imu_filtered", 2);
}


// 订阅函数用于获取里程计数据用于判断是否处于静止状态
void ImuProcessor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 获取里程计的线速度和角速度
    linear_vel_ = msg->twist.twist.linear.x;
    angular_vel_ = msg->twist.twist.angular.z;
    // 标记已获取到里程计数据，未获取到时不处理IMU话题数据进行矫正
    odom_vaild=true;
}

// 订阅函数用于在获取到里程计数据下，进行矫正静止状态的IMU数据
void ImuProcessor::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // 初始时未获得里程计数据的情况下，不进行IMU数据过滤
    if(!odom_vaild) return;

    // 根据里程计数据统计静止状态次数
    if(std::abs(linear_vel_) < x_vel_threshold && std::abs(angular_vel_) < z_vel_threshold)
        static_count++;
    else
        static_count=0;
    // 连续10次发布的里程计统计数据都为静止时，判断为静止状态进行标记姿态解算过程
    if(static_count >= static_threshold)
        Quaternion_Solution.SetFreeze(true);
    else
        Quaternion_Solution.SetFreeze(false);

    // 静止状态或运动状态下的姿态四元数解算
    sensor_msgs::msg::Imu out = *msg;
    Quaternion_Solution(out);

    // 修正原始IMU数据的姿态四元数，并发布静止修正的姿态四元数后的IMU数据
    imu_pub_->publish(out);
}



