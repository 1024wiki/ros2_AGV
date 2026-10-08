
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <iostream>
#include <string.h>
#include <string> 
#include <math.h> 
#include <stdlib.h>    
#include <unistd.h> 
#include "rclcpp/rclcpp.hpp"
#include <rcl/types.h>
#include <sys/stat.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <sys/types.h>
#include <serial/serial.h>

#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "wheeltec_robot_msg/msg/data.hpp" 
#include "wheeltec_robot_msg/msg/supersonic.hpp"
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

//回充相关新增
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <turtlesim/srv/spawn.hpp>

//超声波数据
#include <sensor_msgs/msg/range.hpp>

#include "turn_on_wheeltec_robot/Quaternion_Solution.h"
using namespace std;

#define RESET   string("\033[0m")
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"

//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率
// 超声波测量距离相关变量
#define Distance_DATA_size 19
#define Distance_HEADER    0XFA //Frame_header //帧头
#define Distance_TAIL      0XFC //Frame_tail   //帧尾
//自动回充相关
#define AutoCharge_HEADER      0X7C //Frame_header //自动回充数据帧头
#define AutoCharge_TAIL        0X7F //Frame_tail   //自动回充数据帧尾
#define AutoCharge_DATA_SIZE    8   //下位机发送过来的自动回充数据的长度

//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  1671.84f  	

//extern sensor_msgs::msg::Imu Mpu6050;  //External variables, IMU topic data //外部变量，IMU话题数据


// 基础的运动底盘上行数据帧和下行数据帧
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	uint8_t tx[SEND_DATA_SIZE]; // 11字节的数据缓存区，包括：xyz三轴速度
	float X_speed;	       		// X轴速度
	float Y_speed;           	// Y轴速度
	float Z_speed;         		// Z轴速度
	unsigned char Frame_Tail; 
}SEND_DATA;
//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_     
{
	uint8_t rx[RECEIVE_DATA_SIZE];	// 24字节的数据缓冲区，包括：xyz三轴速度、IMU三轴加速度和三轴角速度、电池电压
	uint8_t Flag_Stop;
	unsigned char Frame_Header;
	float X_speed;  
	float Y_speed;  
	float Z_speed;  
	float Power_Voltage;	
	unsigned char Frame_Tail;
}RECEIVE_DATA;
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;	// X轴线速度或X位置
	float Y;	// Y轴线速度或Y位置
	float Z;	// Z轴线速度或Z位置
}Vel_Pos_Data;
//IMU数据结构体
typedef struct __MPU6050_DATA_
{
	short accele_x_data; 	// X轴加速度
	short accele_y_data; 	// Y轴加速度
	short accele_z_data; 	// Z轴加速度
    short gyros_x_data; 	// X轴角速度
	short gyros_y_data; 	// Y轴角速度
	short gyros_z_data; 	// Z轴角速度

}MPU6050_DATA;


//下位机发送的自动回充相关数据结构体
typedef struct _RECEIVE_AutoCharge_DATA_     
{
	uint8_t rx[AutoCharge_HEADER];  //8字节
	unsigned char Frame_Header;     //帧头
	unsigned char Frame_Tail;		//帧尾
}RECEIVE_AutoCharge_DATA;


//下位机向ROS发送的超声波数据结构体
typedef struct _DISTANCE_DATA_     
{
	uint8_t rx[Distance_DATA_size];
	unsigned char Frame_Header;
	unsigned char Frame_Tail;
}DISTANCE_DATA;
//超声波相关数据结构体
typedef struct _Distance_     
{
	float A = 5.1f;  
	float B = 5.1f;   
	float C = 5.1f;  
	float D = 5.1f;  
	float E = 5.1f;  
	float F = 5.1f;  
 	float G = 5.1f;  
	float H = 5.1f;  
}Supersonic_data;


//机器人底盘类：小车启动的最基本节点类
class turn_on_robot : public rclcpp::Node
{
public:
	// 构造函数：初始化操作，包括保存外部传入的参数及节点参数声明，并创建话题发布者和订阅者，并初始化串口通信对象
	turn_on_robot();  
	// 析构函数：关闭小车启动时，向下位机发送停止小车运动的帧
	~turn_on_robot(); 
	// 循环控制代码：工作的核心逻辑：循环获取下位机stm32串口方式发送的帧数据，进行处理并发布话题
	void Control();   
	// 通用串口驱动类，构造帧格式通过该串口发送给下位机、从下位机从该串口接收帧进行解析。使用前需要初始化指定串口名和波特率并打开
	serial::Serial Stm32_Serial; 

private:
	// 外部启动参数ymal配置
	string car_mode;		// 小车模型
	string usart_port_name;	// stm32底盘串口通信的串口设备路径
	int serial_baud_rate;   // 串口通信波特率
	bool ranger_avoid_flag;	// 底盘超声波避障开关
	bool ultrasonic_avoid;	// NAV2超声波避障开关
	string robot_frame_id;	// 机器人本体坐标
	string ekf_odom_frame_id;	// 里程计(EKF融合)坐标
	string raw_odom_frame_id;	// 未在EKF融合前的坐标
	string gyro_frame_id; 	// IMU(陀螺仪)坐标
	float odom_x_scale;				// 里程计X轴修正系数
	float odom_y_scale;				// 里程计Y轴修正系数
	float odom_z_scale_positive;    // 里程计Z轴(正转)修正系数
	float odom_z_scale_negative;	// 里程计Z轴(反转)修正系数
	
	// 串口通信的发送和接收的数据结构体
	// 基础底盘数据帧数组，包括：携带xyz三轴速度的下行帧和携带xyz三轴速度、IMU三轴加速度和三轴角速度的上行帧
	SEND_DATA Send_Data;       						// ROS2串口发送的底盘运动速度的数据结构体
	RECEIVE_DATA Receive_Data; 						// 串口接收下位机发送的底盘运动状态的数据结构体
	// 在接收到一个完整的上行基础底盘数据帧时进行读取并保存
	Vel_Pos_Data Robot_Vel;    // 机器人的xyz三轴线速度(接收的原始值进行加工后的)
	MPU6050_DATA Mpu6050_Data; // 机器人的IMU数据包括：xyz三轴加速度和角速度(接收的原始值进行加工后的)
	float Power_voltage;       // 电源电压(接收的原始值进行加工后的)
	// 接收上行帧的IMU的三轴角速度和三轴线速度计算姿态四元数仿函数
	QuaternionSolution Quaternion_Solution;
	// 接收上行帧的三轴速度原始值进行位移积分方式计算里程计当前位姿，进行里程计话题发布
	Vel_Pos_Data Robot_Pos;    // 机器人的xyz三轴位置(在上位机接收后通过位移积分计算，下位机只会发送速度、角速度、加速度，不会计算好位移后发送)
	// 协方差矩阵表示里程计话题数据的可信度，在运动状态下不可信，静止状态下可信，用于robt_pose_ekf功能包中进行EKF融合时判断使用odom里程计和imu数据
	// xyz三轴位置和三轴姿态角
	const double odom_pose_covariance_run[36] = {1e-3,    0,    0,   0,   0,    0, 
													0, 1e-3,    0,   0,   0,    0,
													0,    0,  1e6,   0,   0,    0,
													0,    0,    0, 1e6,   0,    0,
													0,    0,    0,   0, 1e6,    0,
													0,    0,    0,   0,   0,  1e3 };
	const double odom_pose_covariance_stop[36] = {1e-9,    0,    0,   0,   0,    0, 
													 0, 1e-3, 1e-9,   0,   0,    0,
													 0,    0,  1e6,   0,   0,    0,
													 0,    0,    0, 1e6,   0,    0,
													 0,    0,    0,   0, 1e6,    0,
													 0,    0,    0,   0,   0, 1e-9 };
	// xyz三轴线速度和角速度
	const double odom_twist_covariance_run[36] = {1e-3,    0,    0,   0,   0,    0, 
													 0, 1e-3,    0,   0,   0,    0,
													 0,    0,  1e6,   0,   0,    0,
													 0,    0,    0, 1e6,   0,    0,
													 0,    0,    0,   0, 1e6,    0,
													 0,    0,    0,   0,   0,  1e3 };
	const double odom_twist_covariance_stop[36] = {1e-9,    0,    0,   0,   0,    0, 
													  0, 1e-3, 1e-9,   0,   0,    0,
													  0,    0,  1e6,   0,   0,    0,
													  0,    0,    0, 1e6,   0,    0,
													  0,    0,    0,   0, 1e6,    0,
													  0,    0,    0,   0,   0, 1e-9} ;


	// 自动回充数据帧数组，包括：含充电电流、红外信号、充电状态等的上行帧
	RECEIVE_AutoCharge_DATA Receive_AutoCharge_Data;// 串口接收下位机发送的自动回充数据结构体
	// 在接收到一个完整的上行自动回充数据帧时进行读取并保存
	bool Charging=0;           			// 机器人是否在充电的标志位
	float Charging_Current=0;  			// 充电电流
	uint8_t Red=0;             			// 机器人是否寻找到红外信号(充电桩)的标志位
	bool charge_set_state = false;		// 回充设置状态
	bool check_AutoCharge_data = false;	// 表示接收到一个完整的回充状态帧等待发布
	// 用于接收外部话题的自动回充标志
	int8_t AutoRecharge=0;				// 表示是否开启自动回充功能


	// 超声波数据帧，包括：含 6 路超声波距离的上行帧
	DISTANCE_DATA Distance_Data; // 串口接收下位机发送的超声波数据结构体
	// 在接收到一个完整的上行自动回充数据帧时进行读取并保存
	bool distance_flag=false;	// 表示接收到一个完整的超声波上行帧
	// 保存接收的超声波传感器数据
	Supersonic_data distance;  // 超声波距离对象


	// 底盘安全等级设置位
	bool SecurityPLY = 0;

	// 机械臂上次位置
	double last_A,last_B,last_C,last_grap;

	// 历史版本遗留未使用
	string cmd_vel;
	
private:
	// 记录每次循环的时间间隔，用于计算里程计对时间积分求位移(里程)
	rclcpp::Time _Now;			// 本次循环开始时间
	rclcpp::Time _Last_Time;  	// 上次循环结束时间
	float Sampling_Time;        // 本次循环时间间隔，用于积分求位移(里程)
	// 从串口读取stm32运动底盘上传的各种帧数据并进行单位转化，但只在接收到一个完整的基础底盘数据帧时，才返回true
	bool Get_Sensor_Data_New(); 

	// 功能函数
	// BBC校验异或计算
	unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); // 基层底盘运动上行帧
	unsigned char Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char mode); // 自动回充上行帧
	uint8_t Check_Sum_Supersonic(const uint8_t* checkdata,uint16_t datalen);		// 超声波数据上行帧
	// 将网络字节序转化为实际数据并转化为国际单位
	short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);  //IMU数据转化读取
	float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //里程计数据转化读取

private:
	// 话题发布者
	// 基层底盘运动状态上行帧
	// IMU数据话题发布者：sensor_msgs::msg::Imu接口包括：xyzw姿态四元数和3*3协方差矩阵、xyz线速度和3*3协方差矩阵、xyz角速度和3*3协方差矩阵
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
	// 在Ctrol函数中循环采集到基础底盘运动帧时的IMU数据：三轴角速度和三轴加速度时，进行处理发布
	void Publish_ImuSensor(); // 发布IMU传感器话题
	// Odom话题发布者：nav_msgs::msg::Odometry接口包括：xyz位置及xyzw姿态四元数和6*6协方差矩阵、xyz线速度和xyz角速度和6*6协议差矩阵
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher; 
	// TF 广播器：发布 odom_raw → base_footprint 的动态 TF 坐标变换
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	// 在Ctrol函数中循环采集到基础底盘运动帧时的数据：三轴线速度并进行位移积分，进行处理发布
	void Publish_Odom();  // 发布里程计话题
	// voltage电压话题发布者
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher; 
	// 在Ctrol函数中循环采集到基础底盘运动帧时的电压数据，然后进行每10次发布一次电压数据
	void Publish_Voltage(); // 发布电源电压话题


	// 回充状态上行帧
	// 找到充电位置红外信息话题发布
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr RED_publisher;
	void Publish_RED();
	// 充电状态和充电模式话题发布
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Charging_publisher;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ChargingMode_publisher;
	void Publish_Charging();
	// 充电电流话题发布
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr Charging_current_publisher;
	void Publish_ChargingCurrent();
	

	// 超声波距离上行帧
	// 6路超声波话题发布者
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_a_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_b_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_c_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_d_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_e_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_f_publisher_;
	void Publish_distance(); // 发布超声波距离

private:
	// 话题订阅者
	// 小车速度话题/cmd_vel的订阅者，订阅回调函数Cmd_Vel_Callback
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
	// 订阅回调函数处理逻辑：将其他节点发布的小车三轴速度消息转化位下行帧进行串口发送给下位机，根据是否开启底盘超声波避障功能执行避障逻辑后的实际小车速度下发
	void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
	//避障处理函数
	geometry_msgs::msg::Twist avoid_obstacle(const geometry_msgs::msg::Twist& twist);

	// 机械臂arm_cmd话题订阅者，在订阅回调arm_cmd_Callback函数中实现接收到机械臂的控制消息然后封装成帧发送给下位机实现控制机械臂
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_cmd_Sub;
	void arm_cmd_Callback(const std_msgs::msg::Float32MultiArray arm_joint);

	//底盘安全等级话题chassis_security订阅者，在chassis_security话题订阅函数，用于接受发布者的安区等级并构造下行帧通过串口发送给stm32下位机
	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr Security_Sub;
	void Security_Callback(const std_msgs::msg::Int8 &Security_Flag);

	// 灯带set_rgb_color话题订阅者，订阅回调函数：实现将消息构造成帧发送给下位机，实现控制下位机灯带颜色
	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr SetRgb_Sub;
	void Set_LightRgb_Callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

	// 小车底盘超声波避障是否开启提示的RangerAvoidFlag话题订阅者，订阅回调函数RangerAvoidFlag_Callback仅仅进行提示
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr AvoidFlag_Sub;
	void RangerAvoidFlag_Callback(const std_msgs::msg::Bool::SharedPtr AvoidFlag);
	
	// 回充相关订阅者
	// 自动回充话题robot_recharge_flag订阅者，在订阅回调函数Recharge_Flag_Callback中向下发送一个下行帧表示是否开启自动回充
	rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr Recharge_Flag_Sub;
	void Recharge_Flag_Callback(const std_msgs::msg::Int8::SharedPtr Recharge_Flag); 
	
	// 寻找充电装的红外信号的订阅者
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Red_Vel_Sub;
	void Red_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux); 
	
	// 回充服务
	rclcpp::Service<turtlesim::srv::Spawn>::SharedPtr SetCharge_Service;
	void Set_Charge_Callback(const shared_ptr<turtlesim::srv::Spawn::Request> req,shared_ptr<turtlesim::srv::Spawn::Response> res);
};
#endif
