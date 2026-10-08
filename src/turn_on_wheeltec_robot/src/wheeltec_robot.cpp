#include "turn_on_wheeltec_robot/wheeltec_robot.h"
#include "wheeltec_robot_msg/msg/data.hpp" 

using std::placeholders::_1;
using namespace std;

/**************************************
Date: January 28, 2021
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); //ROS initializes and sets the node name //ROS初始化 并设置节点名称 
  turn_on_robot Robot_Control;//Instantiate an object //实例化一个对象
  Robot_Control.Control();//Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作
  return 0;  
} 

/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot():rclcpp::Node ("wheeltec_robot")
{
  //清空数据
  Sampling_Time=0;
  Power_voltage=0;
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  //ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  //The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  
  // 参数声明，本来应该在turn_on_wheeltec_robot.launch.py启动节点时，从wheeltec_param.yaml中读取写入的
  this->declare_parameter<int>("serial_baud_rate",114200);  // stm32底盘串口波特率必须一致
  this->declare_parameter<bool>("ranger_avoid_flag",0);     // 底盘超声波避障开关
  this->declare_parameter<bool>("ultrasonic_avoid",0);      // NAV2 超声波避障开关
  this->declare_parameter<std::string>("usart_port_name", "/dev/wheeltec_controller");  // 连接stm32底盘串口进行串口通信的串口设备名
  this->declare_parameter<std::string>("ekf_odom_frame_id", "odom_ekfcombined"); // EKF融合里程计坐标系(在EKF节点融合的里程计数据)
  this->declare_parameter<std::string>("raw_odom_frame_id", "odom_raw");      // 原始里程计坐标系(原始里程计数据，未在EKF节点融合后的)
  this->declare_parameter<std::string>("robot_frame_id", "base_footprint"); // 机器人本体坐标系
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");       // IMU坐标系
  this->declare_parameter<std::string>("car_mode", "four_wheel_diff_bs");   // 车型
  this->declare_parameter<double>("odom_x_scale",1.0);                      // 里程计X轴修正系数
  this->declare_parameter<double>("odom_y_scale",1.0);                      // 里程计Y轴修正系数
  this->declare_parameter<double>("odom_z_scale_positive",1.0);             // 里程计Z轴(正转)修正系数
  this->declare_parameter<double>("odom_z_scale_negative",1.0);             // 里程计Z轴(反转)修正系数

  // 获取声明的参数值进行在保存在成员变量中
  this->get_parameter("ranger_avoid_flag", ranger_avoid_flag);
  this->get_parameter("ultrasonic_avoid", ultrasonic_avoid);
  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("ekf_odom_frame_id", ekf_odom_frame_id);  
  this->get_parameter("raw_odom_frame_id", raw_odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);
  this->get_parameter("car_mode", car_mode);
  this->get_parameter("odom_x_scale", odom_x_scale);
  this->get_parameter("odom_y_scale", odom_y_scale);
  this->get_parameter("odom_z_scale_positive", odom_z_scale_positive);
  this->get_parameter("odom_z_scale_negative", odom_z_scale_negative);
  //将car_mode转换为小写，便于后续判断车型模式
  std::transform(car_mode.begin(), car_mode.end(), car_mode.begin(), [](unsigned char c) { return std::tolower(c); });
       
  // 创建节点的话题发布者
  // 创建里程计话题发布者
  // nav_msgs::msg::Odometry接口包括：xyz位置及xyzw姿态四元数和6*6协方差矩阵、xyz线速度和xyz角速度和6*6协议差矩阵
  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom_raw", 2);    
  // 初始化 TF 广播器（发布 odom_raw → base_footprint 的动态 TF）
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  
  // 创建IMU话题发布者
  // sensor_msgs::msg::Imu接口包括：xyzw姿态四元数和3*3协方差矩阵、xyz线速度和3*3协方差矩阵、xyz角速度和3*3协方差矩阵
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/stm32_raw", 2); 

  // 创建电池电压话题发布者
  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);  

  //回充发布者
  Charging_publisher = create_publisher<std_msgs::msg::Bool>("robot_charging_flag", 10);
  Charging_current_publisher = create_publisher<std_msgs::msg::Float32>("robot_charging_current", 10);
  RED_publisher = create_publisher<std_msgs::msg::UInt8>("robot_red_flag", 10);
  ChargingMode_publisher = create_publisher<std_msgs::msg::Bool>("robot_charging_mode", 1);

  //超声波发布者
  range_a_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_A", 10);
  range_b_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_B", 10);
  range_c_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_C", 10);
  range_d_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_D", 10);
  range_e_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_E", 10);
  range_f_publisher_ = create_publisher<sensor_msgs::msg::Range>("ultrasonic_data_F", 10);


  // 话题订阅者
  // 小车速度话题订阅者：接受发布的小车速度控制话题/cmd_vel接口geometry_msgs::msg::Twist数据，通过Cmd_Vel_Callback回调将发布速度由串口方式发送给stm32下位机
  // geometry_msgs::msg::Twist接口包括：xyz线速度、xyz角速度
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));

  // 机械臂话题订阅者：接收其他节点发送的arm_cmd消息，在订阅回调函数中将机械部控制封装成帧由串口方式发送给下位机
  arm_cmd_Sub = create_subscription<std_msgs::msg::Float32MultiArray>(
        "arm_cmd", 10, std::bind(&turn_on_robot::arm_cmd_Callback, this,std::placeholders::_1));

  // 底盘安全防护关闭话题订阅者
  Security_Sub = create_subscription<std_msgs::msg::Int8>(
        "chassis_security",2, std::bind(&turn_on_robot::Security_Callback, this,std::placeholders::_1));

  // 灯带颜色控制话题订阅者
  SetRgb_Sub = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "set_rgb_color", 5, std::bind(&turn_on_robot::Set_LightRgb_Callback, this,std::placeholders::_1));  

  // 底盘超声波避障标志开启订阅者
  AvoidFlag_Sub = create_subscription<std_msgs::msg::Bool>(
        "RangerAvoidFlag", 10, std::bind(&turn_on_robot::RangerAvoidFlag_Callback, this, _1));

  // 回充订阅者
  // 自动回充话题订阅者
  Recharge_Flag_Sub = create_subscription<std_msgs::msg::Int8>(
      "robot_recharge_flag", 10, std::bind(&turn_on_robot::Recharge_Flag_Callback, this, std::placeholders::_1));
  // 寻找充电装话题订阅者(未实现)
  //Red_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>("charge_red_flag", 10, std::bind(&turn_on_robot::Red_Vel_Callback,this, std::placeholders::_1));
  // 回充服务提供(未实现)
  //SetCharge_Service=this->create_service<turtlesim::srv::Spawn>("/set_charge",std::bind(&turn_on_robot::Set_Charge_Callback,this,std::placeholders::_1 ,std::placeholders::_2));
  
  //速度控制命令订阅回调函数设置
  /*if(ultrasonic_avoid == 1) 
  {
    cmd_vel="/cmd_vel_avoid";
    RCLCPP_INFO(this->get_logger(),"wheeltec_robot use ultrasonic avoid");
  }
  else {
    cmd_vel="/cmd_vel";
    RCLCPP_INFO(this->get_logger(),"wheeltec_robot not use ultrasonic avoid");

  }
  */
  
  // 初始化验证是否可以和下位机stm32串口通信
  try
  { 
    //Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate); //Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); //Open the serial port //开启串口
    Stm32_Serial.flushInput();
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),"wheeltec_robot can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  } 
  if(Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(),"wheeltec_robot serial port opened"); //Serial port opened successfully //串口开启成功提示
  }

  // 小车底层启动节点初始化成功
  RCLCPP_INFO(this->get_logger(),"wheeltec_robot Data init successful!"); 
}

/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  // 对象turn_on_robot析构关闭小车前，向下位机stm32发送速度为0的停止运动的帧
  Send_Data.tx[0]=FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  //The target velocity of the X-axis of the robot //机器人X轴的目标线速度 
  Send_Data.tx[4] = 0;     
  Send_Data.tx[3] = 0;  

  //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;  

  //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
  Send_Data.tx[8] = 0;  
  Send_Data.tx[7] = 0;    
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; 
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }

  // 关闭前，向下位机的机械臂发送停止运动进行复位的帧
  Send_Data.tx[0]=0xAA;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  Send_Data.tx[3]=((short)(1.5707*1000))>>8;
  Send_Data.tx[4]=(short)(1.5707*1000);

  Send_Data.tx[5]=((short)(0.3917*1000))>>8;
  Send_Data.tx[6]=(short)(0.3917*1000);
 
  //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
  Send_Data.tx[7] = 0;    
  Send_Data.tx[8]=Check_Sum(8,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[9]=0xBB; 

  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }

  Stm32_Serial.close(); //Close the serial port //关闭串口  
  RCLCPP_INFO(this->get_logger(),"Shutting down"); //Prompt message //提示信息
}


/**************************************
Date: November 18, 2021
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
【背景知识】下位机(STM32)通过同一条串口线，可能发 3 种不同的数据帧，用不同的帧头/帧尾区分：
  ① 基础底盘数据帧：帧头 0x7B，帧尾 0x7D，总长 24 字节
     （含底盘X/Y/Z速度、IMU三轴加速度/角速度、电池电压）
  ② 自动回充数据帧：帧头 0x7C，帧尾 0x7F，总长 8 字节
     （含充电电流、红外信号、充电状态等）
  ③ 超声波数据帧：  帧头 0xFA，帧尾 0xFC，总长 19 字节
     （含 6 路超声波距离）

 串口是"字节流"——一次只能读 1 个字节，必须自己把字节攒成完整的一帧。
 所以本函数使用"状态机 + 计数器"的方式：
   - 每次调用只读 1 个字节
   - 用 count/count2/count3 记录"当前这帧已经攒了几个字节"
   - 攒够一帧长度后，校验帧尾 + BCC，通过就解析数据

 返回值 true  = 成功收到并解析了一帧完整的"基础底盘数据帧"(24字节)
        false = 还没凑够一帧 / 帧校验失败（下次调用继续攒）
***************************************/
bool turn_on_robot::Get_Sensor_Data_New()
{
  short transition_16=0; // 中间变量，用于把"高字节+低字节"拼成16位整数
  // check/check2/check3 : 三种帧各自的 BCC 校验值
  // error/error2/error3 : 三种帧各自的校验结果标记（0=校验通过）
  // Receive_Data_Pr[1]  : 临时数组，存放本次从串口读到的 1 个字节
  uint8_t check=0,check2=0,check3=0, error=1,error2=1,error3=1,Receive_Data_Pr[1];

  // static 变量：跨多次调用保持值，不会因为函数返回而清零！
  // count / count2 / count3 : 三种帧各自的"已接收字节数"计数器
  // Last_Receive : 上一次读到的字节（用于判断"上一个是帧尾吗？"）
  static int count,count2,count3;
  static uint8_t Last_Receive;

  // ================= 第 1 步：从串口读 1 个字节 =================
  Stm32_Serial.read(Receive_Data_Pr,sizeof(Receive_Data_Pr)); 

  // 把这个字节分别存入三种帧的接收数组（同一个字节，三个数组都存，
  // 哪个帧计数器在攒，就相当于它接收到了这个字节）
  Receive_Data.rx[count] = Receive_Data_Pr[0]; // 基础底盘数据帧数组
  Receive_AutoCharge_Data.rx[count3] = Receive_Data_Pr[0]; // 回充帧数组
  Distance_Data.rx[count2] = Receive_Data_Pr[0]; // 超声波帧数组

  // 取数组第0字节当帧头、第23字节当帧尾（用于判断/调试）
  Receive_Data.Frame_Header = Receive_Data.rx[0]; 
  Receive_Data.Frame_Tail = Receive_Data.rx[23];  

  // ================= 第 2 步：三种帧的"对齐/计数"状态机 =================
  // 原理：当上一个字节是"某个帧的帧尾"、而当前字节是"某个帧的帧头"时，
  //       说明新的一帧开始了，计数器从 1 重新数；
  //       否则如果已经在攒一帧（count>0），就继续累加；
  //       其他情况（既不是帧头也不是攒帧中）计数清零，等待下一个帧头。

  // ① 基础底盘数据帧：帧头 0x7B
  //    判断"新帧开始"：上一个字节是回充帧尾(0x7F) 或 底盘帧尾(0x7D)，
  //                    且当前字节是帧头 0x7B
  if( ((Last_Receive==AutoCharge_TAIL || Last_Receive==FRAME_TAIL ) && Receive_Data_Pr[0] == FRAME_HEADER) || 
        count>0 )
    count++;          // 已在一帧内：字节数 +1
  else
    count=0;          // 还没找到帧头：清零，继续找

  // ② 自动回充数据帧：帧头 0x7C
  if( ((Last_Receive==Distance_TAIL || Last_Receive==FRAME_TAIL ) && Receive_Data_Pr[0] == AutoCharge_HEADER) || 
        count3>0 )
    count3++;
  else
    count3=0;

  // ③ 超声波数据帧：帧头 0xFA
  if( (Last_Receive==FRAME_TAIL && Receive_Data_Pr[0] == Distance_HEADER) || 
        count2>0 )
    count2++;
  else
    count2=0;

  // 记录本次字节，供下一次调用判断"上一个字节是不是帧尾"
  Last_Receive = Receive_Data_Pr[0];

  // ================= 第 3 步：自动回充数据帧解析（8字节） =================
  // 帧格式：rx[0]=帧头0x7C, rx[1..2]=充电电流, rx[3]=红外信号, rx[4]=充电状态,
  //         rx[5]=回充设置状态, rx[6]=BCC校验, rx[7]=帧尾0x7F
  if(count3 == AutoCharge_DATA_SIZE)   // 8字节凑齐了
  {
    count3=0;  // 计数器清零，准备接收下一帧
    if(Receive_AutoCharge_Data.rx[AutoCharge_DATA_SIZE-1]==AutoCharge_TAIL) // 先确认最后一个字节是帧尾0x7F
    {
      check3 =  Check_Sum_AutoCharge(6,0);  // 对前6个字节(rx[0]~rx[5])做BCC异或校验
      if(check3 == Receive_AutoCharge_Data.rx[AutoCharge_DATA_SIZE-2]) // 计算值与第7字节(校验位)一致
      {
        error3=0;  // 校验通过
      }
      if(error3 == 0)  // 只有校验通过才解析数据
      {
        transition_16 = 0;
        // 把 rx[1](高8位) 和 rx[2](低8位) 拼成 16 位整数（充电电流，单位 0.001A）
        transition_16   |=  Receive_AutoCharge_Data.rx[1]<<8;
        transition_16   |=  Receive_AutoCharge_Data.rx[2]; 
        // 定点数转浮点：千分之一精度，单位从 mA 转为 A
        Charging_Current = transition_16/1000+(transition_16 % 1000)*0.001; //充电电流 
        Red =  Receive_AutoCharge_Data.rx[3];    // 红外接收状态（是否找到充电桩的红外信号）
        Charging = Receive_AutoCharge_Data.rx[4];// 小车充电状态（是否在充电）
        charge_set_state = Receive_AutoCharge_Data.rx[5]; // 回充设置状态
        check_AutoCharge_data = true; // 置标志位：告诉 Control() 有新的回充数据可发布
      }
    }
  }

  // ================= 第 4 步：超声波数据帧解析（19字节） =================
  // 帧格式：rx[0]=帧头0xFA, rx[1..12]=6路距离(A~F，各2字节，单位0.001m),
  //         rx[13..16]=自检/预留, rx[17]=BCC校验, rx[18]=帧尾0xFC
  if(count2 == Distance_DATA_size)   // 19字节凑齐了
  {
    count2 = 0;  // 计数器清零
    if(Distance_Data.rx[Distance_DATA_size-1]==Distance_TAIL) // 确认最后一个字节是帧尾0xFC
    {
      check2 = Check_Sum_Supersonic(Distance_Data.rx,17); // 对前17个字节(rx[0]~rx[16])做BCC校验
      if(check2 == Distance_Data.rx[Distance_DATA_size-2]) // 与第18字节(校验位)比较
      {
        error2=0;  // 校验通过
      }
      if(error2==0)  // 校验通过才解析
      {
        // 每一路都是"高8位<<8 | 低8位"拼成16位，再除以1000转成米(m)
        // 前面转 (short) 是为了正确处理负数（有符号）
        distance.A = ((short)((Distance_Data.rx[1]<<8) |(Distance_Data.rx[2] )))/1000.0f;
        distance.B = ((short)((Distance_Data.rx[3]<<8) |(Distance_Data.rx[4] )))/1000.0f;
        distance.C = ((short)((Distance_Data.rx[5]<<8) |(Distance_Data.rx[6] )))/1000.0f;
        distance.D = ((short)((Distance_Data.rx[7]<<8) |(Distance_Data.rx[8] )))/1000.0f;
        distance.E = ((short)((Distance_Data.rx[9]<<8) |(Distance_Data.rx[10])))/1000.0f;
        distance.F = ((short)((Distance_Data.rx[11]<<8)|(Distance_Data.rx[12])))/1000.0f;
        distance.G = 0;  // 此协议只用到 A~F，G/H 固定为 0
        distance.H = 0;
        distance_flag=true; // 置标志位：告诉 Control() 有新的超声波数据可发布
      }
    }
  }

  // ================= 第 5 步：基础底盘数据帧解析（24字节，最重要的帧） =================
  // 帧格式：
  //   rx[0]          = 帧头 0x7B
  //   rx[1]          = 预留位(Flag_Stop)
  //   rx[2],rx[3]    = X 方向线速度 (高8位, 低8位) 单位 0.001 m/s
  //   rx[4],rx[5]    = Y 方向线速度 (高8位, 低8位) 单位 0.001 m/s
  //   rx[6],rx[7]    = Z 方向角速度 (高8位, 低8位) 单位 0.001 rad/s
  //   rx[8],rx[9]    = IMU X 轴加速度 (原始值)
  //   rx[10],rx[11]  = IMU Y 轴加速度
  //   rx[12],rx[13]  = IMU Z 轴加速度
  //   rx[14],rx[15]  = IMU X 轴角速度 (原始值)
  //   rx[16],rx[17]  = IMU Y 轴角速度
  //   rx[18],rx[19]  = IMU Z 轴角速度
  //   rx[20],rx[21]  = 电池电压 (高8位, 低8位) 单位 0.001 V
  //   rx[22]         = BCC 校验位
  //   rx[23]         = 帧尾 0x7D
  if(count == 24) // 24字节凑齐，一帧完整数据到达
  {
    count=0;  // 计数器清零，为下一帧做准备
    if(Receive_Data.Frame_Tail == FRAME_TAIL) // 校验帧尾是否为 0x7D
    {
      check=Check_Sum(22,READ_DATA_CHECK);  // 对前22字节(rx[0]~rx[21])做BCC异或校验

      if(check == Receive_Data.rx[22])  // 计算出的校验值 == 第23字节(校验位)
      {
        error=0;  // 校验通过，置 0
      }
      if(error == 0)  // 只有校验通过才解析数据（防止被干扰的脏数据污染里程计）
      {
        /*// 调试用：打印原始24字节 / ROS_INFO 已被注释
        ROS_INFO("%x-%x-...", ...); */

        // 电机使能位，0x00 时电机使能，其他值失能
        Receive_Data.Flag_Stop=Receive_Data.rx[1]; 

        // ---- 解析：底盘速度（rx[2]~rx[7]，每2字节一组，用 Odom_Trans 拼16位并除以1000）----
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]); // X方向线速度 (m/s)
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]); // Y方向线速度 (m/s)，仅全向/麦轮底盘有效
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); // Z方向角速度 (rad/s)

        // ---- 解析：IMU 原始数据（rx[8]~rx[19]）----
        // Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);   // IMU X轴加速度(原始ADC值)
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]); // IMU Y轴加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]); // IMU Z轴加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);  // IMU X轴角速度(原始值)
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);  // IMU Y轴角速度
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);  // IMU Z轴角速度

        // ---- 电池电压：rx[20]~rx[21] 拼16位，单位 mV → V ----
        transition_16 = 0;
        transition_16 |=  Receive_Data.rx[20]<<8;  // 高8位
        transition_16 |=  Receive_Data.rx[21];     // 低8位
        Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001; //毫伏(mv)->伏(v)

        return true;  // ⭐ 返回 true：成功收到并解析了一帧完整底盘数据
      }
    }
  }
  return false;  // 没凑够一帧 / 帧尾或校验不对 → 返回 false，下次继续攒
}

/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 主循环：不断"读串口→解析数据→更新位姿→发布话题→处理回调"

【这个函数是整个节点的"心脏"，运行在 main() 里：
   main() 创建 turn_on_robot 对象后，就一直阻塞在 Control() 里。
   它是一个"手动控制的 while 循环"，而不是用 rclcpp::spin() 自动转，
   因为它的核心任务是"不停读串口"，回调（cmd_vel等）只是顺带处理。
   注意：这个循环不 sleep，跑多快取决于串口读数据的速度（非实时）。】

【一图看懂本函数在节点中的位置】
   main()
     │
     ├─ rclcpp::init(argc, argv)          ← ROS2 初始化
     ├─ turn_on_robot Robot_Control;      ← 构造：打开串口、创建发布者/订阅者
     ├─ Robot_Control.Control();          ← ★ 本函数：主循环，一直跑到关机
     └─ return 0

【本函数 4 大任务（每轮循环依次执行）】
   ① 读串口 + 解析下位机数据（Get_Sensor_Data_New）
   ② 更新里程计位置 + IMU 姿态
   ③ 发布话题（odom / imu / 电压 / 回充 / 超声波）
   ④ 处理订阅回调（cmd_vel 等 → 发指令给下位机）
***************************************/
void turn_on_robot::Control()
{
  // 记录启动时的时刻，作为第一次"采样间隔"的基准
  _Last_Time = rclcpp::Node::now();
  while(rclcpp::ok())
  {
    try{
      // ---- 记录当前时刻 ----
      _Now = rclcpp::Node::now();

      // ---- 计算采样间隔 dt（两次循环之间的时间差，单位秒）----
      // Sampling_Time = 本次时刻 - 上次时刻
      // 这个 dt 是"里程计积分"的核心参数：位移 = 速度 × dt
      Sampling_Time = (_Now - _Last_Time).seconds();  
                   
      // 第 1 步：读串口并解析下位机数据（最重要的一步）
      // ========================================================
      // Get_Sensor_Data_New() 的工作方式（详见该函数注释）：
      //   · 每次调用只从串口读 1 个字节
      //   · 内部用计数器"攒帧"，攒够 24 字节（基础底盘数据帧）并校验通过
      //   · 解析出：X/Y/Z 速度、IMU 六轴数据、电池电压
      //   · 只有"完整收到并解析好一帧"才 return true
      // 所以：if 成立 = 这轮循环真的拿到了新的一帧底盘数据
      if (Get_Sensor_Data_New()) 
      {
        // 发布三个主要话题（把解析结果广播给整个 ROS 系统）
        // 发布 /odom（位置 + 速度 + 协方差矩阵）
        Publish_Odom();     //   → 上层（EKF、导航、RViz）用来知道"车在哪"
        // 发布 /imu/data_raw（四元数姿态 + 角速度 + 加速度）
        Publish_ImuSensor(); //   → EKF 融合用；rviz 显示姿态
        // 发布 PowerVoltage（电池电压）                     
        Publish_Voltage();  //   → 注意：内部每隔 11 次才真正发一次（降频）   
                             
        // 更新"上次时刻"，供下一轮循环计算采样间隔
        _Last_Time = _Now;
      }
      // （如果 Get_Sensor_Data_New() 返回 false，说明还没攒够一帧，
      //   本轮就不更新位姿，直接往下走——这也保证了"不会用旧数据重复积分"）
      
      // 第 2 步：发布自动回充数据
      // ========================================================
      // check_AutoCharge_data 标志位：
      //   由 Get_Sensor_Data_New() 在收到"回充数据帧(8字节)"且校验通过时置 true
      //   这里发布 3 个话题后清掉，等下一帧回充数据再置位
      if(check_AutoCharge_data)
      {
        Publish_Charging();  // 发布 robot_charging_flag：小车是否在充电
        Publish_RED();       // 发布 robot_red_flag：是否找到充电桩红外信号
        Publish_ChargingCurrent(); // 发布 robot_charging_current：充电电流(A)
        check_AutoCharge_data = false;  // 发布完清标志位，防止重复发布
      }

      // 第 3 步：发布超声波数据
      // ========================================================
      // distance_flag 标志位：
      //   由 Get_Sensor_Data_New() 在收到"超声波数据帧(19字节)"时置 true
      if(distance_flag==true)  
      {
        Publish_distance();  // 发布 6 路超声波距离（A~F）
        distance_flag = false;  // 清标志位
      }

      // 第 4 步：处理订阅回调（让下位机"收到指令"的关键一步）
      // ========================================================
      // rclcpp::spin_some()：非阻塞地"处理一次"当前已经到达的回调消息
      //
      // 【为什么用 spin_some 而不是 spin？】
      //   · spin() 会【阻塞】在这里，一直等回调，永远不返回
      //     → 那串口就没人读了，底盘数据全部丢失！
      //   · spin_some() 只处理"此刻已排队"的回调，处理完立刻返回
      //     → 主循环能继续读串口，两不耽误
      //
      // 这一行会触发的回调（构造函数里注册的）：
      //   · Cmd_Vel_Callback        ← /cmd_vel 速度指令 → 打包串口帧发给STM32
      //   · Security_Callback       ← chassis_security 安全开关
      //   · Recharge_Flag_Callback  ← robot_recharge_flag 回充开关
      //   · arm_cmd_Callback        ← arm_cmd 机械臂控制
      //   · Set_LightRgb_Callback   ← set_rgb_color 灯带设置
      //   · RangerAvoidFlag_Callback← RangerAvoidFlag 避障开关
      rclcpp::spin_some(this->get_node_base_interface());
    }
    catch (const rclcpp::exceptions::RCLError & e )
    {
      // 捕获 rclcpp 底层错误（如话题/时钟异常），打印日志后继续循环
      // 这样即使偶尔出错，节点也不会崩溃退出
      RCLCPP_ERROR(this->get_logger(),"unexpectedly failed whith %s",e.what()); 
    }
  }
}



/**************************************
Date: January 28, 2021
Function: Data conversion function
功能: 数据转换函数，将网络字节序转化为实际数据大小，并统一单位
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  short transition_16;
  transition_16 = 0;
  // 网络字节序
  transition_16 |=  Data_High<<8;   
  transition_16 |=  Data_Low;
  return transition_16;     
}
float turn_on_robot::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  // 网络字节序
  transition_16 |=  Data_High<<8;  // 获取数据的高8位
  transition_16 |=  Data_Low;      // 获取数据的低8位
  data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // 速度单位从mm/s转换为m/s
  return data_return;
}

//设置灯带颜色
void turn_on_robot::Set_LightRgb_Callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  // 检查数据长度是否为 4
  if (msg->data.size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "Data length error. It must be 4. Got %zu elements.", msg->data.size());
    return;
  }

  // 解析数据：索引 0 为 bool (0 或 1)，索引 1-3 为 uint8
  uint8_t en_flag = (msg->data[0] == 1);
  uint8_t r = msg->data[1];
  uint8_t g = msg->data[2];
  uint8_t b = msg->data[3];

  Send_Data.tx[0]=FRAME_HEADER;
  Send_Data.tx[1] = 04; //04代表本帧的功能为设置灯带
  Send_Data.tx[2] = en_flag;//设置使能位
  Send_Data.tx[3] = r;
  Send_Data.tx[4] = g;
  Send_Data.tx[5] = b;
  Send_Data.tx[6] = 0;
  Send_Data.tx[7] = 0;
  Send_Data.tx[8] = 0;
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);  //BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}


//避障实现函数
geometry_msgs::msg::Twist turn_on_robot::avoid_obstacle(const geometry_msgs::msg::Twist& twist)
{
  geometry_msgs::msg::Twist avoid_twist = twist; // 复制输入的 twist 数据

  float base_vz = PI/4.0f;

	//避障标志位-前
	uint8_t avoid_front = 0;
	
	//避障标志位-后
	uint8_t avoid_back = 0;

  if( ranger_avoid_flag == 1 )
  {
    //s300 mini 车型
    if( car_mode.find("s300") != std::string::npos && car_mode.find("mini") != std::string::npos )
    {
      if( distance.A < 0.4f || distance.B < 0.4f || distance.C < 0.4f || 
        distance.D < 0.4f || distance.E < 0.4f )
      {
        avoid_front=1;
        if( avoid_twist.linear.x > 0 )
        {
          if( distance.A+distance.B > distance.D+distance.E)
          {
            //左转
            avoid_twist.angular.z = (avoid_twist.linear.x/0.5f)*base_vz;
          }
          else
          {
            //右转
            avoid_twist.angular.z = -(avoid_twist.linear.x/0.5f)*base_vz;
          }
          avoid_twist.linear.x = 0;
        }
      }
    }

    //s300
    else if ( car_mode.find("s300") != std::string::npos)
    {
      if( distance.A < 0.4f || distance.B < 0.4f || distance.C < 0.4f || 
        distance.D < 0.4f || distance.E < 0.4f || distance.F < 0.4f )
      {
        avoid_front=1;

        if( avoid_twist.linear.x > 0 )
        {
          if( distance.A+distance.B+distance.D > distance.D+distance.E+distance.B)
          {
            //左转
            avoid_twist.angular.z = (avoid_twist.linear.x/0.5f)*base_vz;
          }
          else
          {
            //右转
            avoid_twist.angular.z = -(avoid_twist.linear.x/0.5f)*base_vz;
          }
          avoid_twist.linear.x = 0;
        }
      }
    }

    //s200 / s260
    else if ( car_mode.find("s200") != std::string::npos || car_mode.find("s260") != std::string::npos )
    {
      if( distance.A < 0.4f || distance.B < 0.4f || distance.C < 0.4f || distance.D < 0.4f )
      {
        avoid_front=1;
        if( avoid_twist.linear.x > 0 && avoid_front==1 )
        {
          if( distance.A+distance.B > distance.C+distance.D )
          {
            //右转
            avoid_twist.angular.z = -(avoid_twist.linear.x/0.5f)*base_vz;
          }
          else
          {
            //左转
            avoid_twist.angular.z = (avoid_twist.linear.x/0.5f)*base_vz;
          }
          avoid_twist.linear.x=0;
        }
      }

      if( distance.E < 0.4f || distance.F < 0.4f)
      {
        avoid_back=1;
        if( avoid_twist.linear.x < 0 && avoid_back==1 && charge_set_state==0 )
        {
          avoid_twist.linear.x = 0;//后避障仅禁止后退,且自动回充不开启时才使能
        }
      }
    }

    //割草机
    else if( car_mode.find("v650") != std::string::npos )
    {

    }

    //未指定的车型
    else
    {
      avoid_front=0;
      avoid_back=1;
    }
  }

  return avoid_twist;
}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr ori_twist_aux)
{
  short  transition;  //intermediate variable //中间变量

  Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
  Send_Data.tx[1] = AutoRecharge; //set aside //预留位
  Send_Data.tx[2] = SecurityPLY; //set aside //预留位

  // 避障配置，计算避障后的三轴速度，然后进行封装成基础底盘运动控制下行帧，通过串口方式发送给下位机
  geometry_msgs::msg::Twist avoid_twist = avoid_obstacle(*ori_twist_aux);

  //机器人x轴的目标线速度
  transition=0;
  transition = avoid_twist.linear.x*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  //机器人y轴的目标线速度
  transition=0;
  transition = avoid_twist.linear.y*1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition>>8;

  //机器人z轴的目标角速度
  transition=0;
  transition = avoid_twist.angular.z*1000;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition>>8;

  //BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); 
  Send_Data.tx[10]=FRAME_TAIL; //帧尾0X7D

  // 通过串口将三轴速度的基础底盘控制的下行帧发送给下位机stm32
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}


/**
 * @brief 机械臂控制命令回调函数
 *
 * @param arm_cmd_msg 包含机械臂控制数据的消息，data[0-2]为三个关节角度，data[3]为夹爪状态
 *
 * 该函数接收机械臂控制指令，检测关节角度和夹爪状态的变化，
 * 将控制数据转换为串口通信协议格式后，通过串口发送给STM32下位机。
 * 仅当控制数据发生显著变化时（角度变化>0.01度或夹爪状态改变）才发送数据。
 */
void turn_on_robot::arm_cmd_Callback(const std_msgs::msg::Float32MultiArray arm_cmd_msg)
{
  // 判断是否需要控制
  if(fabs(last_A-arm_cmd_msg.data[0])>0.01||
      fabs(last_B-arm_cmd_msg.data[1])>0.01||
      fabs(last_C-arm_cmd_msg.data[2])>0.01||
      last_grap!=arm_cmd_msg.data[3] ){
    last_A=arm_cmd_msg.data[0];
    last_B=arm_cmd_msg.data[1];
    last_C=arm_cmd_msg.data[2];
    last_grap=arm_cmd_msg.data[3];
  }
  else 
    return;
  
  // 构造发送控制机械部的下行数据帧
  Send_Data.tx[0]=0xAA;
  Send_Data.tx[1]=((short)(arm_cmd_msg.data[0]*1000))>>8;
  Send_Data.tx[2]=(short)(arm_cmd_msg.data[0]*1000);


  Send_Data.tx[3]=((short)(arm_cmd_msg.data[1]*1000))>>8;
  Send_Data.tx[4]=(short)(arm_cmd_msg.data[1]*1000);

  Send_Data.tx[5]=((short)(arm_cmd_msg.data[2]*1000))>>8;
  Send_Data.tx[6]=(short)(arm_cmd_msg.data[2]*1000);

  Send_Data.tx[7]=(uint8_t)arm_cmd_msg.data[3];
  Send_Data.tx[8]=Check_Sum(8,SEND_DATA_CHECK);
  Send_Data.tx[9]=0xBB;
 
  // 串口方式进行发送
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }

}


/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 发布 IMU 数据话题 /imu/data_raw

【作用】把 STM32 传来的 IMU 原始数据，打包成 ROS 标准消息
      sensor_msgs::msg::Imu 发布出去，供 EKF、导航、RViz 等使用。
【调用时机】在 Control() 主循环里，每次成功收到一帧底盘数据后调用。
【数据来源】Mpu6050 是一个全局 sensor_msgs::msg::Imu 对象，
           由 Quaternion_Solution()（姿态解算）和 Get_Sensor_Data_New() 填充。
【消息组成】sensor_msgs/Imu 包含 4 大部分：
      orientation(四元数姿态) + angular_velocity(角速度) + linear_acceleration(加速度) + 各自的协方差
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  // 实例化一条 IMU 消息（本函数内的临时变量）
  sensor_msgs::msg::Imu Imu_Data_Pub;

  // ---- 1. 头部信息：时间戳 + 坐标系 ----
  Imu_Data_Pub.header.stamp = rclcpp::Node::now();       // 当前时间戳
  // IMU 坐标系名（默认 "gyro_link"），必须和 URDF 里的 IMU 坐标系一致，否则 robot_pose_ekf / EKF 融合会报 TF 错误
  Imu_Data_Pub.header.frame_id = gyro_frame_id;          

  // ---- 2. 角速度 angular_velocity（rad/s，绕 x/y/z 三轴的旋转速度）----
  // 数据来源：Get_Sensor_Data_New() 里在Mpu6050_Data中保存的IMU陀螺仪的 ADC 原始值
  // ---- 角速度：陀螺仪原始ADC值 → 物理单位 rad/s ----
  // 与STM32初始化IMU的量程有关，这里陀螺仪量程 ±500°/s
  // 因为机器人一般Z轴速度不快，降低ADC采集量程可以提高采集精度
  // GYROSCOPE_RATIO = 1/65.5/57.30 = 0.00026644（把±32768映射到±500°/s再转弧度rad/s）     
  Imu_Data_Pub.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO; // 绕X轴角速度（滚转 roll rate）
  Imu_Data_Pub.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO; // 绕Y轴角速度（俯仰 pitch rate）
  Imu_Data_Pub.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO; // 绕Z轴角速度（偏航 yaw rate）
  // 角速度协方差矩阵（与上面同理）：x/y 不信任，z 很信任
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;  // (0,0) roll 角速度
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;  // (1,1) pitch 角速度
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6; // (2,2) yaw 角速度

  // ---- 3. 线性加速度 linear_acceleration（m/s²，注意：包含重力分量）----
  // 数据来源：Get_Sensor_Data_New() 里在Mpu6050_Data中保存的IMU加速度计的 ADC 原始值 
  // ---- 加速度：加速度计原始ADC值 → 物理单位 m/s² ----
  // 与STM32初始化IMU时的量程有关，这里量程 ±2g = 19.6m/s²
  // ACCEl_RATIO = 32768/2g = 1671.84（把±32768映射到±19.6m/s²）
  // 注意：静止时 Z 轴不是 0，而是 ≈ +9.8（重力加速度），这是正常现象
  Imu_Data_Pub.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO; // X轴加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO; // Y轴加速度
  Imu_Data_Pub.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO; // Z轴加速度(含重力)
  // 注意：这里没有填 linear_acceleration_covariance，会保持默认全 0，全 0 在 ROS 惯例中通常表示"未知/不用该数据"

  // ---- 4. 姿态 orientation（四元数 xyzw）----
  // 根据IMU的三轴角速度和三轴加速度采用Mahony互补滤波（AHRS）算法解算出的四元数姿态
  // 四元数 (x,y,z,w) 用来表达三维旋转，比欧拉角更稳定、无万向锁问题
  // 供后面的 Publish_ImuSensor() 发布到 /imu/data_raw
  Quaternion_Solution(Imu_Data_Pub);    // 姿态四元数
  // 姿态协方差矩阵（3x3，按行主序只填对角线）：
  //   值越小 = 越信任该数据。这里 x/y 姿态用 1e6（很大=不信任）， z(偏航) 用 1e-6（很小=很信任）。
  //   原因：机器人主要在平面上运动，roll/pitch(倾斜)不可靠，yaw(朝向)靠陀螺仪很准
  Imu_Data_Pub.orientation_covariance[0] = 1e6;  // 元素(0,0)：roll 方向协方差 → 大=不可信
  Imu_Data_Pub.orientation_covariance[4] = 1e6;  // 元素(1,1)：pitch 方向协方差 → 大=不可信
  Imu_Data_Pub.orientation_covariance[8] = 1e-6; // 元素(2,2)：yaw 方向协方差 → 小=很可信

  // ---- 5. 发布到话题 /imu/data_raw ----
  imu_publisher->publish(Imu_Data_Pub);
}

/**************************************
Date: January 28, 2021
功能: 发布里程计话题 /odom
     包含：位置、姿态四元数、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵

【作用】把 Control() 里积分得到的"位置(Robot_Pos) + 速度(Robot_Vel)"
      打包成 ROS 标准消息 nav_msgs::msg::Odometry 发布到 /odom，
      让上层（EKF、导航、RViz）知道"小车现在在哪、以什么速度在动"。
【调用时机】Control() 主循环里，每次成功收到一帧底盘数据后调用。
【消息组成】nav_msgs/Odometry 包含：
      header(时间戳+坐标系) + pose(xyz位姿+协方差) + twist(xyz线速度+协方差) + child_frame_id
***************************************/
void turn_on_robot::Publish_Odom()
{
    // ---- 1. 实例化里程计消息 ----
    nav_msgs::msg::Odometry odom;

    // ---- 2. 头部：时间戳 + 父坐标系(自身坐标系) ----
    odom.header.stamp = rclcpp::Node::now();          // 当前时间
    // 父坐标系（"odom_combined"，表示该odom数据在EKF融合的里程计系odom_conbined，和EKF的坐标系同名不需要TF转换可以直接使用）
    odom.header.frame_id = ekf_odom_frame_id;             

    // ---- 3. 位姿 pose（机器人相对 odom 系的位置和姿态角）----
    // ① 里程计误差修正（用 wheeltec_param.yaml 里配置的系数）
    // 为什么需要修正？编码器测速存在系统误差（轮径不准、打滑、机械磨损），导致算出来的里程和真实距离有偏差。
    // 修正方法：把测到的速度乘一个"修正系数"（默认 1.0 = 不修正），这个修正系数需要进行调试分析
    Robot_Vel.X = Robot_Vel.X*odom_x_scale;   // X方向速度修正系数
    Robot_Vel.Y = Robot_Vel.Y*odom_y_scale;   // Y方向速度修正系数
    // Z 角速度分正反转两套系数：正转/反转的机械间隙不同，误差也不同
    if( Robot_Vel.Z>=0 )                      // 正转（逆时针）
      Robot_Vel.Z = Robot_Vel.Z*odom_z_scale_positive;
    else                                      // 反转（顺时针）
      Robot_Vel.Z = Robot_Vel.Z*odom_z_scale_negative;

    // ② 里程计位置积分（把速度对时间积分，得到位移）
    // 核心公式（二维旋转矩阵，把"车体坐标系速度"投影到"世界坐标系"）：差速小车的运动学正解计算方式
    //   [ X' ]   [ cosθ  -sinθ ] [ Vx ]
    //   [ Y' ] = [ sinθ   cosθ ] [ Vy ]
    // 其中 θ = Robot_Pos.Z（机器人当前偏航角yaw，即Z轴的偏航角度）
    // 直观理解：
    //   · Vx 是车头方向速度（前/后）Vy 是车身侧向速度（左/右，麦轮/全向轮才有）
    //   · 车如果转了 θ 角，那么速度对世界X轴/Y轴的贡献要按角度投影，再乘上时间 dt，就是这一小段时间里走过的距离
    // X 方向位移增量 = (Vx·cosθ - Vy·sinθ) · dt
    Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
    // Y 方向位移增量 = (Vx·sinθ + Vy·cosθ) · dt
    Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
    // Z 方向转角增量：角速度 × 时间（机器人绕Z轴转了多少弧度）
    Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

    // 保存本次位移积分后的xyz坐标位置
    odom.pose.pose.position.x = Robot_Pos.X;          // X 方向位置 (m)
    odom.pose.pose.position.y = Robot_Pos.Y;          // Y 方向位置 (m)
    // 这里表示Z方向上高度，小车贴地里程计里的小车高度为0，Z方向上的姿态角由里程计的四元数进行表示
    odom.pose.pose.position.z = 0;      

    // Robot_Pos.Z 是机器人当前朝向角（弧度，绕Z轴），里程计消息里的姿态必须用四元数表达（不能用欧拉角）
    // 把 Z 轴转角（yaw）转换成四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, Robot_Pos.Z);            // 只绕Z轴转 Robot_Pos.Z，roll/pitch 设为 0
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);  // 转成 ROS 消息类型
    odom.pose.pose.orientation = odom_quat;           // 姿态：上面转换出的四元数（绕Z轴的朝向角）

    // ---- 4. 子坐标系 child_frame_id ----
    // 含义：odom → base_footprint 的 TF 由此消息发布进行动态坐标变换
    odom.child_frame_id = robot_frame_id;             // 子坐标系（默认 "base_footprint"，机器人本体系）

    // ---- 5. 线速度和角速度 twist（机器人当前速度）----
    odom.twist.twist.linear.x =  Robot_Vel.X;         // X 方向线速度 (m/s)
    odom.twist.twist.linear.y =  Robot_Vel.Y;         // Y 方向线速度 (m/s)（麦轮/全向轮才有）
    odom.twist.twist.angular.z = Robot_Vel.Z;         // 绕Z轴角速度 (rad/s)

    // ---- 6. 协方差矩阵：静止/运动用两套（这是给 EKF 的"信任度"提示）----
    // 协方差矩阵 = 6x6，表示这个数据的"可信程度"，值越小越可信。
    // 官方 robot_localization 提供了两套参考矩阵（头文件里定义）：
    //   · odom_pose_covariance2 / odom_twist_covariance2 → 静止时用（更小=更可信）
    //   · odom_pose_covariance    / odom_twist_covariance    → 运动时用
    if(Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0)
    {
      // 静止时：编码器没在转，误差很小 → 认为里程计数据非常可靠 → 用"更小"的协方差
      //（odom_pose_covariance2 的对角线是 1e-9 / 1e-3 等，比运动版小）
      memcpy(&odom.pose.covariance,  odom_pose_covariance_stop,  sizeof(odom_pose_covariance_stop));
      memcpy(&odom.twist.covariance, odom_twist_covariance_stop, sizeof(odom_twist_covariance_stop));
    }
    else
    {
      // 运动时：编码器可能有打滑/滑动误差 → 认为 IMU 更可靠 → 用"较大"的协方差
      //（让 EKF 对里程计稍微"降权"，多参考 IMU）
      memcpy(&odom.pose.covariance,  odom_pose_covariance_run,  sizeof(odom_pose_covariance_run));
      memcpy(&odom.twist.covariance, odom_twist_covariance_run, sizeof(odom_twist_covariance_run));
    }

    // ---- 7. 发布 odom_raw → base_footprint 的动态 TF ----
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = rclcpp::Node::now();
    tf_msg.header.frame_id = raw_odom_frame_id;          // "odom_raw"
    tf_msg.child_frame_id  = robot_frame_id;         // "base_footprint"
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = odom.pose.pose.position.z;
    tf_msg.transform.rotation = odom.pose.pose.orientation;  // 四元数
    tf_broadcaster_->sendTransform(tf_msg);          // ← 通过指针广播到 /tf（注意是 -> 不是 .）

    // ---- 8. 发布到话题 /odom ----
    odom_publisher->publish(odom);
}

/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电源电压话题 PowerVoltage

【作用】把 Get_Sensor_Data_New() 解析出的电池电压 Power_voltage 发布出去。
【特点】⚠️ 本函数做了【降频处理】：不是每次都发，而是每 11 次调用才发 1 次。
       原因：电压变化很慢，没必要高频发布，降频可以减少话题流量/CPU占用。
       注意：即使"这次不发"，Count_Voltage_Pub 也一直在累加。
***************************************/
void turn_on_robot::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs;    // 定义电压消息（Float32 = 单个浮点数）

    // static 局部变量：跨多次调用保持值，不会因函数返回而清零
    // 作用：发布次数计数器（0~11 循环）
    static float Count_Voltage_Pub = 0;

    // 判断：Count_Voltage_Pub 先自增 1，再判断是否 >10
    // 也就是：第 1~10 次调用都不发，第 11 次调用才发，然后清零重新数
    if(Count_Voltage_Pub++ > 10)
    {
      Count_Voltage_Pub = 0;                        // 计数器清零，重新计数
      voltage_msgs.data = Power_voltage;            // 填入电压值（单位 V，来自下位机）
      voltage_publisher->publish(voltage_msgs);     // 发布到话题 PowerVoltage
    }
}

////////// 回充发布与回调 ////////
/**************************************
Date: January 17, 2022
Function: Pub the topic whether the robot finds the infrared signal (charging station)
功能: 发布机器人是否寻找到红外信号(充电桩)的话题
***************************************/
void turn_on_robot::Publish_RED()
{
  // 发布机器人是否找到充电红外信号
  std_msgs::msg::UInt8 msg;
  msg.data=Red;
  RED_publisher->publish(msg); 

}
/**************************************
Date: January 14, 2022
Function: Publish a topic about whether the robot is charging
功能: 发布机器人是否在充电的话题
***************************************/
void turn_on_robot::Publish_Charging()
{
  // 发布接收到回充帧时进行发布充电话题消息
  std_msgs::msg::Bool msg;
  msg.data=Charging;
  Charging_publisher->publish(msg); 

  // 保存上次机器人充电状态
  static bool last_charging;
  // 判断本次是进行充电，还是停止充电
  if(last_charging==false && Charging==true)cout<<GREEN<<"Robot is charging."<<endl<<RESET;
  if(last_charging==true && Charging==false)cout<<RED<<"Robot charging has disconnected."<<endl<<RESET;
  last_charging=Charging;

  //发布底层回充的设置状态的充电话题
  msg.data=charge_set_state;
  ChargingMode_publisher->publish(msg);
}
/**************************************
Date: January 28, 2021
Function: Publish charging current information
功能: 发布充电电流信息
***************************************/
void turn_on_robot::Publish_ChargingCurrent()
{
  // 发布机器人当前充电电流话题消息
  std_msgs::msg::Float32 msg; 
  msg.data=Charging_Current;
  Charging_current_publisher->publish(msg);
}

// 功能：小车底盘超声波避障是否开启提示的订阅函数
void turn_on_robot::RangerAvoidFlag_Callback(const std_msgs::msg::Bool::SharedPtr AvoidFlag)
{
  //设置避障标志位
  ranger_avoid_flag = AvoidFlag->data;
  if(ranger_avoid_flag) 
    RCLCPP_INFO(get_logger(),"ranger avoid turn on!");
  else 
    RCLCPP_INFO(get_logger(),"ranger avoid close!");
}

/**************************************
Date: 
Function: 
功能: 发布超声波测量距离相关信息
***************************************/
void turn_on_robot::Publish_distance()
{
    // 超声波传感器数据
    sensor_msgs::msg::Range range_msg;
    range_msg.header.stamp = this->get_clock()->now(); // 使用节点时钟获取时间戳
    range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND; // 数据类型为超声波
    range_msg.field_of_view = 0.52; // 视场角，约30°
    range_msg.min_range = 0.02; // 2cm 盲区
    range_msg.max_range = 5.3; // 5米最远距离

    // 发布 rangerA
    range_msg.header.frame_id = "ultrasonic_data_A";
    range_msg.range = distance.A;
    range_a_publisher_->publish(range_msg);

    // 发布 rangerB
    range_msg.header.frame_id = "ultrasonic_data_B";
    range_msg.range = distance.B;
    range_b_publisher_->publish(range_msg);

    // 发布 rangerC
    range_msg.header.frame_id = "ultrasonic_data_C";
    range_msg.range = distance.C;
    range_c_publisher_->publish(range_msg);

    // 发布 rangerD
    range_msg.header.frame_id = "ultrasonic_data_D";
    range_msg.range = distance.D;
    range_d_publisher_->publish(range_msg);

    // 发布 rangerE
    range_msg.header.frame_id = "ultrasonic_data_E";
    range_msg.range = distance.E;
    range_e_publisher_->publish(range_msg);

    // 发布 rangerF
    range_msg.header.frame_id = "ultrasonic_data_F";
    range_msg.range = distance.F;
    range_f_publisher_->publish(range_msg);
}

/**************************************
Date: January 14, 2022
Function: Subscription robot recharge flag bit topic, used to tell the lower machine speed command is normal command or recharge command
功能: 订阅机器人是否回充标志位话题，用于告诉下位机速度命令是正常命令还是回充命令
***************************************/
void turn_on_robot::Recharge_Flag_Callback(const std_msgs::msg::Int8::SharedPtr Recharge_Flag)
{
  //设置自动回充标志位
  AutoRecharge=Recharge_Flag->data;
  
  //新版本协议
  Send_Data.tx[0] = FRAME_HEADER; //frame head 0x7B //帧头0X7B
  Send_Data.tx[1] = 1;

  if( AutoRecharge!=0 ) Send_Data.tx[2] = 0xA1;
  else Send_Data.tx[2] = 0xA0;
  
  Send_Data.tx[3] = 0;Send_Data.tx[4] = 0;
  Send_Data.tx[5] = 0;Send_Data.tx[6] = 0;
  Send_Data.tx[7] = 0;Send_Data.tx[8] = 0;
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
  }
  catch (...)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }

  //旧版本协议
  if( AutoRecharge==0 )
  {
    Send_Data.tx[0] = FRAME_HEADER; //frame head 0x7B //帧头0X7B
    Send_Data.tx[1] = AutoRecharge; //set aside //预留位
    Send_Data.tx[2] = 0; //set aside //预留位
    Send_Data.tx[3] = 0;Send_Data.tx[4] = 0;
    Send_Data.tx[5] = 0;Send_Data.tx[6] = 0;
    Send_Data.tx[7] = 0;Send_Data.tx[8] = 0;
    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
    try
    {
      Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
    }
    catch (...)   
    {
      RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
    }
  }

}

// 底盘安全开启或关闭的标志位，并构造帧格式发送给下位机stm32
void turn_on_robot::Security_Callback(const std_msgs::msg::Int8 &Security_Flag)
{
  // 默认其他话题发布的值大于0，就是开启底盘安全，否则关闭
  SecurityPLY = Security_Flag.data;
  
  // 采用新旧两版本分别发送1个帧给下位机表示开启或关闭底盘安全
  //新版本协议
  Send_Data.tx[0] = FRAME_HEADER; // 帧头0X7B
  Send_Data.tx[1] = 0;            // 预留位
  if( SecurityPLY!=0 ) Send_Data.tx[2] = 0xB1;// 安全开指令0xB1
  else Send_Data.tx[2] = 0xB0;                // 安全关指令0xB0
  // xyz线速度数据区（此命令不需要数据，置0）
  Send_Data.tx[3] = 0;Send_Data.tx[4] = 0;
  Send_Data.tx[5] = 0;Send_Data.tx[6] = 0;
  Send_Data.tx[7] = 0;Send_Data.tx[8] = 0;
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); // BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; // 帧尾0X7D
  try
  {
    // 通过串口向下位机发送数据 
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));
  }
  catch (...)   
  {
    // 如果发送数据失败，打印错误信息
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); 
  }

  //旧版本协议
  Send_Data.tx[0] = FRAME_HEADER; // 帧头0X7B
  Send_Data.tx[1] = 0;            // 预留位
  Send_Data.tx[2] = SecurityPLY;  // SecurityPLY值用来表示安全开/关指令
  // xyz线速度数据区（此命令不需要数据，置0）
  Send_Data.tx[3] = 0;Send_Data.tx[4] = 0;
  Send_Data.tx[5] = 0;Send_Data.tx[6] = 0;
  Send_Data.tx[7] = 0;Send_Data.tx[8] = 0;
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); // BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; // 帧尾0X7D
  try
  {
    // 通过串口向下位机发送数据 
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx));  
  }
  catch (...)   
  {
    // 如果发送数据失败，打印错误信息
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); 
  }
}


// //服务
// void turn_on_robot::Set_Charge_Callback(const shared_ptr<turtlesim::srv::Spawn::Request> req,shared_ptr<turtlesim::srv::Spawn::Response> res)
// {
//     //以服务方式开启回充
//     auto msg = std::make_shared<std_msgs::msg::Int8>();
//     msg->data = round(req->x);
//     Recharge_Flag_Callback(msg);

//     if( msg->data==0 )
//     {
//       if(charge_set_state==0)
//         AutoRecharge=0,res->name = "true";
//       else
//         res->name = "false";
//     }
//     else
//     {
//       if(charge_set_state==1)
//         res->name = "true";
//       else
//         res->name = "false";
//     }
//     return;
// }
////////// 回充发布与回调 ////////


// 基础底盘运动帧的上行帧校验函数
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
     }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
     }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}
//自动回充上行帧的校验函数
unsigned char turn_on_robot::Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_AutoCharge_Data.rx[k]; //By bit or by bit //按位异或
    }
  }

  return check_sum;
}
//超声波上行帧的校验函数
uint8_t turn_on_robot::Check_Sum_Supersonic(const uint8_t* checkdata,uint16_t datalen)
{
	char bccval = 0;
	for(uint16_t i=0;i<datalen;i++)
	{
		bccval ^= checkdata[i];
	}
	return bccval;
}


