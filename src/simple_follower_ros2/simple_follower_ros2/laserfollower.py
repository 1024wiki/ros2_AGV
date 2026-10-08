#!/usr/bin/env python3
# ============================================================================
# 文件 : laserfollower.py（simple_follower_ros2 包）
# 功能 : 激光雷达目标跟随控制节点（laserfollower）
#
# 作用：订阅 lasertracker 检测到的目标位置（角度 + 距离）/object_tracker/current_position，
#       用 PID 控制器算出转向角速度 + 前进线速度，发布 /cmd_vel 让小车始终正对目标并保持设定距离（跟随目标移动）。
#
# 数据流：
#   lasertracker --/object_tracker/current_position(角度+距离)--> 本节点
#   本节点 --/cmd_vel(速度)--> 底盘驱动节点 --> STM32 --> 电机运动
#
# 说明：本文件是 wheeltec 从 OSRF 2016 年开源 obstacle_avoidance 包
#       移植到 ROS2 的，保留了手柄按键开关等旧代码（未使用可忽略）。
# ============================================================================

import rclpy
import _thread          # 低级线程库（旧代码遗留，实际未用到）
import threading        # 高级线程库（用于控制器连接丢失定时器）
import time             # 时间（PID 计算 deltaT 用）
import numpy as np      # 数组/矩阵运算（PID 是向量形式）
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan     # Joy 是手柄（旧代码遗留，未用到）
from geometry_msgs.msg import Twist, Vector3   # Twist = 速度消息（/cmd_vel 类型）
from turn_on_wheeltec_robot.msg import Position as PositionMsg  # 底盘包定义的目标位置消息
from std_msgs.msg import String as StringMsg
from rclpy.qos import QoSProfile
from example_interfaces.srv import AddTwoInts  # 旧代码遗留，未用到
from std_msgs.msg import Int8                 # 跟随标志消息类型

# 全局变量（旧代码遗留，实际未被类内使用）
angle=[0.0]*3    # 预留角度数组
distan=[0.0]*3   # 预留距离数组

class LaserFollower(Node):

	def __init__(self):
		super().__init__('laserfollower')

		# ---------- 连接丢失保护定时器（手柄相关，本版本未真正启用） ----------
		# 正常情况下手柄消息会不断重置这个 1 秒定时器；
		# 若超时未收到手柄消息 → 调用 controllerLoss() 停车（防失控保护）
		self.controllerLossTimer = threading.Timer(1, self.controllerLoss)
		self.controllerLossTimer.start()

		# ---------- PID 参数声明（可被外部 launch 参数覆盖） ----------
		# 每个都是双元素数组：[角速度通道, 线速度通道]
		self.declare_parameter('P',[1.6 ,0.5])       # 比例系数 Kp（角/线）
		self.declare_parameter('I',[0.00,0.00])      # 积分系数 Ki（默认 0，不用积分）
		self.declare_parameter('D',[0.03 ,0.005])    # 微分系数 Kd（角/线）
		self.declare_parameter('~targetDist',[0.0,0.6])  # 目标值：期望角度 0（正对目标）、期望距离 0.6m
		# 手柄开关模式（旧代码遗留：本版本无手柄输入，开关逻辑实际不生效）
		self.switchMode= True
		self.max_speed = self.declare_parameter('~maxSpeed',0.5)  # 最大速度限制（声明但代码里用硬编码0.4）
		#self.controllButtonIndex = self.declare_parameter('~controllButtonIndex').value
		self.controllButtonIndex = -4    # 手柄开关按钮索引（PS3 手柄）
		self.buttonCallbackBusy=False    # 按钮防抖标记
		self.active=False                # 跟随是否激活（旧代码：需手柄按键开启）
		self.i=0                         # 计数：只发布一次激活标志

		qos = QoSProfile(depth=10)  # QoS：队列深度 10
		# ---------- 发布者 ----------
		# /laser_follow_flag：跟随激活标志（Int8=1 表示开始跟随，供上层/用户查看）
		self.laserfwflagPublisher = self.create_publisher(Int8,'/laser_follow_flag',qos)
		# /cmd_vel：速度指令 → 底盘驱动节点订阅执行（核心输出！）
		self.cmdVelPublisher = self.create_publisher(Twist, 'cmd_vel', qos)

		# ---------- 订阅者 ----------
		# /object_tracker/current_position：订阅 lasertracker 发布的目标位置（角度+距离）
		self.positionSubscriber = self.create_subscription(
		    PositionMsg,
		    '/object_tracker/current_position',
		    self.positionUpdateCallback,   # 每收到一个目标位置 → 计算速度并发布
		    qos)
		# /object_tracker/info：订阅 lasertracker 的提示信息（如"没找到目标"），只打日志
		self.trackerInfoSubscriber = self.create_subscription(
		    StringMsg,
		    '/object_tracker/info',
		    self.trackerInfoCallback,
		    qos)

		# ---------- 初始化 PID 控制器 ----------
		targetDist = self.get_parameter('~targetDist').value  # 目标 [0角度, 0.6m距离]
		P = self.get_parameter('P').value    # [1.6, 0.5]
		I = self.get_parameter('I').value    # [0.0, 0.0]
		D = self.get_parameter('D').value    # [0.03, 0.005]
		self.get_logger().info('simplepid_parameter: target:{} P:{} I:{} D:{}'.format(targetDist,P,I,D))
		# 创建双通道 PID（向量 PID）：第0通道管角度、第1通道管距离
		self.PID_controller = simplePID(targetDist,P,I,D)

	# 收到 lasertracker 的提示信息时的回调（目前只打日志）
	def trackerInfoCallback(self, info):
		self.get_logger().warn(info.data)   # 比如"没找到目标" → 打印警告

	# 发布"跟随激活"标志（只调用一次，通知外部激光跟随已开始）
	def publish_flag(self):
		laser_follow_flag=Int8()
		laser_follow_flag.data=1
		self.laserfwflagPublisher.publish(laser_follow_flag)
		self.get_logger().info('laser_follow_flag: {}'.format(laser_follow_flag))

	# ========================================================================
	# positionUpdateCallback：核心跟随控制回调
	# 触发时机：每收到一帧 lasertracker 的目标位置（角度+距离）
	# 跟随控制思想：
	#   目标有两个量需要控制：
	#     ① 角度误差 → 控制转向（角速度 angular.z），让小车正对目标
	#     ② 距离误差 → 控制前进/后退（线速度 linear.x），保持 0.6~0.8m 距离
	#   用"双通道 PID"算出两个速度，再限幅，最后发 /cmd_vel
	# ========================================================================
	def positionUpdateCallback(self, position):
		# position 里的两个量：angle_x=目标相对正前方的角度(弧度)、distance=目标距离(米)
		angle_x= position.angle_x
		distance = position.distance
		# ---- 角度归一化：把角度转换到 [-π, π] 范围 ----
		# lasertracker 的角度范围是 [-π, π]（负=右侧，正=左侧）
		# 而 PID 期望角度是 0（正对前方），这里做了一个 π 的翻转，
		# 使"目标在右(负角)"→ 误差为负 → PID 输出正角速度向右转（配合下方取负号）
		if(angle_x>0):
			angle_x=angle_x-3.1415   # 大于0（左侧）→ 减 π，变成接近 -0 的负值
		else :
			angle_x=angle_x+3.1415   # 小于0（右侧）→ 加 π

		# ---- 调用双通道 PID，得到 [未限幅的角速度, 未限幅的线速度] ----
		[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angle_x, distance])
		# ---- 速度限幅到 ±0.4（防止过冲/速度突变） ----
		angularSpeed = np.clip(-uncliped_ang_speed, -0.4, 0.4)  # 取负 = 方向修正（见上方角度翻转）
		linearSpeed  = np.clip(-uncliped_lin_speed, -0.4, 0.4)

		# ---- 构造速度消息 ----
		velocity = Twist()	
		velocity.linear.y = 0.0    # 轮式底盘不能横移
		velocity.linear.z = 0.0
		velocity.angular.x = 0.0   # 无侧倾/俯仰旋转
		velocity.angular.y = 0.0
		# ---- 安全判断：满足条件才动，否则停车（防撞/防误动） ----
		angle_error=abs(abs(angle_x)-3.14)   # 真正"目标在正前方偏了多少"（还原翻转后的误差）
		linear_error=abs(distance-0.8)       # 距目标的距离误差（目标理想 0.8m，注意和 PID 的 0.6 不一致）
		if distance>0.3 and angle_error > 0.05 and linear_error > 0.05:
			# ① 目标距离 > 0.3m（太近就停，防撞）
			# ② 角度误差 > 0.05（还没正对目标）
			# ③ 距离误差 > 0.05（还没到目标距离）
			# 全部满足 → 正常跟随：前进 + 转向
			velocity.linear.x = linearSpeed
			velocity.angular.z = -angularSpeed
		else:
			# 条件不满足（目标太近 / 已正对 / 已到距离）→ 停车
			velocity.linear.x = 0.0
			velocity.angular.z = 0.0

		# ---- 发布速度到 /cmd_vel → 底盘驱动执行 ----
		self.cmdVelPublisher.publish(velocity)
		# 只发布一次"跟随激活"标志（第一次回调时）
		if self.i < 1:
			self.i = self.i +1
			self.publish_flag()

	# ========================================================================
	# 以下 buttonCallback/threadedButtonCallback 是"手柄按键开关"逻辑
	# （旧代码遗留：本文件未订阅 /joy，因此不会真正触发，可忽略学习）
	# 设计目的：按下手柄某键 → 激活/停止跟随（防误启动的开关）
	# ========================================================================
	def buttonCallback(self, joy_data):
		# 收到手柄消息时调用（若订阅了 /joy 才会触发）
		# 1) 重置"连接丢失"定时器：有手柄消息 = 连接正常
		self.controllerLossTimer.cancel()
		self.controllerLossTimer = threading.Timer(0.5, self.controllerLoss)
		self.controllerLossTimer.start()

		# 2) 防抖：手柄消息发布很快，一次按键会收到约10条重复消息，
		#    只处理第一条，丢弃其余（用 buttonCallbackBusy 标记）
		if self.buttonCallbackBusy:
			return 
		else:
			# 用独立线程处理按键，避免阻塞主回调
			thread.start_new_thread(self.threadedButtonCallback,  (joy_data, ))

	# 在线程里真正处理一次按键
	def threadedButtonCallback(self, joy_data):
		self.buttonCallbackBusy = True   # 置忙，丢弃重复消息

		# 情况 A：当前激活 && 按键被按下 → 停止跟随
		if(joy_data.buttons[self.controllButtonIndex]==self.switchMode and self.active):
			self.get_logger().info('stoping')
			self.stopMoving()    # 发全零速度停车
			self.active = False  # 标记为未激活
			time.sleep(0.5)      # 延时防连续触发
		# 情况 B：当前未激活 && 按键按下 → 激活跟随
		elif(joy_data.buttons[self.controllButtonIndex]==True and not(self.active)):
			self.get_logger().info('activating')
			self.active = True #enable response
			time.sleep(0.5)

		self.buttonCallbackBusy = False  # 解锁，允许处理下一次按键

	# 停车：发布全零速度
	def stopMoving(self):
		velocity = Twist()
		velocity.linear.x = 0.0
		velocity.linear.y = 0.0
		velocity.linear.z = 0.0

		velocity.angular.x = 0.0
		velocity.angular.y = 0.0
		velocity.angular.z = 0.0
		self.cmdVelPublisher.publish(velocity)

	# 连接丢失保护：超过定时器时间没收到手柄消息 → 停车并失活
	def controllerLoss(self):
		self.stopMoving()      # 立即停车（防止失控乱跑）
		self.active = False    # 标记未激活
		self.get_logger().info('lost connection')


# ============================================================================
# simplePID：简单的离散 PID 控制器（向量形式，可同时控制多个通道）
#
# PID 公式（离散化）：
#   u(t) = Kp*error + Ki*∫error·dt + Kd*(d error/dt)
#   输出 = 比例项 + 积分项 + 微分项
#
# 本项目中两个通道：
#   [0] = 角度通道：让角度误差 → 0（正对目标）
#   [1] = 距离通道：让距离误差 → 0（保持目标距离）
# ============================================================================
class simplePID:
	'''very simple discrete PID controller'''
	def __init__(self, target, P, I, D):
		# 注：下面三行把传入参数覆盖成固定值，是个 bug/遗留
		#（意味着 launch 里的 P/I/D 参数实际不生效，PID 增益恒为下面固定值）
		P = [1.5, 0.5]    # Kp：角度通道1.5、距离通道0.5
		I = [0, 0]        # Ki：不用积分
		D = [0.0,0.0]     # Kd：固定为0（微分也未生效！）
		node = rclpy.create_node('simplepid')
		# 参数形状校验：P/I/D 长度一致，target 维度与 P 兼容
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')
		self.Kp		=np.array(P)   # 比例增益数组
		self.Ki		=np.array(I)   # 积分增益数组
		self.Kd		=np.array(D)   # 微分增益数组
		self.setPoint   =np.array(target)  # 目标值数组 [期望角度, 期望距离]
		
		self.last_error=0              # 上一次误差（算微分用）
		self.integrator = 0            # 误差积分累积值
		self.integrator_max = float('inf')  # 积分限幅（无穷=不限）
		self.timeOfLastCall = None     # 上次调用时间（算 deltaT 用）

	def update(self, current_value):
		# 输入当前测量值 [角度, 距离]，输出 PID 控制量 [角速度, 线速度]
		current_value=np.array(current_value)
		# 形状校验：测量值和目标维度必须一致
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			# 第一次调用：不知道 deltaT，不输出控制（返回0），只记录时间
			self.timeOfLastCall = time.perf_counter()
			return np.zeros(np.size(current_value))

		# ---- 计算误差：目标 - 当前 ----
		error = self.setPoint - current_value

		# （被注释掉的死区处理：误差<0.05 时置0）

		# ---- 距离误差放大（人性化处理） ----
		# 当目标较近且误差为正（需要接近）时，放大误差，让小车在远处快、近处慢，接近更平滑
		if (error[1]>0 and self.setPoint[1]<1.2):
			error[1]=error[1]*(1.2/self.setPoint[1])*0.7

		# ---- 三个分量 ----
		P =  error   # 比例项 = 当前误差
		
		currentTime = time.perf_counter()
		deltaT      = (currentTime-self.timeOfLastCall)  # 本次与上次的时间间隔
		# 积分项 = 误差 × 时间累积
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		# 微分项 = 误差变化率（(本次误差-上次误差)/dt）
		D = (error-self.last_error)/deltaT
		
		self.last_error = error        # 保存本次误差供下次微分
		self.timeOfLastCall = currentTime
		
		# ---- 合成 PID 输出 ----
		return self.Kp*P + self.Ki*I + self.Kd*D

# main 入口：创建节点并阻塞运行
def main(args=None):
    print('starting')
    rclpy.init(args=args)
    laserfollower = LaserFollower()   # 创建节点（构造函数完成订阅/发布/PID初始化）
    try:
        rclpy.spin(laserfollower)     # 阻塞运行：持续收目标位置 → 算速度 → 发 /cmd_vel
    finally:
        laserfollower.destroy_node()  # 退出时销毁节点
        rclpy.shutdown()

if __name__ == '__main__':
    main()

