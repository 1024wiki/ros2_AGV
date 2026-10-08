#!/usr/bin/env python3
# ============================================================================
# 文件 : visualTracker.py（simple_follower_ros2 包）
# 功能 : 视觉目标检测节点（visualtracker）——用相机图像跟踪目标
#
# 作用：订阅 RGB 彩色图 + 深度图（时间同步），通过 HSV 颜色阈值分割出目标，
#       计算目标在画面中的中心坐标 + 距离，发布目标相对小车的角度和距离，
#       供 visualFollower（视觉跟随控制节点）使用。
#
# 与激光跟随的区别：激光找"最近点"，视觉找"指定颜色的目标"（如红色球）。
# 数据流：
#   相机 --/camera/color/image_raw + /camera/depth/image_raw--> 本节点
#   本节点 --/object_tracker/current_position(角度+距离)--> visualFollower
#
# 说明：本文件同样源自 OSRF 2016 年开源 obstacle_avoidance 包，wheeltec 移植到 ROS2。
# ============================================================================

from __future__ import division
import message_filters            # ROS 消息同步库（RGB 图和深度图时间对齐用）
import numpy as np                # 数组运算
import cv2                        # OpenCV 图像处理（HSV 分割、轮廓检测）
import rclpy
from rclpy.node import Node
from matplotlib import pyplot as plt   # 调试时显示图像用（displayImage=True 才用）
import cv_bridge                  # ROS Image 消息 <-> OpenCV 图像 转换

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from turn_on_wheeltec_robot.msg import Position as PositionMsg  # 目标位置消息（角度+距离）
from std_msgs.msg import String as StringMsg

np.seterr(all='raise')    # numpy 运算遇到警告/错误直接抛异常（调试用）
displayImage=False        # 调试开关：True 时会弹窗显示 HSV/掩码图像
plt.close('all')          # 关闭所有 matplotlib 图窗

class VisualTracker(Node):
	def __init__(self):
		super().__init__('visualtracker')
		qos = QoSProfile(depth=10)
		self.bridge = cv_bridge.CvBridge()   # 消息转换桥：Image <-> numpy 数组

		# ---------- 目标颜色 HSV 阈值（默认检测"红色"，可被 launch 参数覆盖） ----------
		# HSV 色彩空间：H=色调(0~180)、S=饱和度、V=亮度（OpenCV 里 H 上限 180）
		# targetUpper/targetLower 定义目标颜色范围（这里是红色在 HSV 中的典型范围）
		self.targetUpper = np.array([0, 50, 50])       # H 下限 0
		self.targetLower = np.array([180, 255, 255])   # H 上限 180（红色横跨 0 和 180 两端）

		# ---------- 相机参数（画面尺寸 + 视场角） ----------
		# 这些是相机固有参数（本版本硬编码，未从 launch 读取）
		self.pictureHeight= 480      # 图像高度（像素）
		self.pictureWidth = 640      # 图像宽度（像素）
		vertAngle =0.43196898986859655     # 相机垂直视场角的一半（弧度）
		horizontalAngle =  0.5235987755982988  # 相机水平视场角的一半（弧度）

		# 预计算正切值（后面角度换算只用得到 tan，提前算好省时间）
		self.tanVertical = np.tan(vertAngle)
		self.tanHorizontal = np.tan(horizontalAngle)	
		self.lastPoCsition =None   # 上一次目标位置（用于判断位置是否合理）

		# ---------- 订阅 RGB 图 + 深度图，并用时间同步器对齐 ----------
		# 为什么用 message_filters？
		#   目标位置 = 颜色图里的坐标 + 深度图里的距离，两者必须"同一时刻"才有意义。
		#   但 RGB 相机和深度相机发布频率/时刻不同，需要把两路消息按时间戳配对。
		im_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')   # 彩色图
		dep_sub = message_filters.Subscriber(self,Image,'/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)  # 深度图

		# ApproximateTimeSynchronizer：近似时间同步器，
		# 把时间戳最接近的一帧 RGB + 一帧深度图打包，一起交给 trackObject 回调（容差 0.05s）
		queue_size = 30   # 同步队列长度
		self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub],queue_size,0.05)
		self.timeSynchronizer.registerCallback(self.trackObject)

		# ---------- 发布者 ----------
		# /object_tracker/current_position：目标位置（角度+距离）→ 给 visualFollower
		self.positionPublisher = self.create_publisher( PositionMsg,'/object_tracker/current_position', qos)
		self.posMsg=PositionMsg()   # 复用的消息对象，避免每次分配

	
	# ========================================================================
	# trackObject：核心回调（每收到一帧时间对齐的 RGB+深度图调用一次）
	# 流程：RGB 图 → HSV 阈值分割出目标 → 腐蚀去噪 → 找轮廓 → 取最大轮廓
	#       → 计算中心坐标 + 平均距离 → 发布角度和距离
	# ========================================================================
	def trackObject(self, image_data, depth_data):
		# ---- 1. 检查并转换图像格式 ----
		if(image_data.encoding != 'rgb8'):
			raise ValueError('image is not rgb8 as expected')
		# 把 ROS Image 消息转成 numpy 数组（OpenCV 处理格式）
		frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='rgb8')           # RGB 彩色图
		depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#"32FC1"）  # 深度图（保留原始编码）
		# 校验图像尺寸是否符合预期（防止相机分辨率变了导致后续索引出错）
		if(np.shape(frame)[0:2] != (self.pictureHeight, self.pictureWidth)):
			raise ValueError('image does not have the right shape. shape(frame): {}, shape parameters:{}'.format(np.shape(frame)[0:2], (self.pictureHeight, self.pictureWidth)))
		# （可选：先高斯模糊降噪，当前被注释掉）
		#blurred = cv2.GaussianBlur(frame, (11,11), 0)
		# ---- 2. 转 HSV 并做颜色阈值分割 ----
		hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)	# RGB → HSV（颜色在 HSV 下对光照更鲁棒）
		# 选中颜色范围内的所有像素：范围内的置 255（白），范围外置 0（黑）→ 得到二值掩码
		org_mask = cv2.inRange(hsv, self.targetUpper, self.targetLower)	

		# ---- 3. 形态学腐蚀：去除小噪声点 ----
		mask = cv2.erode(org_mask, None, iterations=4)	# 腐蚀 4 次，消除孤立噪点
		#（可选：再膨胀恢复目标大小，当前被注释）
		#mask = cv2.dilate(mask, None, iterations=3)
		# ---- 4. 找轮廓 ----
		# 只取最外层轮廓（RETR_EXTERNAL），返回轮廓列表（[-2] 兼容旧版 OpenCV 索引）
		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

		# ---- 5. 调试显示（displayImage=True 时弹窗查看） ----
		if displayImage:
			backConverted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
			#cv2.imshow('frame', backConverted)
			#cv2.waitKey(0)
			#print(backConverted)			
			plt.figure()
			plt.subplot(2,2,1)
			plt.imshow(frame)
			plt.xticks([]),plt.yticks([])
			plt.subplot(2,2,2)
			plt.imshow(org_mask, cmap='gray', interpolation = 'bicubic')			
			plt.xticks([]),plt.yticks([])
			plt.subplot(2,2,3)			
			plt.imshow(mask, cmap='gray', interpolation = 'bicubic')
			plt.xticks([]),plt.yticks([])
			plt.show()
			rclpy.sleep(0.2)

		# ---- 6. 遍历轮廓，从大到小，取最大的一个作为目标 ----
		newPos = None # 本次相机检测到的红色物体的中心位置
		for contour in sorted(contours, key=cv2.contourArea, reverse=True):
			# 按轮廓面积从大到小排序（面积大 → 更可能是目标，不是噪声）

			# 计算这个轮廓的目标位置（中心坐标 + 平均距离），contour轮廓为(N,1,2)形状的ndarray数组
			pos = self.analyseContour(contour, depthFrame)

			# 记录第一个（最大的）轮廓位置，作为"没找到合适目标"时的兜底值
			if newPos is None:
				newPos = pos
			# 检查位置是否合理（当前被注释，直接信任最大轮廓）
			if self.checkPosPlausible(pos):
				self.lastPosition = pos       # 保存本次目标位置
				self.publishPosition(pos)     # 发布角度和距离
				return   # 只处理最大的轮廓，处理完直接返回
		
		# 没找到任何轮廓 → 保存 None 作为兜底（下次可判断目标丢失）
		self.lastPosition = newPos #we didn't find a plossible last position, so we just save the biggest contour 
		# and publish warnings
		#self.get_logger().warn('no position found')
		#self.infoPublisher.publish(StringMsg('visual:nothing found'))
		
	# ========================================================================
	# publishPosition：把目标的像素中心坐标+距离换算成"角度+距离"并发布
	# ========================================================================
	def publishPosition(self, pos):
		# 计算目标相对正前方的水平角度和垂直角度
		self.posMsg.angle_x = self.calculateAngleX(pos)   # 水平角（左右偏）
		print(self.posMsg.angle_x)
		self.posMsg.angle_y = self.calculateAngleY(pos)   # 垂直角（上下偏）
		self.posMsg.distance=float(pos[1])                # 目标距离（米，来自深度图均值）
		# 发布到 /object_tracker/current_position（给 visualFollower 用）
		self.positionPublisher.publish(self.posMsg)

	# ========================================================================
	# checkPosPlausible：判断目标位置是否"合理"（当前被调用处注释，暂未使用）
	# 原理：和上一帧位置对比，距离/坐标跳变太大就认为是误检
	# ========================================================================
	def checkPosPlausible(self, pos):
		'''Checks if a position is plausible. i.e. close enough to the last one.'''

		# 第一帧没有历史可比较，直接认为不合理
		if self.lastPosition is None:
			return true

		# 解包位置：((中心X, 中心Y), 距离)
		((centerX, centerY), dist)=pos	
		((PcenterX, PcenterY), Pdist)=self.lastPosition
		
		if np.isnan(dist):      # 距离无效（NaN）→ 不合理
			return False

		if abs(dist-Pdist)>0.5: # 距离相对上一帧跳变超过 0.5m → 不合理
			return False

		if abs(centerX-PcenterX)>(self.pictureWidth /5):   # 中心 X 跳变超过画面宽度的 1/5 → 不合理
			return False

		if abs(centerY-PcenterY)>(self.pictureHeight/5):   # 中心 Y 跳变超过画面高度的 1/5 → 不合理
			return False
		
		return True   # 所有条件通过 → 合理
			
	# ========================================================================
	# calculateAngleX：把目标中心的像素 X 坐标换算成"水平偏移角"
	# 计算原理（小孔成像近似）：假设相机焦距为f（光心到图像平面的距离)，画面宽高为(W,H)，此时物体X轴和y轴中心点为(X,Y)
	# 	则X和Y轴最大角度(tanA,tanB) = (W/2, H/2)/f，物体相对于相机的X轴和Y轴角度分别为(tana,tanb)=（X-W/2, Y-H/2）/f（相对于中心点左负右正)
	#	则此时物品相当于机器人的X轴和Y轴的偏移角度为(a,b) = arctan(2*(X/W, Y/H)-1)
	# ========================================================================
	def calculateAngleX(self, pos):
		'''calculates the X angle of displacement from straight ahead'''
		centerX = pos[0][0]                              # 目标中心 X 像素坐标
		displacement = 2*centerX/self.pictureWidth-1     # 归一化到 [-1,1]（中心=0，右=正）
		angle = -1*np.arctan(displacement*self.tanHorizontal)  # 转成水平角度（弧度），取负使方向符合右手系
		return angle
	def calculateAngleY(self, pos):
		'''同上：垂直方向的角度换算'''
		centerY = pos[0][1]                              # 目标中心 Y 像素坐标
		displacement = 2*centerY/self.pictureHeight-1    # 归一化到 [-1,1]（中心=0，上=正）
		angle = -1*np.arctan(displacement*self.tanVertical)  # 垂直角度（弧度）
		return angle
	
	# ========================================================================
	# analyseContour：对给定轮廓计算"中心坐标 + 平均距离"
	# ========================================================================
	def analyseContour(self, contour, depthFrame):
		'''Calculates the centers coordinates and distance for a given contour

		Args:
			contour (opencv contour): contour of the object
			depthFrame (numpy array): the depth image
		
		Returns:
			centerX, centerY (doubles): center coordinates
			averageDistance : distance of the object
		'''
		# ---- 1. 求轮廓的最小外接矩形（含旋转角度） ----
		centerRaw, size, rotation = cv2.minAreaRect(contour)

		# ---- 2. 中心取整（才能作为图像索引） ----
		center = np.round(centerRaw).astype(int)

		# ---- 3. 确定"目标内部安全区域"的大小 ----
		# 取外接矩形短边的 1/3 作为半径，保证取样区域完全落在目标内部（避开边缘/背景）
		minSize = int(min(size)/3)

		# ---- 4. 取这个区域内的深度值 ----
		depthObject = depthFrame[(center[1]-minSize):(center[1]+minSize), (center[0]-minSize):(center[0]+minSize)]

		# ---- 5. 去掉无效点（NaN），取平均深度作为目标距离 ----
		depthArray = depthObject[~np.isnan(depthObject)]
		#averageDistance = np.mean(depthArray)
		
		if len(depthArray) == 0:
			self.get_logger().warn('empty depth array. all depth values are nan')  # 该区域深度全无效
			averageDistance=0
		else:
			averageDistance = np.mean(depthArray)   # 取平均（比单点更可靠）
			
		# ---- 6. 距离合理性钳制（注意：这段逻辑有 bug，实际等价于"超出范围就置400"）----
		# 原意可能是"距离应在 400~3000mm 之间"，但条件写反了：
		#   averageDistance>400 or averageDistance<3000 几乎恒真 → 什么都不做；
		#   否则（averageDistance<=400 且 >=3000，不可能）→ 置 400。
		if(averageDistance>400 or averageDistance<3000):
			pass
		else:
			averageDistance=400


		return (centerRaw, averageDistance) # 中心点+中心点附近的平均距离
	
# main 入口：创建节点并阻塞运行
def main(args=None):
    print('visual_tracker')
    rclpy.init(args=args)
    visualTracker = VisualTracker()   # 创建节点（构造函数完成订阅/发布/同步器初始化）
    
    print('visualTracker init done')
    try:
        rclpy.spin(visualTracker)     # 阻塞运行：持续收同步的 RGB+深度图 → 检测目标 → 发布位置
    finally:
        visualTracker.destroy_node()  # 退出时销毁节点
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    


