#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy, LaserScan
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from rclpy.qos import QoSProfile,QoSPresetProfiles
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String as StringMsg	


# ============================================================================
# LaserTracker：激光目标检测节点（lasertracker）
#
# 功能：在激光雷达 /scan 数据中找到"最近的真实目标"，输出该目标相对小车的
#       角度和距离/object_tracker/current_position，供上层跟随节点（laserfollower）使用。
#
# 核心思想：
#   单纯的"最近点"可能是噪声/地面/偶然障碍，所以本节点用了"两次扫描比对"的方法：
#   	只把【上一帧也有点、且距离几乎没变】的最近点当作"真实目标"，从而过滤掉闪烁的噪声点，让跟随更稳定。
#
# 数据流：
#   激光雷达 --/scan--> LaserTracker --/object_tracker/current_position(角度+距离)--> laserfollower
# ============================================================================


class LaserTracker(Node):

	def __init__(self):
		super().__init__('lasertracker')
		self.lastScan=None    # 缓存上一帧的激光距离数组（用于两帧比对找稳定目标）
		# 两帧激光雷达数据比较的参数声明
		self.winSize = self.declare_parameter('~winSize', 2)      # 参数：对比窗口半宽（保留，代码里硬编码用 i-2~i+2）
		self.deltaDist =self.declare_parameter('~deltaDist', 0.2) # 参数：两帧距离变化阈值（保留，代码里硬编码 0.2）
		# 发布：目标当前位置（角度 angle_x + 距离 distance）→ 给 laserfollower 消费
		self.positionPublisher = self.create_publisher(PositionMsg, 'object_tracker/current_position', 10)
		# 发布：找不到目标时的提示信息 → 给 laserfollower 打日志用
		self.infoPublisher = self.create_publisher(StringMsg, 'object_tracker/info', 10)
		# 订阅：激光扫描数据 /scan（用传感器 QoS：best_effort，保证实时不阻塞），激光订阅数据的回调函数registerScan
		self.scanSubscriber = self.create_subscription(
		    LaserScan,
		    '/scan',
		    self.registerScan,
			qos_profile=qos_profile_sensor_data)
			
	# ========================================================================
	# registerScan：每收到一帧 /scan 激光数据就调用一次
	# 流程：找最近点 → 两帧比对确认是稳定目标 → 计算目标角度/距离并发布
	# ========================================================================
	def registerScan(self, scan_data):
		# 把激光 360° 一圈的距离值（米）转成 numpy 数组，方便做向量化运算
		ranges = np.array(scan_data.ranges)
		# 按距离从小到大排序，得到"最近→最远"的下标顺序索引
		# 作用：先检查近处的点是不是目标（近处可信度更高）
		sortedIndices = np.argsort(ranges)

		# 判断本次扫描的最近激光帧在上帧的同样位置的上下两帧相差距离不超过0.2m是否出现
		minDistanceID = None          # 记录"最近真实目标"在数组中的下标（第几个激光束）
		minDistance   = float('inf')  # 记录该目标的距离，初始为正无穷
		if(not(self.lastScan is None)):
			# 只有缓存了上一帧，才能做两帧比对（第一帧进来时 lastScan 为空，跳过）
			for i in sortedIndices:
				# 从最近的激光束开始逐个检查
				tempMinDistance   = ranges[i]   # 当前激光束测到的距离
				a = i-2             # 对比窗口起点：当前束往前 2 个索引
				b = i+3             # 对比窗口终点：当前束往后 3 个索引（左开右闭）
				c = len(self.lastScan)  # 上一帧长度，用于越界裁剪
				windowIndex = np.clip([a,b],0,c)   # 把窗口裁剪到合法范围 [0, len] 内的 [a, b] 窗口区间
				window = self.lastScan[windowIndex[0]:windowIndex[1]]  # 取上一帧对应窗口的距离值进行比较
				with np.errstate(invalid='ignore'):
					# ★ 核心判断：上一帧这个位置附近（±2个束）有没有一个点，且距离和当前几乎一样（差 ≤ 0.2m）？
					#   有 → 说明这不是一闪而过的噪声，而是"稳定的真实目标"
					if(np.any(abs(window-tempMinDistance)<=0.2)):
						minDistanceID = i            # 记录这个目标的下标
						minDistance = ranges[minDistanceID]  # 记录距离
						break # 找到最近的真实目标，退出循环（后面的点更远，不需要）
		self.lastScan=ranges   # 缓存当前帧，供下一帧比对

		# ---- 结果处理：分"没找到目标"和"找到目标"两种 ----
		# 情况 1：minDistance 仍是无穷大没找到稳定目标（或只有噪声点），向object_tracker/info话题发布未找到信息
		if(minDistance > scan_data.range_max):
			msg = StringMsg()
			msg.data = 'laser:nothing found'
			self.get_logger().warn('laser no object found')  # 打印警告日志
			self.infoPublisher.publish(msg)   # 通知 laserfollower：当前没目标
			self.get_logger().info(' waiting again...')
		# 情况 2：找到了目标，发布它的角度和距离到object_tracker/current_position
		else:
			msgdata = PositionMsg()
			# 把"第几个激光束"换算成角度：角度 = 起始角 + 下标 × 每束的角度增量
			minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
			msgdata.angle_x = minDistanceAngle   # 目标相对小车正前方的角度（弧度）
			msgdata.angle_y = 42.0               # 预留字段（本功能未使用，固定 42）
			print(minDistanceID)
			msgdata.distance = float(minDistance) # 目标相对小车的距离（米）
			self.positionPublisher.publish(msgdata)  # 发布给 laserfollower 做跟随控制
def main(args=None):
    print('starting')
    rclpy.init(args=args)
    lasertracker = LaserTracker()   # 创建节点（构造函数里完成订阅/发布初始化）
    print('seem to do something')
    try:
        rclpy.spin(lasertracker)    # 阻塞运行，持续接收 /scan 调用 registerScan
    finally:
        lasertracker.destroy_node() # 退出时销毁节点
        rclpy.shutdown()

if __name__ == '__main__':
    main()
