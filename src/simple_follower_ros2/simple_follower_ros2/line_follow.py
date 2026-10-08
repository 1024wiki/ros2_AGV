#!/usr/bin/env python3
# ============================================================================
# 文件 : line_follow.py（simple_follower_ros2 包）
# 功能 : 视觉巡线节点（follower）——小车沿地面上的彩色线条行驶
#
# 原理：
#   1. 订阅相机图像，转 HSV 色彩空间；
#   2. 按选定的颜色阈值(inRange)分割出"线"的像素（二值掩码）；
#   3. 只取图像底部一小条区域（贴近车头，代表"眼前的路"）；
#   4. 计算该区域中线的重心(几何中心) X 坐标；
#   5. 重心偏离画面中心多少 → 误差 → 控制小车转向角速度，保持线在画面中间；
#   6. 通过 OpenCV 窗口的滑动条实时切换要追踪的颜色（红/绿/蓝/黄/黑）。
#
# 与激光/视觉跟随的区别：这里跟踪的是"地面上的线"，不是"目标物"，
#   而且不用 PID，只用简单的比例控制（P 控制）。
# ============================================================================

import cv2
import numpy as np
import rclpy
import time
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
import cv_bridge
from geometry_msgs.msg import Twist

last_erro=0     # 上一帧的误差（用于计算误差变化率，当前未真正使用）
tmp_cv = 0      # 全局标志（保留，未实际使用）

def nothing(s):
    pass   # OpenCV 滑动条回调占位函数（滑动条必须有回调，但这里不需要处理）

# 各颜色在 HSV 空间的阈值范围（H,S,V 下限, H,S,V 上限）
# 格式：(lowerH, lowerS, lowerV, upperH, upperS, upperV)
col_black = (0,0,0,180,255,46)      # 黑线
col_red = (0,100,80,10,255,255)     # 红线（红色在 HSV 中 H 跨 0 和 180 两端）
col_blue = (100,43,46,124,255,255)  # 蓝线
col_green= (35,43,46,77,255,255)    # 绿线
col_yellow = (26,43,46,34,255,255)  # 黄线

# 滑动条标题：0=红 1=绿 2=蓝 3=黄 4=黑
Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'


class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        self.bridge = cv_bridge.CvBridge()   # ROS Image 消息 <-> OpenCV 图像转换桥
        qos = QoSProfile(depth=10)
        self.mat = None
        # 图像话题名参数（默认空，运行时可用 image_input:= 覆盖）
        self.declare_parameter('image_input', '')
        self.image_input = self.get_parameter('image_input').get_parameter_value().string_value
        # 订阅相机图像（话题名由 image_input 参数决定）
        self.image_sub = self.create_subscription(
            Image,
            self.image_input,
            self.image_callback,
            qos)

        # 发布速度指令 /cmd_vel → 底盘驱动执行
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.twist = Twist()
        self.tmp = 0   # 标志：只初始化一次 OpenCV 窗口和滑动条

    # ========================================================================
    # image_callback：每收到一帧图像调用一次（巡线核心）
    # 流程：BGR→HSV → 按颜色分割 → 取底部条带 → 求重心 → 偏差控制转向
    # ========================================================================
    def image_callback(self, msg):
        global last_erro
        global tmp_cv
        # ---- 1. 首次进入：创建调参窗口 + 颜色选择滑动条 ----
        if self.tmp==0:
            cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)              # 创建窗口
            cv2.createTrackbar(Switch,'Adjust_hsv',0,4,nothing)          # 创建滑动条(0~4)切换颜色
            self.tmp=1

        # 把 ROS Image 消息转成 OpenCV BGR 图像
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # BGR → HSV（hsv将RGB图像分解成色调H，饱和度S，明度V，颜色分割更鲁棒）
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # ---- 2. 形态学处理：先腐蚀后膨胀（开运算），去噪点 ----
        # 颜色的范围说明（inRange 三参数含义）：
        #   lower：低于这个值的像素 → 置 0
        #   upper：高于这个值的像素 → 置 0
        #   在 lower~upper 之间的像素 → 置 255
        kernel = numpy.ones((5,5),numpy.uint8)             # 5x5 卷积核
        hsv_erode = cv2.erode(hsv,kernel,iterations=1)     # 腐蚀：消除小噪点
        hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)  # 膨胀：恢复线条宽度

        # ---- 3. 读取滑动条当前位置，选择要追踪的颜色 ----
        m=cv2.getTrackbarPos(Switch,'Adjust_hsv')   # 0红 1绿 2蓝 3黄 4黑
        if m == 0:      # 红色
            lowerbH=col_red[0]; lowerbS=col_red[1]; lowerbV=col_red[2]
            upperbH=col_red[3]; upperbS=col_red[4]; upperbV=col_red[5]
        elif m == 1:    # 绿色
            lowerbH=col_green[0]; lowerbS=col_green[1]; lowerbV=col_green[2]
            upperbH=col_green[3]; upperbS=col_green[4]; upperbV=col_green[5]
        elif m == 2:    # 蓝色
            lowerbH=col_blue[0]; lowerbS=col_blue[1]; lowerbV=col_blue[2]
            upperbH=col_blue[3]; upperbS=col_blue[4]; upperbV=col_blue[5]
        elif m == 3:    # 黄色
            lowerbH=col_yellow[0]; lowerbS=col_yellow[1]; lowerbV=col_yellow[2]
            upperbH=col_yellow[3]; upperbS=col_yellow[4]; upperbV=col_yellow[5]
        elif m == 4:    # 黑色
            lowerbH=col_black[0]; lowerbS=col_black[1]; lowerbV=col_black[2]
            upperbH=col_black[3]; upperbS=col_black[4]; upperbV=col_black[5]
        else:           # 其他：全范围（全白，不做分割）
            lowerbH=0; lowerbS=0; lowerbV=0
            upperbH=255; upperbS=255; upperbV=255

        # ---- 4. 颜色阈值分割：得到二值掩码（线=白255，其余=黑0） ----
        mask=cv2.inRange(hsv_dilate,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
        # 把掩码叠加到原图上（仅调试显示用，不影响后续计算）
        masked = cv2.bitwise_and(image, image, mask=mask)

        # ---- 5. 只保留图像底部的一条窄带（贴近车头，代表"眼前的路"） ----
        # 因为只考虑底部 30 行宽的图像，所以用 numpy 切片把以外的区域清空为 0
        h, w, d = image.shape      # 图像 高、宽、通道数
        search_top = h-30          # 搜索区域上边界（底部 30 行）
        search_bot = h             # 搜索区域下边界（图像最底）
        mask[0:search_top, 0:w] = 0     # 清空上半部分
        mask[search_bot:h, 0:w] = 0     # 清空底部边缘（超出范围）

        # ---- 6. 计算 mask 的重心（几何中心），即线的中心位置 ----
        M = cv2.moments(mask)      # 计算图像矩（描述形状的统计量）
        if M['m00'] > 0:           # m00 = 白色像素总数，>0 说明有线条被检测到
            cx = int(M['m10']/M['m00'])   # 重心 X 坐标（m10/m00）
            cy = int(M['m01']/M['m00'])   # 重心 Y 坐标（m01/m00）
            #（以下画圆为调试用，已注释）
            #cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)
            #cv2.circle(image, (cx-60, cy), 10, (0, 0, 255), -1)
            #cv2.circle(image, (w/2, h), 10, (0, 255, 255), -1)
            if cv2.circle:   # 恒真，仅为保留原代码结构
                # ---- 7. 计算偏差：线的重心偏离"期望位置"多少 ----
                # 期望线在画面中心偏右 60 像素处（cx - w/2 - 60 = 0 时对齐）
                erro = cx - w/2-60
                d_erro=erro-last_erro   # 误差变化率（微分项，当前系数为0未使用）
                self.twist.linear.x = 0.11   # 前进速度固定 0.11 m/s（巡线匀速前进）
                # ---- 8. 比例控制：误差越大，转向越猛 ----
                # angular.z = -(误差 * Kp - 误差变化率 * Kd)
                # Kp=0.0011（比例），Kd=0（微分未启用，top_akm_bs 车型注释）
                if erro<0:
                    self.twist.angular.z = -(float(erro)*0.0011-float(d_erro)*0.0000) #top_akm_bs
                elif erro>0:
                    self.twist.angular.z = -(float(erro)*0.0011-float(d_erro)*0.0000) #top_akm_bs
                else :
                    self.twist.angular.z = 0.0   # 偏差为0 → 直行
                last_erro=erro   # 保存本次误差供下一帧算微分
        else:
            # ---- 没检测到线（画面里没有该颜色）→ 停车 ----
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        # ---- 9. 发布速度指令 + 显示调试窗口 ----
        self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("Adjust_hsv", mask)   # 显示掩码窗口（白线=检测到的线）
        cv2.waitKey(3)                   # 等待 3ms 刷新窗口
        #cv2.imshow("Adjust_hsv", mask)
        #print('start windows')
        #cv2.waitKey(3)

def main(args=None):
    rclpy.init(args=args)
    print('start patrolling')
    follower = Follower()   # 创建节点
    # 手动 spin_once 循环（每 0.1 秒处理一次，保证 OpenCV 窗口能及时刷新）
    while rclpy.ok():
        rclpy.spin_once(follower)
        time.sleep(0.1)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
