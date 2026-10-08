# ============================================================================
#  文件 : visual_follower.launch.py（simple_follower_ros2 包）
#  功能 : 启动【视觉目标跟随】——小车自动跟踪相机画面中的指定颜色目标
#
#  "视觉跟随" = 相机拍摄画面 → 按颜色(HSV)找出目标 → 算出目标相对小车的角度和距离
#              → PID 控制器控制小车转向 + 前进/后退，始终保持与目标一定距离和角度。
#
#  视觉目标检测节点：采用 HSV 颜色阈值分割的方法，在画面中找出目标颜色区域：
#   1.先采用message_filters过滤器同步订阅/camera/color/image_raw彩色图和/camera/depth/image_raw深度图的话题消息，
#     保证获取的RGB图和深度图数据是几乎同步的，获取的数据才有意义。
#   2.把 RGB 图像转成 HSV 色彩空间，用颜色阈值(inRange)选出目标颜色像素得到二值掩码，即找到检测目标的二值分割图后，先进行腐蚀去噪、然后寻找轮廓，
#     按轮廓面积从大到小排序进行判断轮廓是否为检测目标，先计算轮廓的外接矩阵，并通过该外接矩阵计算轮廓中心点(X,Y)位置，从深度图中计算中心点位置周围
#     平均深度D，通过和上帧检测物体的中心点和距离进行比较，判断误差在可控制范围内时，则确定该轮廓为检测目标轮廓。
#
#   这里只进行简单的根据颜色来作为检测目标，可以考虑采用深度学习的计算机视觉算法，比如YOLOV8、图像分割算法等进行物品识别，根据识别的物品作为检测目标。
#
#   3.在判断轮廓为检测目标轮廓时，根据轮廓的中心点(X,Y)位置和深度D，采用小孔成像原理计算检测物品轮廓中心点偏移机器人相机中心的X轴和Y轴角度，
#     然后根据深度D，构造角度和距离的自定义消息 Position 到 /object_tracker/current_position。
#  【小孔成像原理计算偏移角度】
#       假设相机焦距为f（光心到图像平面的距离)，画面宽高为(W,H)，此时物体X轴和y轴中心点为(X,Y)
# 	  则X和Y轴最大角度(tanA,tanB) = (W/2, H/2)/f，物体相对于相机的X轴和Y轴角度分别为(tana,tanb)=（X-W/2, Y-H/2）/f（相对于中心点左负右正)
#	  则此时物品相当于机器人的X轴和Y轴的偏移角度为(a,b) = arctan(2*(X/W, Y/H)-1)
#
#  视觉跟随控制节点：和激光雷达跟随控制节点的实现逻辑几乎完全相同
#     在接收到【视觉目标检测节点】发布的相对于目标点的角度和距离的话题/object_tracker/current_position消息时，
#     在判断距离和角度均不超过相应阈值时，则发布X轴和Y轴线速度为0，Z轴角速度为0的消息到 /cmd_vel 话题进行发布。
#     在判断距离或角度超过相应阈值时，采用PID控制器计算当前位姿和预期位姿的距离误差和角度误差，并根据该距离误差和角度误差计算
#     X轴线速度和Z轴角速度，Y轴线速度为0的消息到 /cmd_vel 话题进行发布。
#
#  PID控制器计算和实现原理（与激光跟随 laser_follower.launch.py 完全相同）：
#   【为什么需要 PID】目标位置是"位置量"，而小车只能通过"速度"来控制，需要一个控制器把"位置误差"换算成"该给多大速度"。
#
#   【PID 公式】u = Kp*e + Ki*∫e·dt + Kd*de/dt
#   三项作用（增益 Kp/Ki/Kd 均取正，方向由误差定义与输出映射决定）：
#     ① 比例项 P：e = 误差，误差越大输出越大 → 快速逼近目标（"现在错多少纠多少"）
#     ② 积分项 I：把过去误差累加 → 消除稳态误差，让车"精准到位"，防止接近时误差很小导致始终保持误稳态误差（"历史欠账清算"）
#     ③ 微分项 D：看误差变化率 → 提前减速/阻尼，防止超调震荡，误差下降过快导致越界（"趋势预警刹车"）
#
#   【采用双通道 PID 方式同时计算线速度和角速度】（向量形式，两个通道独立并行）：
#     通道[0] 角度环：误差e = 当前角度 - 目标角度(0) → 输出转向控制量 → angular.z 角速度
#     通道[1] 距离环：误差e = 当前距离 - 目标距离(0.6m) → 输出前进控制量 → linear.x 线速度
#   【离散化PID公式】∫e·dt积分→累加(Σe·Δt)、de/dt微分→差分((e_k - e_{k-1})/Δt)、dt时间微分Δt = t_k - t_{k-1}
#
#  跟随流程：
#   1. 启动小车底层（turn_on_wheeltec_robot.launch.py）→ 底盘驱动 + EKF + 模型/TF
#   2. 启动相机（wheeltec_camera.launch.py）→ 提供 /camera/color/image_raw + /camera/depth/image_raw 图像数据
#   3. 启动 visualtracker（视觉目标检测节点）→ 订阅 RGB+深度图，按颜色找目标，发布其相对角度/距离 /object_tracker/current_position
#   4. 启动 visualfollow（视觉跟随控制节点）→ 订阅目标位置/object_tracker/current_position，用 PID 算速度，发布 /cmd_vel 让小车跟随
# ============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,TextSubstitution
import launch_ros.actions
from launch.conditions import IfCondition

def generate_launch_description():
    # ------------------------------------------------------------
    # 第 1 步：声明可被外部覆盖的 launch 参数（相机画面尺寸）
    # ------------------------------------------------------------
    # Width/Height：图像分辨率（像素），运行时可用 width:= 覆盖
    # 不同深度相机分辨率不同，视觉检测节点需要据此校验图像尺寸是否匹配
    Width_arg = DeclareLaunchArgument(
      'Width', default_value=TextSubstitution(text='640')    # 默认宽 640 像素
    )
    #if use DaBai or Gemini [Height = 400]（注释：若用 DaBai/Gemini 相机，高度为 400）
    Height_arg = DeclareLaunchArgument(
      'Height', default_value=TextSubstitution(text='480')   # 默认高 480 像素
    )

    # ------------------------------------------------------------
    # 第 2 步：定位"底盘包"的 launch 目录（供下面 Include 引用）
    # ------------------------------------------------------------
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # ------------------------------------------------------------
    # 第 3 步：组装要启动的节点 / 子launch
    # ------------------------------------------------------------

    # ① 小车底盘启动节点
    #    作用：启动底盘串口驱动 + IMU去漂移 + EKF位姿融合 + 机器人模型TF
    #    提供：/odom_raw、/imu/stm32_raw、/odom_ekfcombined、订阅 /cmd_vel
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )

    # ② 相机传感器启动节点
    #    作用：驱动深度相机，发布彩色图 /camera/color/image_raw 和深度图 /camera/depth/image_raw
    #    视觉跟随需要"颜色找目标 + 深度测距离"两路图像
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_camera.launch.py')),
    )

    # ③ 视觉目标检测节点（visualtracker）
    #    对应源码 simple_follower_ros2/visualTracker.py
    #    作用：订阅时间同步的 RGB 图 + 深度图，HSV 颜色阈值分割找目标（默认红色），
    #          结合深度图计算目标中心坐标和平均距离，发布该目标相对小车的角度和距离
    #    发布：/object_tracker/current_position（角度 angle_x/angle_y + 距离 distance）
    #    参数：Width/Height（相机分辨率，用于校验图像尺寸）
    visial_track_node = launch_ros.actions.Node(
        package='simple_follower_ros2',      # 功能包名
        executable='visualtracker',          # 可执行程序（setup.py 的 console_scripts 入口）
        name='visualtracker',                # 节点名
        parameters=[
            {'Width': LaunchConfiguration('Width')},    # 画面宽度参数
            {'Height': LaunchConfiguration('Height')}   # 画面高度参数
            ]
    )

    # ④ 视觉跟随控制节点（visualfollow）
    #    对应源码 simple_follower_ros2/visualFollower.py
    #    作用：订阅 visualtracker 发布的目标位置（/object_tracker/current_position），
    #          用 PID 控制器（角度环 + 距离环）计算转向角速度与前进线速度，
    #          发布 /cmd_vel 控制小车始终保持与目标一定距离并正对目标
    #    发布：/cmd_vel（速度指令）→ 底盘驱动节点接收执行
    visial_follow_node = launch_ros.actions.Node(
        package='simple_follower_ros2', 
        executable='visualfollow',           # 可执行程序（visualFollower.py:main）
        name='visualfollow',
        output='screen',                     # 日志输出到终端
    )

    # ------------------------------------------------------------
    # 第 4 步：按顺序加入 LaunchDescription 并返回
    # ------------------------------------------------------------
    # 启动顺序：先声明参数 → ① 底层 → ② 相机 → ③ 检测 → ④ 跟随
    # （节点启动本身无强依赖，可并行就绪；底层、相机必须先于检测/跟随出数据）
    return LaunchDescription([
        Width_arg,          # 声明画面宽度参数
        Height_arg,         # 声明画面高度参数
        wheeltec_robot,     # ① 底盘
        wheeltec_camera,    # ② 相机
        visial_track_node,  # ③ 视觉目标检测
        visial_follow_node, # ④ 视觉跟随控制
    ])

