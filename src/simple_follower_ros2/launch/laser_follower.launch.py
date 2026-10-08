# ============================================================================
#  文件 : laser_follower.launch.py（simple_follower_ros2 包）
#  功能 : 启动【激光雷达目标跟随】——小车自动跟踪激光检测到的最近物体
#
#  "激光跟随" = 激光雷达发现前方最近的目标 → 算出目标相对小车的角度和距离
#              → PID 控制器控制小车转向 + 前进/后退，始终保持与目标一定距离和角度。
#
#  激光目标检测节点：仅仅采用单纯的"最近点"可能是噪声/地面/偶然障碍，为保证稳定性，采用"两次扫描比对"的方法：
#   	只把【上一帧也有点、且距离几乎没变】的最近点当作"真实目标"，从而过滤掉闪烁的噪声点，让跟随更稳定。
#       具体作法是保存上个激光雷达数据帧的数据，在本次激光雷达数据到达后，按照距离从近到远的顺序，
#       先后比较每个角度的激光雷达数据帧与上个角度的激光雷达数据帧的相差角度在阈值内，判断基本一致并且相差距离不超过0.2m，
#       则判断为真实的最近点目标，发布这个角度和距离的自定义消息Position到话题/object_tracker/current_position。
#
#  激光跟随控制节点：在接收到【激光目标检测节点】发布的相对于目标点的角度和距离的话题/object_tracker/current_position消息时，
#       在判断距离和角度均不超过相应阈值时，则发布X轴和Y轴线速度为0，Z轴角速度为0的消息到 /cmd_vel 话题进行发布。
#       在判断距离或角度超过相应阈值时，采用PID控制器计算当前位姿和预期位姿的距离误差和角度误差，并根据该距离误差和角度误差计算
#       X轴线速度和Z轴角速度，Y轴线速度为0的消息到 /cmd_vel 话题进行发布。
#  
#  PID控制器计算和实现原理：
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
#   2. 启动激光雷达（wheeltec_lidar.launch.py）→ 提供 /scan 激光数据
#   3. 启动 lasertracker（激光目标检测节点）→ 订阅 /scan，找最近的目标点，发布其相对角度/距离 /object_tracker/current_position
#   4. 启动 laserfollower（激光跟随控制节点）→ 订阅目标位置/object_tracker/current_position，用 PID 算速度，发布 /cmd_vel 让小车跟随
# ============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    # ------------------------------------------------------------
    # 第 1 步：定位"底盘包"的 launch 目录（供下面 Include 引用）
    # ------------------------------------------------------------
    # get_package_share_directory()：返回包在 install 目录下的共享路径，
    # 编译后 launch 文件实际装在 install/turn_on_wheeltec_robot/share/.../launch
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # ------------------------------------------------------------
    # 第 2 步：组装要启动的节点 / 子launch
    # ------------------------------------------------------------

    # ① 小车底盘启动节点
    #    作用：启动底盘串口驱动 + IMU去漂移 + EKF位姿融合 + 机器人模型TF
    #    提供：/odom_raw、/imu/stm32_raw、/odom_ekfcombined、订阅 /cmd_vel
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )

    # ② 激光雷达传感器启动节点
    #    作用：驱动激光雷达，发布 /scan（LaserScan 激光扫描数据）
    #    跟随需要 /scan 才能"看到"前方目标
    wheeltec_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )

    # ③ 激光目标检测节点（lasertracker）
    #    对应源码 simple_follower_ros2/laserTracker.py
    #    作用：订阅 /scan，在激光点云里找【最近的一个目标】，发布该目标相对小车的角度和距离（PositionMsg 类型）
    #    发布：/object_tracker/current_position（角度 angle_x + 距离 distance）
    #          /object_tracker/info（找不到目标时的提示信息）
    laser_tracker = launch_ros.actions.Node(
        package='simple_follower_ros2',   # 功能包名
        executable='lasertracker',        # 可执行程序（setup.py 里的 console_scripts 入口）
        name='lasertracker',)             # 节点名

    # ④ 激光跟随控制节点（laserfollower）
    #    对应源码 simple_follower_ros2/laserfollower.py
    #    作用：订阅 lasertracker 发布的目标位置（/object_tracker/current_position），
    #          用 PID 控制器（角度环 + 距离环）计算转向角速度与前进线速度，发布 /cmd_vel 控制小车始终保持与目标约 0.6m 的距离并正对目标
    #    发布：/cmd_vel（速度指令）→ 底盘驱动节点接收执行
    #          /laser_follow_flag（跟随激活标志）
    laser_follower = launch_ros.actions.Node(
        package='simple_follower_ros2', 
        executable='laserfollower',)

    # ------------------------------------------------------------
    # 第 3 步：按顺序加入 LaunchDescription 并返回
    # ------------------------------------------------------------
    # 启动顺序：① 先底层 → ② 再雷达 → ③ 检测 → ④ 跟随
    # （节点启动本身无强依赖，可并行就绪；底层和雷达必须先于检测/跟随出数据）
    return LaunchDescription([
        wheeltec_robot,   # ① 底盘
        wheeltec_lidar,   # ② 雷达
        laser_tracker,    # ③ 目标检测
        laser_follower    # ④ 跟随控制
    ])

