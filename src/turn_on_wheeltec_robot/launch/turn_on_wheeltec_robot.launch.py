# ============================================================================
#  文件 : turn_on_wheeltec_robot.launch.py
#  功能 : 小车【最基本启动】的入口 launch 文件（也叫 bringup 启动文件）
#
#  运行方式：
#     ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
#
#  启动后主要完成 4 件事：
#    1. 底盘串口驱动：base_serial.launch.py
#       -> 启动 wheeltec_robot_node（与 STM32 单片机串口通信）
#       -> 订阅 cmd_vel 控制轮子，发布 /odom 里程计、/imu、电压等话题
#    2. 关节状态发布：joint_state_publisher
#       -> 发布 /joint_states，供 RViz 显示机器人模型
#    3. 位姿融合：wheeltec_ekf.launch.py
#       -> 用 EKF 把 IMU 和轮式里程计融合，输出更稳定的位姿
#    4. 机器人模型：robot_mode_description.launch.py
#       -> 加载 URDF 模型，发布 TF 坐标变换树
#
#  注意：本文件【不包含】SLAM 建图、自主导航等功能，
#        那些在 wheeltec_robot_slam / wheeltec_robot_nav2 包中。
# ============================================================================

import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition


def generate_launch_description():
    # ROS2 launch 文件的主入口函数，函数名必须是 generate_launch_description
    # （这是 ROS2 框架的约定，launch 时会自动调用它）
    # 返回值：一个 LaunchDescription 对象，描述"要启动哪些节点/子launch"

    # ------------------------------------------------------------
    # 第 1 步：获取本功能包在 install 目录下的共享路径
    # ------------------------------------------------------------
    # get_package_share_directory() 返回类似：
    #   install/turn_on_wheeltec_robot/share/turn_on_wheeltec_robot
    # 注意：这个路径在编译（colcon build）之后才会存在
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    # 拼接出本包 launch 子目录的完整路径，方便引用其他 .launch.py 文件
    launch_dir = os.path.join(bringup_dir, 'launch')

    # ------------------------------------------------------------
    # 第 2 步：声明"运行时可以被外部覆盖"的 launch 参数
    # ------------------------------------------------------------
    # LaunchConfiguration('xx', default='False') : 一个参数占位符
    # DeclareLaunchArgument('xx', default_value='False') : 正式声明该参数
    # 这样运行时就可以临时覆盖，而不用改代码，例如：
    #   ros2 launch ... turn_on_wheeltec_robot.launch.py carto_slam:=True

    # 参数1：是否同时启动 cartographer 建图（默认 False = 不启动）
    carto_slam = LaunchConfiguration('carto_slam', default='False')
    carto_slam_dec = DeclareLaunchArgument('carto_slam', default_value='False')

    # 参数2：是否同时启动导航（默认 False = 不启动）
    robot_nav = LaunchConfiguration('robot_nav', default='False')
    robot_nav_dec = DeclareLaunchArgument('robot_nav', default_value='False')

    # imu 配置文件的路径（本包 config 目录下的 imu.yaml）
    # （本文件中暂时没有直接使用，由子 launch 读取）
    imu_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'imu.yaml')

    # ------------------------------------------------------------
    # 第 3 步：组装各个子 launch / 节点
    # ------------------------------------------------------------

    # (1) 底盘核心启动：base_serial.launch.py
    #     作用：启动 wheeltec_robot_node（串口驱动主节点），
    #           实现与下位机stm32的运动底盘进行通信，不断循环接收下位机上传的基础底盘运动状态帧等，
    #           并进行计算发布odom里程计话题、imu/data_raw(imu/data_filtered)IMU话题等数据，
    #           同时接收/cmd_vel话题进行下发运动控制帧控制小车运动
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
    )
    #    IMU 零点漂移矫正节点：imu_processor.launch.py
    #      作用：订阅 /imu/stm32_raw 话题数据，进行去零点漂移处理，发布 /imu/imu_filtered 话题
    #      注意：每个子 launch 必须用独立的 IncludeLaunchDescription 启动（不能塞进同一个）
    imu_processor = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'imu_processor.launch.py'))
    )

    # (2) EKF 里程计融合：wheeltec_ekf.launch.py
    #     作用：调用ROS官方的 robot_localization 包，把 IMU + 轮式里程计采用EKF结点进行融合，
    #           将/odom_raw话题和/imu/data_raw(/imu/data_filtered)话题进行融合，
    #           输出更稳定、更精确的位姿（odom_ekfcombined话题发布），以提供给建图、导航使用。
    #           输出odom_ekfconbined起点坐标->base_footprintf小车本体投影坐标的TF坐标变换。
    #     launch_arguments：把上面声明的 carto_slam / robot_nav 参数传给它，
    #           分别对应3种启动EKF结点进行融合的配置方式：普通、建图、导航。
    robot_ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_ekf.launch.py')),
            launch_arguments={'carto_slam': carto_slam, 'robot_nav': robot_nav}.items(),
    )

    # (3) 机器人模型描述+关节状态结点发布：robot_mode_description.launch.py
    #     作用：启动joint_state_publisher 节点，发布机器人关节状态话题 /joint_states，
    #           加载 URDF 机器人模型并启动 robot_state_publisher，发布URDF中fixed关节joint的静态TF变换，
    #           配合joint_state_publisher 节点的 /joint_states，发布URDF中continue关节joint的动态TF变换， 
    #           这样 RViz 才能正确显示并跟踪机器人。
    car_mode_type = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )

    # ------------------------------------------------------------
    # 第 4 步：把上面所有"动作"按顺序加入 LaunchDescription 并返回
    # ------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(carto_slam_dec)               # 声明参数1（carto_slam）
    ld.add_action(robot_nav_dec)                # 声明参数2（robot_nav）
    ld.add_action(wheeltec_robot)               # ① 底盘串口驱动 + 传感器
    ld.add_action(imu_processor)                #    IMU 零点漂移矫正节点
    ld.add_action(robot_ekf)                    # ② EKF 位姿融合
    ld.add_action(car_mode_type)                # ④ 关节状态结点+机器人模型 / TF
    return ld

