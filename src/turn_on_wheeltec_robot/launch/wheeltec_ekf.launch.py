# ============================================================================
#  文件 : wheeltec_ekf.launch.py
#  功能 : 启动 EKF（扩展卡尔曼滤波）节点，融合 IMU + 轮式里程计
#
#  为什么要融合？
#    轮式里程计（/odom）：短期准、长期会漂（打滑、轮胎磨损导致误差累积）
#    IMU（/imu/data）   ：高频、平滑，但会零点漂移、积分会发散
#    两者各有优缺点，EKF 把它们"按可信度的协方差矩阵权重融合"，输出更稳定准确的位姿。
#
#  输出：
#    /odometry/filtered  ->  重映射为 /odom_combined（融合后的里程计）
#    /odom_combined -> base_footprint 的 TF 坐标变换
#
#  三种模式（通过 launch 参数选择，默认用普通 ekf.yaml）：
#    carto_slam=true : 用于 Cartographer 建图（ekf_carto.yaml）
#    robot_nav=true  : 用于 NAV2 导航（ekf_nav.yaml）
#    两者都 false    : 普通模式（ekf.yaml）
# ============================================================================

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ------------------------------------------------------------
    # 第 1 步：定位本包，拼接出三个 EKF 配置文件的路径
    # ------------------------------------------------------------
    pkg_dir = get_package_share_directory('turn_on_wheeltec_robot')

    ekf_config = Path(pkg_dir, 'config', 'ekf.yaml')          # 普通模式
    ekf_carto_config = Path(pkg_dir, 'config', 'ekf_carto.yaml')  # 建图模式
    ekf_nav_config = Path(pkg_dir, 'config', 'ekf_nav.yaml')  # 导航模式

    # ------------------------------------------------------------
    # 第 2 步：声明 launch 参数（默认 false）
    # ------------------------------------------------------------
    # 注意：这里没有写 default，所以默认值是空字符串''，
    #       真正默认值由下面 carto_slam_dec / robot_nav_dec 给出（'false'）
    carto_slam = LaunchConfiguration('carto_slam')
    robot_nav = LaunchConfiguration('robot_nav')

    carto_slam_dec = DeclareLaunchArgument('carto_slam', default_value='false')
    robot_nav_dec = DeclareLaunchArgument('robot_nav', default_value='false')

    # 启动时打印一行日志，方便确认参数值
    # LogInfo 的 msg 支持列表，里面可以混用普通字符串和 LaunchConfiguration
    log_args = LogInfo(msg=['[ekf launch] carto_slam=', carto_slam, ', robot_nav=', robot_nav])

    # ------------------------------------------------------------
    # 第 3 步：定义动态生成 EKF 节点的函数（OpaqueFunction 会在运行时调用）
    # ------------------------------------------------------------
    def launch_ekf(context, *args, **kwargs):
        # perform(context) 取出参数的"字符串值"，再转成 bool
        carto_slam_val = LaunchConfiguration('carto_slam').perform(context)
        robot_nav_val = LaunchConfiguration('robot_nav').perform(context)
        # lower() 统一转小写，避免用户传 True/TRUE 也识别
        carto_slam_bool = carto_slam_val.lower() == 'true'
        robot_nav_bool = robot_nav_val.lower() == 'true'

        nodes = []  # 要启动的节点列表

        # 分支1：建图模式（carto_slam=true）
        if carto_slam_bool:
            nodes.append(
                Node(
                    package='robot_localization',   # 官方机器人位姿融合包
                    executable='ekf_node',          # 运行 EKF 节点
                    name='carto_ekf_filter_node',   # 节点名（用于区分三种模式）
                    parameters=[ekf_carto_config],  # 载入对应配置文件
                    remappings=[
                        # 把 EKF 默认输出话题 /odometry/filtered 改名为 /odom_combined，统一出口
                        ('/odometry/filtered', 'odom_ekfcombined')
                    ]
                )
            )

        # 分支2：导航模式（robot_nav=true）
        elif robot_nav_bool:
            nodes.append(
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='nav_ekf_filter_node',
                    parameters=[ekf_nav_config],
                    remappings=[
                        ('/odometry/filtered', 'odom_ekfcombined')
                    ]
                )
            )

        # 分支3：普通模式（默认，两者都 false）
        else:
            nodes.append(
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    parameters=[ekf_config],
                    remappings=[
                        ('/odometry/filtered', 'odom_ekfcombined')
                    ]
                )
            )

        return nodes  # 返回实际要启动的节点

    # ------------------------------------------------------------
    # 第 4 步：组装 LaunchDescription
    # ------------------------------------------------------------
    return LaunchDescription([
        carto_slam_dec,   # 声明参数1
        robot_nav_dec,    # 声明参数2
        log_args,         # 打印参数日志
        OpaqueFunction(function=launch_ekf),  # 运行时动态生成并启动 EKF 节点
    ])