# ============================================================================
#  文件 : slam_gmapping.launch.py（slam_gmapping 包）
#  功能 : 启动 gmapping 2D 建图（SLAM）
#
#  gmapping 是什么？
#   经典的老牌 SLAM 算法（粒子滤波），适合学习和小场景建图。它用"一簇粒子"来猜机器人位置，
#   每个粒子 = 一个假想的机器人位姿，不断根据激光/里程计更新，淘汰不相符的粒子，留下符合的粒子，最后粒子会收敛出最可能的位姿并拼出地图。
#
#  gmapping建图流程：
#   1.启动本launch文件，打开3个结点：ros2 launch slam_gmapping slam_gmapping.launch.py
#    ① 启动小车底层（turn_on_wheeltec_robot.launch.py）
#       → 底盘驱动 + EKF + 模型/TF
#    ② 启动激光雷达（wheeltec_lidar.launch.py）
#       → 提供 /scan 激光数据
#    ③ 启动 slam_gmapping 节点（核心建图）
#       → 融合 /scan + /odom_ekfcombined 建图，发布 /map
#   2.遥控小车缓慢走遍周围环境。
#   3.采用命令保存地图，ros2 run nav2_map_server map_saver_cli -f ~/map/my_map
#
#  gmapping建图原理：
#   采用一定数量的粒子，每个粒子代表一个假想的机器人位姿，随着每次激光扫描收到一次激光/scan话题数据，进行以下过程淘汰不符合粒子并保留符合粒子：
#   1.预测：计算每个粒子按照/odom_ekfcombined里程计数据进行计算位姿。
#   2.打分：比对/scan激光扫描结果对每个粒子进行匹配打分。
#   3.重采样：按照分数重新抽样，高分粒子被复制多份，低分粒子直接淘汰，粒子总数不变。
#   4.用高分粒子的位置，叠加当前激光扫描结果贴进地图/map，进行更新地图。
# ============================================================================

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # use_sim_time：是否用仿真时间（Gazebo 用 true，真机用 false）
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')

    # 定位底层包的 launch 目录（和 Cartographer 一样）
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # ① 启动激光雷达，提供 /scan 激光数据（建图核心输入）
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )
    # ② 启动小车底层（注意：这里没传 carto_slam=true，
    #    所以 EKF 用普通配置 ekf.yaml，publish_tf=true 自己发 odom→base 的 TF）
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    return LaunchDescription([
        wheeltec_robot,    # ② 底层
        wheeltec_lidar,    # ① 雷达
        # 设置日志输出流缓冲（避免日志被缓冲导致看不到实时输出）
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # ③ 核心建图节点：slam_gmapping
        #    - 订阅 /scan（激光）、/odom_ekfcombined（EKF里程计）
        #    - 用粒子滤波融合，实时建图，发布 /map
        launch_ros.actions.Node(
            package='slam_gmapping',       # 功能包名
            executable='slam_gmapping',    # 可执行程序
            output='screen',               # 日志输出到终端
            parameters=[{'use_sim_time':use_sim_time}]),  # 传 use_sim_time 参数
    ])
