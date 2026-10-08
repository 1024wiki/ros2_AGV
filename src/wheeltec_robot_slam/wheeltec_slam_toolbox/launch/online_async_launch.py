# ============================================================================
#  文件 : online_async_launch.py（wheeltec_slam_toolbox 包）
#  功能 : 启动 slam_toolbox 2D 建图（异步模式）
#
#  slam_toolbox 是什么？
#   轻量的图优化 SLAM 算法（和 Cartographer 类似，但更简单，
#   还支持"建图/重定位"模式切换）。
#
#  async（异步） vs sync（同步）：
#   - async：扫描匹配和建图分线程，响应快（本文件）
#   - sync ：单线程，简单但慢
#
#  slam_toolbox建图流程：
#   1.启动launch文件，会启动3个结点，ros2 launch wheeltec_slam_toolbox online_async_launch.py：
#    ① 启动小车底层（turn_on_wheeltec_robot.launch.py）
#       → 底盘驱动 + EKF + 模型/TF
#    ② 启动激光雷达（wheeltec_lidar.launch.py）
#       → 提供 /scan 激光数据
#    ③ 启动 async_slam_toolbox_node（核心建图）
#       → 融合 /scan + /odom_ekfcombined 建图，发布 /map
#   2.控制机器人缓慢运动进行边定位边建图。
#   3.采用命令保存地图，ros2 run nav2_map_server map_saver_cli -f ~/map/my_map
#
#  slam_toolbox建图原理：与cartographer建图原理完全相同，不同在于slam_toobox需要处理好的odom数据/odom_ekfcombined，并且更轻量。
#   1.猜：采用EKF融合后的/odom_ekfcombined里程计数据，通过运动预测机器人位置。
#   2.纠：在机器人预测位置获取激光雷达/scan话题数据，与已有地图进行扫描匹配（相关性搜索），
#          找到最吻合位置，修正预测位置，并把新扫描结果贴进地图。
#   3.调：在机器人扫描到曾经来过的位置时，进行回环检测确认，
#          然后进行全局图优化（Ceres），调整所有相关节点位置，消除累积漂移误差。
# ============================================================================

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ------------------------------------------------------------
    # 第 1 步：定位底层包的 launch 目录
    # ------------------------------------------------------------
    # 编译后 launch 文件实际安装在 install/.../share/.../launch
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # ------------------------------------------------------------
    # 第 2 步：启动小车底层 + 激光雷达
    # ------------------------------------------------------------
    # ① 启动小车底层节点
    #    注意：没传 carto_slam=true → EKF 用普通配置（publish_tf=true，EKF 自己发 odom→base TF）
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    # ② 启动激光雷达，提供 /scan 激光数据（建图核心输入）
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )

    # ------------------------------------------------------------
    # 第 3 步：启动 slam_toolbox 核心建图节点
    # ------------------------------------------------------------
    # ③ async_slam_toolbox_node（异步图优化建图）
    slam_toolbox_map = launch_ros.actions.Node(
        # 加载配置文件（所有建图参数都在这个 yaml 里）
        parameters=[
            get_package_share_directory("wheeltec_slam_toolbox") + '/config/mapper_params_online_async.yaml'
        ],
        package='slam_toolbox',           # 功能包名（ROS 官方包）
        executable='async_slam_toolbox_node',  # 异步建图可执行程序
        name='slam_toolbox',              # 节点名（yaml 顶层键对应）
        output='screen',                  # 日志输出到终端
        # ★ 话题重映射：slam_toolbox 默认订阅 /odom，
        #   重映射成实际存在的 /odom_ekfcombined（EKF 融合里程计）
        remappings=[('odom','/odom_ekfcombined')]
    )

    # ------------------------------------------------------------
    # 第 4 步：组装 LaunchDescription
    # ------------------------------------------------------------
    return LaunchDescription([
        wheeltec_robot,      # ① 底层
        wheeltec_lidar,      # ② 雷达
        slam_toolbox_map     # ③ slam_toolbox 建图
    ])
