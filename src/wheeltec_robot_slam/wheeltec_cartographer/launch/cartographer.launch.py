# ============================================================================
#  文件 : cartographer.launch.py（wheeltec_cartographer 包）
#  功能 : 启动 Cartographer 2D 建图（SLAM）
#
#  建图 = 让机器人在环境中走一圈，同时用激光/里程计/IMU 拼出环境地图
#  cartographer建图流程：
#   1.启动本launch文件，打开 4 个结点：
#    ① 启动小车底层（turn_on_wheeltec_robot.launch.py，传 carto_slam=true）
#       → 底盘驱动 + EKF（建图配置）+ 模型/TF
#    ② 启动激光雷达（wheeltec_lidar.launch.py）
#       → 提供 /scan 激光数据
#    ③ 启动 cartographer_node（核心建图节点）
#       → 融合 /scan + /odom + /imu，实时建图
#    ④ 启动 cartographer_occupancy_grid_node（地图栅格节点）
#       → 把 Cartographer 内部子图转成 ROS 标准 /map 地图话题
#   运行方式：
#    ros2 launch wheeltec_cartographer cartographer.launch.py
#
#   2.控制小车运动，缓慢在房间内扫描周围环境进行建图。
#   3.执行ros2 run nav2_map_server map_saver_cli -f ~/map/my_map命令，生成my_map.pgm格栅地图文件
#
#  catorgrapher建图原理："里程计+IMU预测 + 激光扫描匹配 + 回环检测优化" 三步，让机器人边走动边"定位自己 + 拼接环境地图"，最后输出一张完整地图。
#   1.猜：采用原始/odom_raw里程计数据+原始IMU数据/stm32_raw，通过增量进行运动预测机器人位置。
#   2.纠：在机器人预测位置获取激光雷达/scan话题数据，与已有子图进行扫描匹配，找到最吻合位置。进行修正预测位置，并把新扫描结果贴进子图。
#   3.调：在机器人扫描到曾经来过的位置时，进行回环检测确认，然后进行全局优化，调整所有相关子图位置，消除累积漂移误差。
# ============================================================================

import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 声明 launch 参数（可被命令行覆盖）
    # use_sim_time：是否用仿真时间（Gazebo 用 true，真机用 false）
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # cartographer_config_dir：Cartographer 配置文件目录（本包 config 目录）
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
        default=os.path.join(get_package_share_directory('wheeltec_cartographer') , 'config'))
    # configuration_basename：具体使用的 lua 配置文件（核心参数都在这里）
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer.lua')

    # resolution：地图栅格分辨率（每格代表多少米，0.05=每格5cm，越小越精细）
    resolution = LaunchConfiguration('resolution', default='0.05')
    # publish_period_sec：/map 地图话题的发布周期（0.5秒发一次）
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')

    # 启动小车底层 + 激光雷达
    # 定位底层包的 launch 目录
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    # 启动小车底层节点，并传 carto_slam=true，让 EKF 使用 ekf_carto.yaml（建图配置：publish_tf=false、3D模式）
    #    → 因为建图时 TF 由 Cartographer 自己发布，EKF 要让位
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
            launch_arguments={'carto_slam': 'true'}.items(),
    )
    # 启动激光雷达传感器，获取 /scan 激光雷达话题数据（建图的核心输入）
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )

    # 核心建图节点：cartographer_node
    #    - 订阅 /scan（激光）、/odom_raw（里程计）、/imu/stm32_raw（IMU）
    #    - 用这些数据实时拼接地图，输出内部子图
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        # 设置节点的 ROS 参数，被节点的参数服务器管理并取出配置节点参数使用，因此可以在运行时动态修改
        parameters=[{'use_sim_time': use_sim_time}],   
        # C++程序命令行参数（argv），加载cartographer_node节点的lua配置
        arguments=[
            '-configuration_directory', cartographer_config_dir,   # 配置目录
            '-configuration_basename', configuration_basename],   # lua配置名
        # 话题重映射——把节点内部想订阅的话题名，改成系统里实际存在的话题名
        # 订阅原始里程计话题数据/odom_raw和原始IMU话题数据/imu/stm32_raw，不使用EKF融合后的里程计/odom_ekfcombined，不使用去零点漂移的IMU数据/imu/imu_filtered
        # catographer建图算法采用原始odom和imu数据，自带数据处理方法，配合/scan数据进行扫描纠正，融合更加精确
        remappings=[('odom','/odom_raw'), ('imu','/imu/stm32_raw')]) 

    # 地图栅格发布节点：cartographer_occupancy_grid_node
    #    作用：把 Cartographer 内部建好的子图，转换成 ROS 标准的
    #          nav_msgs/OccupancyGrid 地图，发布到 /map 话题
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    # 组装 LaunchDescription，包括声明的相关参数和启动小车底盘节点、激光雷达节点、catographer建图节点、catographer建图话题发布节点
    return LaunchDescription([
        # 先进行声明上面用到的 4 个可覆盖参数，以便后面其他启动节点使用时可以不会报错未声明定义
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        wheeltec_robot,   
        wheeltec_lidar,   
        cartographer_node,
        cartographer_occupancy_grid_node
    ])
