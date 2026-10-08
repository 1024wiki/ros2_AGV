# ============================================================================
#  文件 : wheeltec_lidar.launch.py
#  功能 : 启动激光雷达传感器
#
#  这个文件做什么？
#    根据 wheeltec_param.yaml 里配置的 lidar_type（激光雷达型号），
#    自动选择对应的【雷达驱动包】，并启动雷达节点发布 /scan 话题。
#
#  支持的雷达型号（在 wheeltec_param.yaml 的 lidar_type 里配置）：
#    ls_M10_uart / ls_M10_net / ls_M10P_uart / ls_M10P_net   → lslidar_driver 包（LS系列）
#    ls_N10_uart / ls_N10_net / ls_N10Plus_uart / ls_N10Plus_net → lslidar_driver 包（LS系列）
#    ldstl19p / ldstl06nbj / ldstl19n   → ldlidar 包
#    rplidar_c1                        → rplidar_ros 包
#
#  ★ 关键设计：OpaqueFunction 运行时动态选择
#    因为"不同型号的雷达用不同驱动包"，所以先用 OpaqueFunction 读配置、
#    根据 lidar_type 动态决定启动哪个雷达节点，而不是写死。
# ============================================================================

import os
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    GroupAction, OpaqueFunction
)
from launch_ros.actions import LifecycleNode,Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


# -------------------------------------------------
# 工具函数：读取 yaml 配置文件
# -------------------------------------------------
def load_yaml(path: Path) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


# -------------------------------------------------
# 核心函数：根据 lidar_type 动态生成雷达节点（OpaqueFunction 运行时调用）
# -------------------------------------------------
def include_lidar_launch(context, *args, **kwargs):
    # ---- 第 1 步：读取参数文件 ----
    # lidar_type_yaml 默认指向 wheeltec_param.yaml（含 lidar_type 配置）
    yaml_path = LaunchConfiguration('lidar_type_yaml').perform(context)
    cfg = load_yaml(Path(yaml_path))

    # ---- 第 2 步：取 lidar_type（雷达型号）----
    # 优先用命令行传入的 lidar_type，否则用 yaml 里的默认值
    lidar_type = LaunchConfiguration('lidar_type').perform(context) or cfg['lidar_type']
    print(f'lidar_type:{lidar_type}')

    # ---- 第 3 步：根据型号决定启动哪个雷达节点 ----
    actions = []  # 存放要启动的节点/子launch

    # 分支1：LS 系列雷达（lslidar_driver 包）
    if lidar_type.startswith('ls'):            # LS 系列
        launch_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'launch')

        # 1a. 多线雷达 lscx（需要额外启动 点云→激光 转换）
        if lidar_type == 'lscx':
            # 读 lslidar 包自带的配置模板，多线雷达走网口，在lslidar_cx.yaml配置文件配置IP地址和端口
            template_yaml = Path(
                get_package_share_directory('lslidar_driver'),
                'config', 'lslidar_cx.yaml'
                )
            cx_cfg = yaml.safe_load(template_yaml.read_text())['cx']['lslidar_driver_node']['ros__parameters']
            # 如果 wheeltec_param.yaml 里配了裁剪角度，则覆盖到雷达配置里
            if cfg['lscx']['angle_disable_min'] != 0 and cfg['lscx']['angle_disable_max'] != 0 :
                cx_cfg['angle_disable_min'] = cfg['lscx']['angle_disable_min']
                cx_cfg['angle_disable_max'] = cfg['lscx']['angle_disable_max']

            # 用 GroupAction 把两个节点打包一起启动：
            #   ① lslidar_driver_node（雷达驱动，出3D点云）
            #   ② pointcloud_to_laserscan（3D点云转2D激光 /scan）
            lidar_launch = GroupAction(
                actions=[
                    LifecycleNode(package='lslidar_driver',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    namespace='cx', # 与对应yaml文件中命名空间一致
                                    parameters=[cx_cfg],
                                    output='screen'
                                    ),
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('pointcloud_to_laserscan'),
                            'launch', 'pointcloud_to_laserscan_launch.py')),
                        )
                    ]
                )
        # 1b. 单线雷达（ls_M10 / ls_N10 系列，默认走 lslidar_x10.yaml 模板）
        else:
            template_yaml = Path(
                get_package_share_directory('lslidar_driver'),
                'config', 'lslidar_x10.yaml'
                )
            lidar_port = cfg['x10']['lidar_port']   
            # 读取模板配置，作为基础参数
            x10_cfg = yaml.safe_load(template_yaml.read_text())['x10']['lslidar_driver_node']['ros__parameters']

            # 根据型号后缀决定通信方式： 串口设备类型的雷达配置dev设备名lidar_port，这个在wheeltec_param.yaml配置
            #   net（网口）→ 不填串口；uart（串口）→ 填串口设备名
            if lidar_type.endswith('net'):
                x10_cfg['serial_port'] = ''
            elif lidar_type.endswith('uart'):
                x10_cfg['serial_port'] = lidar_port
            # 根据型号前缀设置雷达型号参数（驱动需要知道具体型号）
            if lidar_type.startswith('ls_M10'):
                if lidar_type.startswith('ls_M10P'):
                    x10_cfg['lidar_model'] = 'M10P'
                else:
                    x10_cfg['lidar_model'] = 'M10'
            if lidar_type.startswith('ls_N10'):
                if lidar_type.startswith('ls_N10Plus'):
                    x10_cfg['lidar_model'] = 'N10Plus'
                else:
                    x10_cfg['lidar_model'] = 'N10'

            # 如果配置了裁剪角度，覆盖到雷达配置里
            if cfg['x10']['angle_disable_min'] != 0 and cfg['x10']['angle_disable_max'] != 0 :
                x10_cfg['angle_disable_min'] = cfg['x10']['angle_disable_min']
                x10_cfg['angle_disable_max'] = cfg['x10']['angle_disable_max']

            # 启动雷达驱动节点（LifecycleNode：生命周期节点，启动后需手动激活）
            lidar_launch = LifecycleNode(package='lslidar_driver',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    namespace='x10', # 与对应yaml文件中命名空间一致
                                    parameters=[x10_cfg],
                                    output='screen'
                                    )

    # 分支2：LD 系列雷达（ldlidar 包，直接用现成 launch，该launch文件内部配置雷达传感器的dev）
    elif lidar_type == 'ldstl19p':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ldlidar'), 'launch','stl19p.launch.py')),)
    elif lidar_type == 'ldstl06nbj':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ldlidar'), 'launch','stl06nbj.launch.py')),)
    elif lidar_type == 'ldstl19n':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ldlidar'), 'launch', 'stl19n.launch.py')),)

    # 分支3：RPLIDAR 系列雷达（rplidar_ros 包，需要手动指定参数dev: serial_port:=/dev/rplidar）
    elif lidar_type == 'rplidar_c1':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')),)
    # 其他型号 → 不支持，直接报错
    else:
        raise ValueError(f'Unsupported lidar: {lidar_type}')

    # ---- 第 4 步：把生成的雷达节点加入列表并返回 ----
    actions.append(lidar_launch)
    return actions


# -------------------------------------------------
# 主入口（ROS2 launch 约定函数名）
# -------------------------------------------------
def generate_launch_description():
    return LaunchDescription([
        # 声明可被外部覆盖的 launch 参数
        # lidar_type_yaml：雷达配置文件名（默认 wheeltec_param.yaml）
        DeclareLaunchArgument(
            'lidar_type_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot'),
                'config', 'wheeltec_param.yaml'),
            description='Path to lidar_type.yaml'),
        # lidar_type：雷达型号（运行时可用 lidar_type:=xxx 覆盖，默认空=用yaml）
        DeclareLaunchArgument(
            'lidar_type',
            default_value='',
            description='Which lidar model to launch'),
        # 用 OpaqueFunction 在运行时动态生成雷达节点
        OpaqueFunction(function=include_lidar_launch),
    ])
