# ============================================================================
#  文件 : base_serial.launch.py
#  功能 : 小车底盘的核心启动文件（由 turn_on_wheeltec_robot.launch.py 调用）
#
#  这个文件主要做一件事：
#    启动底盘串口驱动主节点 wheeltec_robot_node，
#    并根据配置文件 wheeltec_param.yaml 中的参数决定：
#      - 用哪种车型（car_mode）
#      - 是否开启超声波避障（ultrasonic_avoid / ranger_avoid_flag）
#      - 用哪种 IMU（imu_mode：stm32 板载 或 H30 外置高精度 IMU）
#
#  小技巧：ROS2 的 launch 文件本质就是一段 Python 脚本，所以它可以用
#    yaml 配置、条件判断、运行时动态生成节点等灵活方式组织启动逻辑。
# ============================================================================

import os
import yaml
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer

from launch_ros.descriptions import ComposableNode


# -------------------------------------------------
# 工具函数：读取 yaml 配置文件，返回 dict
# -------------------------------------------------
def load_yaml(file_path: str) -> dict:
    return yaml.safe_load(Path(file_path).read_text())


# -------------------------------------------------
# 动态生成节点列表的函数（本文件的核心逻辑）
# -------------------------------------------------
# 为什么用 OpaqueFunction？
#   普通 launch 在"解析阶段"就确定了启动内容（静态的）。而 OpaqueFunction 里的函数会在"launch 真正运行时"才被调用，
#   并能拿到 context，因此可以实时读取配置文件 / 参数，动态决定到底启动哪些节点——非常灵活。
def include_imu_launch(context, *args, **kwargs):
    # 1) 读取 imu_mode_yaml 指向的配置文件，得到默认 imu_mode
    #    LaunchConfiguration('xxx').perform(context) 的作用：
    #    在运行时取出 launch 参数 xxx 的实际值（字符串）
    #    imu_mode_yaml默认值的配置文件路径为 turn_on_wheeltec_robot/config/wheeltec_param.yaml
    cfg_path = LaunchConfiguration('imu_mode_yaml').perform(context)
    cfg = load_yaml(cfg_path)
    # 如果运行时没传 imu_mode 参数（默认值 ''），就采用 wheeltec_param.yaml 里配置的读取的cfg['imu_mode']值
    imu_mode = LaunchConfiguration('imu_mode').perform(context) or cfg['imu_mode']
    print(f'using imu_mode:{imu_mode}')

    # 2) 读取 wheeltec_param_yaml（车型、避障等核心配置），默认值为 turn_on_wheeltec_robot/config/wheeltec_param.yaml
    parm_file = LaunchConfiguration('wheeltec_param_yaml').perform(context)
    car_cfg = load_yaml(parm_file)
    # 车型：如 mini_mec / mini_akm / top_akm_dl 等（决定底盘运动学）
    car_mode = LaunchConfiguration('car_mode').perform(context) or car_cfg['car_mode']
    # 是否开启"底盘自身"的超声波避障，默认值为空，则从wheeltec_param.yaml配置文件中获取
    ranger_avoid_flag = LaunchConfiguration('ranger_avoid_flag').perform(context) or car_cfg['ranger_avoid_flag']
    # 是否开启"NAV2 导航时"的超声波避障，从wheeltec_param.yaml配置文件中获取
    ultrasonic_avoid = car_cfg['ultrasonic_avoid']
    print(f'ultrasonic_avoid:{ultrasonic_avoid}')

    # 3) 组装传给 wheeltec_robot_node 的公共参数
    #    这些参数最终会在 wheeltec_robot.cpp 的构造函数里被读取
    common_params = {
        'usart_port_name': '/dev/wheeltec_controller',  # 底盘串口设备名（udev 规则映射）
        'serial_baud_rate': 115200,                     # 串口波特率（须与单片机一致）
        'robot_frame_id': 'base_footprint',             # 机器人本体坐标系名
        'ekf_odom_frame_id': 'odom_ekfcombined',           # EKF里程计坐标系名（EKF融合后的里程计数据）
        'raw_odom_frame_id': 'odom_raw',                # 原始里程计坐标系名（未EKF融合前的原始里程计数据）
        'cmd_vel': 'cmd_vel',                           # 速度指令话题名
        'akm_cmd_vel': 'none',                          # 阿克曼车型转向话题（非阿克曼填 none）
        'product_number': 0,                            # 产品编号
        'odom_x_scale': 1.0,                            # 里程计 X 轴修正系数
        'odom_y_scale': 1.0,                            # 里程计 Y 轴修正系数
        'odom_z_scale_positive': 1.0,                   # 里程计 Z 轴（正转）修正系数
        'odom_z_scale_negative': 1.0,                   # 里程计 Z 轴（反转）修正系数
        'car_mode': car_mode,                           # 车型模式
        'ranger_avoid_flag': ranger_avoid_flag,         # 底盘超声波避障开关
        'ultrasonic_avoid': ultrasonic_avoid            # NAV2 超声波避障开关
    }

    nodes = []  # 最终要启动的"节点 / 子launch"列表
    # 4) 如果开启超声波避障，额外启动超声波位姿节点
    if ultrasonic_avoid:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('turn_on_wheeltec_robot'),
                        'launch',
                        'ultrasonic_pose.launch.py'
                    )
                ])
            )
        )

    # 5) 如果使用板载 STM32 的 IMU：不需要重映射，清空 remappings（不改话题名）
    remappings = []
    if imu_mode == 'H30':
        # 默认话题重映射：imu/data_raw -> imu/data_board
        # 含义：把 wheeltec 主节点发布的 IMU 话题从 imu/data_raw 改名为 imu/data_board
        # 为什么？如果使用外置 H30 高精度 IMU，外置 IMU 要占用 imu/data_raw，
        #         板载 STM32 的 IMU 就让位，改发到 imu/data_board。
        remappings = [('imu/stm32_raw', 'imu/H30_raw')]
        # 外置 H30 高精度 IMU：启动 yesense 惯导节点，
        # 它发布到 imu/data_raw（此时板载stm32 IMU 已被重映射到 imu/data_board）
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('yesense_std_ros2'),
                        'launch',
                        'yesense_node.launch.py'
                    )
                ])
            )
        )
    elif imu_mode == 'stm32':
        remappings = []   # 板载 IMU：不重映射（此分支实际不做额外事）
    else:
        # 其他值都不支持，直接报错阻止启动
        raise ValueError(f'Unsupported imu_mode: {imu_mode}')
        

    # 6) 创建底盘串口驱动主节点（本包最核心的节点！）
    #    对应源码 src/wheeltec_robot.cpp，类名 turn_on_robot
    wheeltec_node = Node(
        package='turn_on_wheeltec_robot',   # 功能包名
        executable='wheeltec_robot_node',   # 可执行程序名（CMakeLists 里 add_executable）
        output='screen',                    # 日志直接打印到终端
        parameters=[common_params],         # 传入参数（构造函数里读取）
        remappings=remappings,              # 话题重映射
    )
    nodes.append(wheeltec_node)
    
    return nodes  # 返回所有要启动的节点/子launch


# -------------------------------------------------
# 主入口（ROS2 launch 约定的入口函数）
# -------------------------------------------------
def generate_launch_description():

    #  声明可覆盖的 launch 参数
    # -----------------------------------------
    # imu_mode_yaml：IMU 配置文件路径（默认指向 config/wheeltec_param.yaml，
    #                注意：这个 yaml 里其实存着 imu_mode 等所有配置）
    declare_imu_path = DeclareLaunchArgument(
        'imu_mode_yaml',
        default_value=os.path.join(
            get_package_share_directory('turn_on_wheeltec_robot'),
            'config', 'wheeltec_param.yaml'
        ),
        description='Path to imu_mode.yaml'
    )
    # imu_mode：IMU 类型，运行时可覆盖，如：
    #   ros2 launch ... base_serial.launch.py imu_mode:=H30
    declare_imu_mode = DeclareLaunchArgument(
        'imu_mode',
        default_value='',   # 默认空 -> 用 wheeltec_param.yaml 里的配置
        description='stm32 or H30'
    )

    # wheeltec_param_yaml：底盘主配置文件路径
    declare_carparam_mode = DeclareLaunchArgument(
        'wheeltec_param_yaml',
        default_value=os.path.join(
            get_package_share_directory('turn_on_wheeltec_robot'),
            'config', 'wheeltec_param.yaml'),
        description='Path to wheeltec_param.yaml'
    )
    # car_mode：车型，运行时可覆盖（如 car_mode:=top_akm_dl）
    declare_car_mode = DeclareLaunchArgument(
        'car_mode',
        default_value='',
        description='Car mode configuration'
    )
    # ranger_avoid_flag：底盘超声波避障开关，运行时可覆盖
    declare_ranger = DeclareLaunchArgument(
        'ranger_avoid_flag',
        default_value='',
        description='Whether to enable ranger avoidance'
    )

    #  组装并返回 LaunchDescription
    #    OpaqueFunction(function=include_imu_launch)：
    #    运行时才执行 include_imu_launch() 动态生成节点列表。
    return LaunchDescription([
        declare_imu_path,
        declare_imu_mode,
        declare_carparam_mode,
        declare_car_mode,
        declare_ranger,
        OpaqueFunction(function=include_imu_launch)
    ])
