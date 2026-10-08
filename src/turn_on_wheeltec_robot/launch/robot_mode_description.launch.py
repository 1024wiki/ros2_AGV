# ============================================================================
#  文件 : robot_mode_description.launch.py
#  功能 : 启动机器人的"模型描述 + 静态坐标变换（TF）"
#
#  这个文件做 4 件事：
#    1. robot_state_publisher  ：加载 URDF 模型，发布机器人各部件间的 TF
#    2. base→laser  ：发布"底盘→激光雷达"的静态 TF
#    3. base→camera ：发布"底盘→相机"的静态 TF
#    4. base→link / base→gyro / base→radar：发布其他传感器的静态 TF
#
#  为什么需要它？
#    上层（导航/建图/RViz）要知道"激光在哪、相机在哪、IMU在哪"，
#    这些传感器相对于机器人本体的位置是【固定不变的】，
#    所以用"静态变换发布器"(static_transform_publisher)发布一次即可。
#
#  ★ 关键设计：用 OpaqueFunction + 读 yaml 动态生成节点
#    因为"不同的车型(如 mini_mec / top_akm_dl) 传感器安装位置不同"，
#    所以先从 robot_model.yaml 读取当前车型的传感器坐标，
#    再运行时动态生成对应的静态变换节点。
# ============================================================================

import os,yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterFile

# -------------------------------------------------
# 工具函数：读取 yaml 配置文件
# -------------------------------------------------
def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


# -------------------------------------------------
# 真正构造节点的函数（由 OpaqueFunction 在运行时调用）
# -------------------------------------------------
def spawn_robot_nodes(context, *args, **kwargs):
    """真正构造节点的函数，由 OpaqueFunction 调用"""

    # ---- 第 1 步：取得两个配置文件的实际路径 ----
    # LaunchConfiguration('xxx').perform(context)：在运行时取出参数值
    # wheeltec_param_yaml  → 主配置（含 car_mode 车型）
    # robot_model_yaml     → 模型配置（含各车型的传感器安装坐标）
    param_file = LaunchConfiguration('wheeltec_param_yaml').perform(context)
    robot_model_file = LaunchConfiguration('robot_model_yaml').perform(context)
    cfg = load_yaml(Path(param_file))
    model_cfg = load_yaml(Path(robot_model_file))

    # ---- 第 2 步：取出车型，并校验 ----
    # car_mode：如 mini_mec / top_akm_dl 等（运行时没传就用 yaml 里的默认值）
    car_mode = LaunchConfiguration('car_mode').perform(context) or cfg['car_mode']
    print(f'car_mode:{car_mode}')
    # 检查这个车型在 robot_model.yaml 里是否存在，不存在直接报错（防止配置写错）
    if car_mode not in model_cfg['robot_model']:
        raise ValueError(f'Unknown car_mode "{car_mode}" in {param_file}')

    # 取出当前车型的传感器安装坐标（如 base_to_laser 的 xyz+rpy）
    model_cfg = model_cfg['robot_model'][car_mode]

    # ---- 第 3 步：拼出当前车型的 URDF 模型文件路径 ----
    # URDF（机器人模型描述文件）在 wheeltec_robot_urdf 包里，按车型命名
    urdf_path = os.path.join(
        get_package_share_directory('wheeltec_robot_urdf'),
        'urdf',
        f'{car_mode}_robot.urdf'
    )

    # ---- 第 4 步：组装要启动的所有节点 ----
    actions = [
        # ------------------------------------------------
        # ① robot_state_publisher：发布机器人模型 TF
        # ------------------------------------------------
        # 作用：读入 URDF 模型文件，持续发布模型里所有"关节joint"的 TF，"实物link"不会被robot_state_publisher进行发布。
        #       对于continue类型的joint会读取joint_state_publisher发布的 /joint_states 话题结合joint的origin进行动态发布TF的位置和姿态角
        #       对于fix类型的joint会直接根据joint的origin进行静态发布TF，因为父子link间的角度是固定的。
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path],   # 传入 URDF 文件路径作为参数
        ),

        # ------------------------------------------------
        # ② base_footprint → base_link：发布小车底盘投影base_footprintf（里程计和导航使用的坐标系）到 URDF文件的base_link(小车本体中心参考点) 的静态 TF
        # ------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_link',
            arguments=[*map(str, model_cfg['base_to_link']), 'base_footprint', 'base_link'],
        ),

        # ------------------------------------------------
        # ③ base_footprint → 各类传感器的坐标位置，可以考虑使用base_footprint->base_link，
        #     然后叠加urdf中被robot_state_publisher发布的base_link到各类传感器的joint的静态TF，由TF系统叠加实现
        #     不需要再次发布base_footprintf->各类传感器坐标位置，需要保证前面坐标数据的正确性，为实际真实距离。
        
        # base → laser：静态发布底盘到激光雷达的静态 TF 
        # 数据来自 robot_model.yaml 里的 base_to_laser（不同车型位置不同）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',    # static_transform_publisher：发布一个"固定不变"的坐标变换
            name='base_to_laser',
            arguments=[*map(str, model_cfg['base_to_laser']), 'base_footprint', 'laser'], # 参数：x y z roll pitch yaw 父坐标系 子坐标系
        ),

        # base → camera：静态发布底盘到相机的静态 TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[*map(str, model_cfg['base_to_camera']), 'base_footprint', 'camera_link'],
        ),

        # base → gyro：静态发布底盘到 IMU（陀螺仪）的静态 TF ：
        # 注意：gyro_link 必须和 wheeltec_robot.cpp 里 Publish_ImuSensor() 的 frame_id 一致，否则 EKF 报 TF 错误
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gyro',
            arguments=[*map(str, model_cfg['base_to_gyro']), 'base_footprint', 'gyro_link'],
        ),

        # base → radar：静态发布底盘到雷达的静态 TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_radar',
            arguments=[*map(str, model_cfg['base_to_radar']), 'base_footprint', 'radar'],
        ),
    ]

    return actions   # 返回要启动的所有节点


# -------------------------------------------------
# 主入口（ROS2 launch 约定函数名）
# -------------------------------------------------
def generate_launch_description():
    #   joint_state_publisher 节点：
    #    作用：调用ROS官方的 joint_state_publisher 包，发布机器人关节状态话题 /joint_states，
    #          配合 URDF 模型在 RViz 里显示各关节（如云台、轮子）姿态。
    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher',        # 所属功能包
            executable='joint_state_publisher',     # 要运行的可执行程序
            name='joint_state_publisher',           # 节点名称（决定话题命名空间）
    )

    return LaunchDescription([
        # 关节状态结点
        joint_state_publisher_node,

        # 声明可被外部覆盖的 launch 参数
        # wheeltec_param_yaml：主配置文件路径（含车型等）
        DeclareLaunchArgument(
            'wheeltec_param_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot'),
                'config', 'wheeltec_param.yaml'),
            description='Path to wheeltec_param.yaml'
        ),
        # robot_model_yaml：模型配置文件路径（含各车型传感器坐标）
        DeclareLaunchArgument(
            'robot_model_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot'),
                'config', 'robot_model.yaml'),
            description='Path to robot_model.yaml'
        ),
        # car_mode：车型，运行时可覆盖（空字符串=用 yaml 里的默认值）
        DeclareLaunchArgument(
            'car_mode',
            default_value='',
            description='Which robot mode to launch (mini_akm, mini_mec...)'
        ),

        # 用 OpaqueFunction 在运行时动态生成节点 
        # 普通 launch 是"静态"的；这里用 OpaqueFunction 延迟到运行时，以便先读 yaml、再根据车型动态生成节点列表
        OpaqueFunction(function=spawn_robot_nodes)
    ])

