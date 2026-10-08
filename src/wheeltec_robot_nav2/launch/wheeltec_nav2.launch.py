# ============================================================================
#  文件 : wheeltec_nav2.launch.py（wheeltec_robot_nav2 包）
#  功能 : 启动 NAV2 自主导航（主入口）
#
#  导航 = 用已建好的地图 + 当前定位（我在哪）+ 目标点（去哪），让小车自主规划路径并走过去。
#
#  ────────────────────────────────────────────────────────────────
#  nav2导航启动命令ros2 launch wheeltec_robot_nav2 wheeltec_nav2.launch.py map:=~/map/my_map.yaml，启动后会打开以下4个结点文件：
#    ① 启动 waypoint_cycle（多点巡航节点）
#       在 RViz 里点多个目标点（航点），它就按顺序发布导航目标，让小车自动走完所有点并循环（巡检/巡航用）。
#       a)接收RViz的/clicked_pose话题在map上点击的导航目标，并通过/path_pose话题在map上发布可视化的导航目标点序号和路径，RViz通过订阅该话题在map上可视化显示。
#       b)通过订阅bt_navigator导航目标完成状态的/navigate_to_pose/_action/status话题，来获取上个导航目标点的完成状态。
#         在成功时：执行到指定目标导航点的业务逻辑，包括采用相机传感器进行拍照等。然后通过/goal_pose话题向bt_navigator发布下一个导航目标，当终点导航目标被执行后，回到起点导航目标。
#         在失败时：尝试再次重复进行导航，再次导航失败时，通过/goal_pose话题向bt_navigator发布下一个导航目标，当终点导航目标被执行后，回到起点导航目标。
#
#    ② 启动小车底层（turn_on_wheeltec_robot.launch.py）→ 底盘驱动结点 + EKF结点+ 模型/TF
#        a) 底盘驱动结点（wheeltec_robot_node）
#           发布：/odom_raw（原始里程计）、/imu/stm32_raw（原始IMU）、订阅：/cmd_vel（速度指令）
#        b) 去零点漂移结点（imu_processor）
#           订阅：/imu/stm32_raw，发布：/imu/imu_filtered（去漂移IMU）
#        c) EKF 位姿融合结点（ekf_filter_node，用 ekf_nav.yaml）
#           订阅：/odom_raw + /imu/imu_filtered、发布：/odom_ekfcombined（融合位姿）+ odom_ekfcombined→base TF
#        d) 模型/TF 结点（robot_state_publisher + joint_state_publisher + static_transform）
#    ③ 启动激光雷达（wheeltec_lidar.launch.py）→ 提供 /scan 激光数据
#    ④ 启动 NAV2 核心（nav2官方的bringup_launch.py）→ AMCL 定位 + 全局规划 + 局部控制 + 行为树
#           bringup_launch.py会启动localization_launch.py定位模块、navigation_launch.py导航模块、根据需要可以配置启动slam_launch.py建图模块
#           localization_launch.py定位模块：
#                   map_server进行加载地图yaml文件，并发布/map话题
#                   AMCL进行定位所在地图位置，并去除里程计漂移误差，发布map->odom的TF变换
#           navigation_launch.py导航模块：
#                   bt_navigator订阅/goal_pose话题获取导航目标，并根据导航行为树决策下步导航动作，先进行全局规划、控制机器人运动、卡住进行恢复并重新规划
#                   planner_server进行全局路径规划，根据A*算法规划一条当前位置到目标点的无障碍物路径，并发布到/plan话题中
#                   controller_server进行局部控制机器人按照全局路径运动，发布/cmd_vel话题数据
#                   behavior_server实现出现运动卡住等异常时进行恢复
#                           
#  ────────────────────────────────────────────────────────────────    
#  nav2导航基本过程原理：
#  第 1 步：加载地图 + 定位
#   1) map_server 加载地图 yaml 文件 → 发布 /map 话题（栅格地图）
#   2) AMCL 订阅 /scan（激光）+ /odom_ekfcombined（EKF里程计），用类似gmapping建图的粒子滤波算法，
#       通过采用大量粒子结合/odom_ekfcombined里程计数据进行预测机器人位置，并结合/scan数据匹配，增加匹配度高的粒子数量，淘汰匹配度低的粒子，
#       最后粒子进行收敛确定机器人所在位置，并根据激光数据匹配在 map 中定位机器人位置。发布 map→odom_ekfcombined 的 TF 变换。
#       这个过程在导航过程会持续进行，每次接收/scan数据时进行，一边进行定位map位置，并修正里程计漂移误差。
#
#  第 2 步：RViz 点击目标 → 开始导航
#   用户在 RViz 点"2D Nav Goal" → 发布目标到 /goal_pose 话题，之后按以下 4 个子流程执行：
#     (1) bt_navigator：行为树决策（导航大脑）
#         bt_navigator 会接收到 /goal_pose话题数据的导航目标，然后开始进行导航。
#         根据 XML 行为树决定下一步动作："先全局规划 → 再控制运动 → 卡住了就恢复并重新规划"
#         
#     (2) planner_server：全局路径规划（地图师）
#         用 A* 算法结合全局代价地图，从当前定位位置到目标点规划一条【无障碍】路径 → 发布到 /plan 话题（全局路径）
#
#     (3) controller_server：局部控制（司机）
#         结合 /scan 数据和局部代价地图（实时避障）和/odom_ekfcombined数据确定位置，控制机器人沿全局路径/plan运动 → 发布 /cmd_vel 速度指令
#         → 底盘驱动收到 /cmd_vel → 下位机 → 电机转动
#
#     (4) behavior_server：恢复（救援队）
#         当机器人运动卡住 / 规划失败时，执行恢复动作（旋转、清除代价地图等）→ 然后 bt_navigator 重新规划
#         
#  ────────────────────────────────────────────────────────────────
#  【数据流总结】
#   输入：/map（地图）、/scan（激光）、/odom_ekfcombined（里程计）、/goal_pose（目标）
#   处理：AMCL定位 → 全局规划 → 局部控制 → 行为树决策
#   输出：/cmd_vel（速度）→ 底盘驱动 → 电机
#   坐标：map → odom_ekfcombined（AMCL发）→ base_footprint（EKF发）
#    
# ============================================================================

import os,yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 工具函数：读取 yaml 配置文件
def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # use_sim_time：是否用仿真时间（Gazebo 用 true，真机用 false）
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 定位各包的 launch 目录
    # ------------------------------------------------------------
    # 底层包（turn_on_wheeltec_robot）的 launch 目录
    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')

    # 本导航包（wheeltec_nav2）的 launch 目录
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_nav_launchr = os.path.join(wheeltec_nav_dir, 'launch')


    # 读车型配置，确定地图和参数文件
    # ------------------------------------------------------------
    # 读 wheeltec_param.yaml 里的 car_mode（车型，如 four_wheel_diff_bs）
    cfg_params = load_yaml(os.path.join(get_package_share_directory('turn_on_wheeltec_robot'),'config','wheeltec_param.yaml'))
    car_mode = cfg_params['car_mode']
    print(f"car_mode:{car_mode}")

    # 地图文件（默认用包自带的 WHEELTEC.yaml，运行时可覆盖 map:=）
    map_dir = os.path.join(wheeltec_nav_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(map_dir, 'WHEELTEC.yaml'))

    # 导航参数文件（按车型选择 param_<车型>.yaml）
    param_dir = os.path.join(wheeltec_nav_dir, 'param','wheeltec_params')
    param_file = LaunchConfiguration('params', default=os.path.join(param_dir, f'param_{car_mode}.yaml'))
    print(os.path.join(param_dir, f'param_{car_mode}.yaml'))


    # 组装要启动的节点/子launch
    # ------------------------------------------------------------
    # ① 多点巡航节点（waypoint 循环导航）
    # 在 RViz 里点几个目标点（航点），它就按顺序一个一个地发布导航目标，让小车自动走完所有点，然后循环（巡航）
    waypoint_cycle = Node(
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
            name='waypoint_cycle',
        )

    # ② 小车底层启动
    #    注意：carto_slam=false, robot_nav=true → EKF 用导航配置（ekf_nav.yaml，EKF 自己发 odom→base TF）
    turn_on_robot =IncludeLaunchDescription(
            PythonLaunchDescriptionSource([wheeltec_launch_dir, '/turn_on_wheeltec_robot.launch.py']),
            launch_arguments={'carto_slam': 'false','robot_nav':'true'}.items(),
        )

    # ③ 激光雷达启动（提供 /scan 话题数据，导航定位和避障用）
    turn_on_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([wheeltec_launch_dir, '/wheeltec_lidar.launch.py']),
        )

    # ④ NAV2 核心启动（最关键：AMCL 定位 + 规划 + 控制）
    #    bringup_launch.py 会启动：
    #      - map_server（加载地图）
    #      - AMCL（定位：我在哪）
    #      - planner_server（全局路径规划）
    #      - controller_server（局部控制/路径跟随）
    #      - bt_navigator（行为树导航决策）
    #   订阅数据包括：/odom_ekfcombined(EKF里程计融合数据)、/scan激光雷达话题数据
    #   发表数据包括：/cmd_vel 机器人运动速度
    nav2_core = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([wheeltec_nav_launchr, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,          # 地图文件
                'use_sim_time': use_sim_time,  # 是否仿真时间
                'params_file': param_file}.items(),  # 导航参数文件
        )


    # 组装 LaunchDescription
    # ------------------------------------------------------------
    return LaunchDescription([
        # 声明两个可覆盖参数（map / params）
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),

        # 导航相关结点启动
        waypoint_cycle,   # ① 多点巡航
        turn_on_robot,    # ② 底层
        turn_on_lidar,    # ③ 雷达
        nav2_core,        # ④ NAV2 核心（定位+规划+控制）
    ])
