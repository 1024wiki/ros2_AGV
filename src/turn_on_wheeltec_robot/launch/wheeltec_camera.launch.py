# ============================================================================
#  文件 : wheeltec_camera.launch.py
#  功能 : 启动相机传感器
#
#  这个文件做什么？
#    根据 wheeltec_param.yaml 里配置的 camera_mode（相机型号），
#    自动选择对应的【相机驱动包】（astra_camera 或 usb_cam），
#    启动相机节点发布图像话题，并额外启动"图像压缩"节点。
#
#  支持的相机型号（在 wheeltec_param.yaml 的 camera_mode 里配置）：
#    astra / astra_pro / dabai_u3 / gemini   → astra_camera 包（奥比中光深度相机系列）
#    usb                                    → usb_cam 包（普通 USB 摄像头）
#
#  ★ 和雷达（wheeltec_lidar.launch.py）一样：OpaqueFunction 运行时动态选择
# ============================================================================

import os, yaml
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# -------------------------------------------------
# 工具函数：读取 yaml 配置文件
# -------------------------------------------------
def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


# -------------------------------------------------
# 核心函数：根据 camera_mode 动态生成相机节点（OpaqueFunction 运行时调用）
# -------------------------------------------------
def spawn_camera_nodes(context, *args, **kwargs):
    # ---- 第 1 步：读取参数文件 ----
    # camera_modes_yaml 默认指向 wheeltec_param.yaml（含 camera_mode 配置）
    param_file = LaunchConfiguration('camera_modes_yaml').perform(context)
    cfg = load_yaml(Path(param_file))

    # ---- 第 2 步：取 camera_mode（相机型号）----
    # 优先命令行传入，否则用 yaml 默认值
    camera_mode_ = LaunchConfiguration('camera_mode').perform(context) or cfg['camera_mode']
    print(f'camera_mode:{camera_mode_}')
    # 相机的具体 launch 文件名 = "型号.launch.xml"（如 astra.launch.xml）
    file_name = f'{camera_mode_}.launch.xml'

    # ---- 第 3 步：构造相机节点 ----
    # 两个可能的相机驱动包目录
    astra_dir = get_package_share_directory('astra_camera')
    astra_launch_dir = os.path.join(astra_dir, 'launch')

    usbcam_dir = get_package_share_directory('usb_cam')
    usbcam_launch_dir = os.path.join(usbcam_dir, 'launch')

    # USB 相机的设备参数（默认 /dev/video0）
    usbcam_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='video device serial number.'
    )

    # 分支1：奥比中光深度相机系列（astra/astra_pro/dabai_u3/gemini）
    #   用 astra_camera 包，直接加载对应型号的 launch 文件
    #   不需要配置相机传感器插入被识别的dev文件，内部会直接通过 USB 枚举找设备，用 vendor_id（厂商ID）+ product_id（产品ID）自动识别
    if camera_mode_.startswith('astra') or camera_mode_.startswith('dabai') or camera_mode_.startswith('gemini'):
        camera_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(astra_launch_dir, file_name)),
            launch_arguments=[('enable_d2c_viewer', 'True')]   # 开启 深度-彩色 对齐查看
        )
    # 分支2：普通 USB 摄像头
    #   用 usb_cam 包，指定视频设备 /dev/RgbCam ，根据wheeltec_udev.sh将相机devX映射为RgbCam
    elif camera_mode_.startswith('usb'):
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usbcam_launch_dir, 'demo.launch.py')),
            launch_arguments={'video_device': '/dev/RgbCam'}.items()
        )

    # 收集要启动的节点/子launch
    actions = []
    actions.append(camera_launch)   # ① 相机驱动本身

    # ---- 第 4 步：额外启动"图像压缩"节点 ----
    # 为什么需要压缩？原始图像每帧几 MB，网络传输很慢/很占带宽。
    # 压缩成 jpeg 后（如 85 质量）体积小几十倍，适合远程查看。
    # image_transport 的 republish 节点：把 raw（原始）图像转成 compressed（压缩）图像

    # ④a 彩色图像压缩
    compress_color = Node(
        package='image_transport',
        executable='republish',           # "重新发布"工具：订阅一种格式，转成另一种格式再发
        name='republish_color_compressed',
        arguments=['raw', 'compressed'],  # 从 raw（原始）转成 compressed（jpeg压缩）
        remappings=[
            ('in', '/camera/color/image_raw'),          # 输入：彩色原图话题订阅同时会被输出话题名：/camera/color/image_raw
            ('out/compressed', '/camera/color/image_raw/compressed')  # 输出：压缩后彩色图话题名：/camera/color/image_raw/compressed
        ],
        parameters=[{
            'format': 'jpeg',       # 压缩格式
            'jpeg_quality': 85      # 压缩质量 0~100，越大越清晰但带宽占用越高
        }]
    )

    # ④b 深度图像压缩（深度相机才有 /camera/depth/image_raw）
    compress_depth = Node(
        package='image_transport',
        executable='republish',
        name='republish_depth_compressed',
        arguments=['raw', 'compressedDepth'],   # 深度图用压缩Depth格式（保留深度值精度）
        remappings=[
            ('in', '/camera/depth/image_raw'),           # 输入：深度原图话题订阅同时会被输出话题名：/camera/depth/image_raw
            ('out/compressedDepth', '/camera/depth/image_raw/compressed')  # 输出：压缩深度图话题名：/camera/depth/image_raw/compressed
        ],
        output='screen'
    )

    actions.append(compress_color)   # ② 彩色压缩节点
    actions.append(compress_depth)   # ③ 深度压缩节点
    return actions   # 返回所有要启动的节点


# -------------------------------------------------
# 主入口（ROS2 launch 约定函数名）
# -------------------------------------------------
def generate_launch_description():
    return LaunchDescription([
        # 声明可被外部覆盖的 launch 参数
        # camera_modes_yaml：相机配置文件（默认 wheeltec_param.yaml）
        DeclareLaunchArgument(
            'camera_modes_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot'),
                'config', 'wheeltec_param.yaml'),
            description='Path to camera_modes.yaml'),
        # camera_mode：相机型号（运行时可覆盖，空=用 yaml 默认）
        DeclareLaunchArgument(
            'camera_mode',
            default_value='',
            description='Which camera mode to launch'),
        # 用 OpaqueFunction 在运行时动态生成相机节点
        OpaqueFunction(function=spawn_camera_nodes),
    ])
