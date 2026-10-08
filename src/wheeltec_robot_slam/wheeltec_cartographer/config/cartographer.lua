-- ==========================================================================
-- cartographer.lua —— Cartographer 2D 建图的【核心配置】
--
-- 作用：定义建图算法的所有参数（坐标系、传感器、扫描匹配、回环检测、优化）
-- 被谁用：cartographer_node 启动时通过 arguments 加载
-- 结构：
--   include 两个子配置（map_builder 和 trajectory_builder）
--   options 表：全局选项（坐标系、传感器开关）
--   TRAJECTORY_BUILDER_2D：2D轨迹构建器（局部建图/扫描匹配）
--   POSE_GRAPH：位姿图（全局优化/回环检测）
--  建图过程：
--  → 激光帧到达
--  → 体素滤波降采样（voxel_filter_size=0.05）
--  → 在线扫描匹配（和当前子图比对，ceres精修）
--  → 攒够45帧生成一个子图（submaps.num_range_data=45）
--  → 走到曾经来过的地方 → 回环检测（min_score=0.65）
--  → 每50个节点全局优化一次（optimize_every_n_nodes=50）
--  → 修正累积漂移

-- ==========================================================================

-- 引入两个基础配置文件（定义 MAP_BUILDER 和 TRAJECTORY_BUILDER 结构）
include "map_builder.lua"        -- 地图构建器定义
include "trajectory_builder.lua" -- 轨迹构建器定义

-- ==========================================================================
-- options：Cartographer 的全局选项
-- ==========================================================================
options = {
    map_builder = MAP_BUILDER,          -- 用上面 include 的地图构建器
    trajectory_builder = TRAJECTORY_BUILDER,  -- 轨迹构建器

    -- ★★★ 坐标系设置（和你的 TF 对应）★★★
    map_frame = "map",                  -- 地图坐标系（建出的地图所在的系），默认会发布map→odom_cartocombined的 TF
    tracking_frame = "base_footprint",  -- 跟踪坐标系（Cartographer 跟踪机器人的系，就是机器人本体）
    published_frame = "base_footprint", -- 发布坐标系（对外发布的机器人位姿所在系）
    -- 不需要和EKF一样，接收的/odom_raw消息的父坐标系和odom_frame可以不一致，因为Cartographer不需要将/odom_raw消息转化到odom_cartocombined坐标系，而是使用车体增量进行预测修正
    odom_frame = "odom_cartocombined",  -- 里程计坐标系（Cartographer 认为的起点系，解决EKF的同名TF冲突）
    provide_odom_frame = true,          -- 发布自己提供的 odom_cartocombined→base_footprint 的 TF，false不会发布
    publish_frame_projected_to_2d = true,  -- 把位姿投影到2D平面再发布（丢z/roll/pitch）

    -- ★★★ 传感器开关 ★★★
    use_odometry = true,        -- 是否用里程计（/odom_raw，作运动预测）
    use_nav_sat = false,        -- 是否用GPS（否）
    use_landmarks = false,      -- 是否用路标（否）

    -- ★★★ 传感器数量 ★★★
    num_laser_scans = 1,              -- 用 1 路 2D 激光（/scan）
    num_multi_echo_laser_scans = 0,   -- 不用多回波激光
    num_subdivisions_per_laser_scan = 1, -- 每帧激光分1段
    num_point_clouds = 0,             -- 不用点云输入

    -- ★★★ 发布周期（多久发一次）★★★
    lookup_transform_timeout_sec = 0.2,     -- 查TF超时
    submap_publish_period_sec = 0.3,        -- 子图发布周期（0.3秒）
    pose_publish_period_sec = 5e-3,         -- 位姿发布周期（5ms，很快）
    trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期（30ms）

    -- ★★★ 采样比例（每N帧数据取1帧）★★★
    rangefinder_sampling_ratio = 1.,  -- 激光：全用
    odometry_sampling_ratio = 0.1,    -- 里程计：每10帧取1帧
    fixed_frame_pose_sampling_ratio = 1., -- 固定帧位姿：全用
    imu_sampling_ratio = 1.,          -- IMU：全用
    landmarks_sampling_ratio = 1.,    -- 路标：全用
}

-- 使用 2D 轨迹构建器（2D 建图）
MAP_BUILDER.use_trajectory_builder_2d = true

-- ==========================================================================
-- TRAJECTORY_BUILDER_2D：局部建图（扫描匹配 + 子图）
-- ==========================================================================
TRAJECTORY_BUILDER_2D.min_range = 0.15    -- 激光最小距离（<0.15m 的点丢弃，盲区）
TRAJECTORY_BUILDER_2D.max_range = 20.0    -- 激光最大距离（>20m 的点丢弃，超量程）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.  -- 没测到障碍时按3米补
TRAJECTORY_BUILDER_2D.use_imu_data = true -- 用IMU（估算重力方向/零偏，辅助扫描匹配）

-- 使用"在线相关扫描匹配"：先用暴力匹配粗对齐，再用ceres精修
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 运动滤波器：角度变化 < 0.1° 就跳过这帧（省算力，静止时不用反复匹配）
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- Ceres 扫描匹配（精修）：平移权重 200，迭代最多 20 次
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

-- 累计多少帧点云做一次匹配（1=每帧都匹配，更实时）
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 体素滤波：把点云按5cm网格降采样（减少点数，加速）
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- 每个子图包含 45 帧激光（攒够45帧生成一个子图）
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- 后台线程数（建图计算用4个线程并行）
MAP_BUILDER.num_background_threads = 4

-- ==========================================================================
-- POSE_GRAPH：全局优化（回环检测 + 位姿图优化）
-- ==========================================================================
-- 约束匹配的最低分数（低于0.65认为不是同一个地方）
POSE_GRAPH.constraint_builder.min_score = 0.65

-- 全局定位的最低分数（重定位时用）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 全局采样比例（随机采样建约束，0.001=稀疏采样）
POSE_GRAPH.global_sampling_ratio = 0.001

-- 约束采样比例
POSE_GRAPH.constraint_builder.sampling_ratio = 0.001

-- 每 50 个节点做一次全局优化（回环检测后优化整个位姿图）
POSE_GRAPH.optimize_every_n_nodes = 50

-- 返回配置给 Cartographer 使用
return options