# 轮趣（WHEELTEC）ROS2 小车项目技术文档

> 基于 ROS2 Humble + Ubuntu 22.04，覆盖 **底盘驱动 → 传感器 → 位姿融合 → 建图 → 导航** 全链路。
> 本文按模块讲解实现原理与数据流动过程，每个模块配一张数据流图（Mermaid）。

---

## 目录

1. [模块一：底盘串口驱动（base_serial.launch.py）](#模块一底盘串口驱动base_seriallaunchpy)
2. [模块二：IMU 去零点漂移（imu_processor.launch.py）](#模块二imu-去零点漂移imu_processorlaunchpy)
3. [模块三：机器人模型与关节发布（robot_mode_description.launch.py）](#模块三机器人模型与关节发布robot_mode_descriptionlaunchpy)
4. [模块四：EKF 里程计融合（wheeltec_ekf.launch.py）](#模块四ekf-里程计融合wheeltec_ekflaunchpy)
5. [模块五：三种 SLAM 建图方式](#模块五三种-slam-建图方式)
6. [模块六：NAV2 多点导航（wheeltec_nav2.launch.py）](#模块六nav2-多点导航wheeltec_nav2launchpy)

---

## 系统总体架构

```mermaid
flowchart LR
    subgraph 底层[底盘驱动层 turn_on_wheeltec_robot]
        A[wheeltec_robot_node<br/>串口驱动] -->|/odom_raw| E
        A -->|/imu/stm32_raw| B
        A -->|PowerVoltage| V[电压]
    end

    subgraph 处理层[数据预处理与融合]
        B[imu_processor<br/>去零点漂移] -->|/imu/imu_filtered| E
        C[robot_state_publisher<br/>模型TF/关节] --> TF[(TF树)]
        D[joint_state_publisher<br/>关节状态] --> C
        E[EKF<br/>robot_localization] -->|/odom_ekfcombined| F
    end

    subgraph 建图导航层[SLAM / NAV2]
        F --> G[SLAM 建图<br/>gmapping / cartographer / slam_toolbox]
        F --> H[NAV2 导航<br/>AMCL+planner+controller]
        G -->|/map| H
    end

    subgraph 传感层[传感器]
        L[激光雷达<br/>wheeltec_lidar] -->|/scan| G
        L -->|/scan| H
    end

    H -->|/cmd_vel| A
```

**数据流一句话**：底盘(轮子/IMU/电压) → 串口 → 里程计+IMU 话题 → 去漂移/EKF 融合 → 建图/导航 → 又回到底盘执行 `/cmd_vel`，形成闭环。

---

## 模块一：底盘串口驱动（base_serial.launch.py）

### 1.1 模块功能

`base_serial.launch.py` 是**整个项目的起点**。它启动底盘驱动主节点 `wheeltec_robot_node`，负责：

- 通过**串口**（`/dev/wheeltec_controller`，波特率 115200）和底盘单片机（STM32）通信；
- **接收**下位机上报的：轮速、IMU 六轴原始数据、电池电压（可选：充电/超声波数据）；
- **发送**上位机的速度指令（`/cmd_vel`）给下位机，驱动电机；
- **发布** `/odom_raw`（原始里程计）、`/imu/stm32_raw`（原始 IMU）、`/PowerVoltage`（电压）等话题。

### 1.2 实现原理：三帧状态机串口解析

下位机以固定格式（帧头 + 数据 + 校验 + 帧尾）持续上报数据，共 **3 种帧**：

| 帧类型 | 帧头 | 帧尾 | 长度 | 内容 |
|--------|------|------|------|------|
| 基础底盘数据帧 | `0x7B` | `0x7D` | 24 字节 | 轮速 X/Y/Z、IMU 六轴、电池电压 |
| 自动回充数据帧 | `0x7C` | `0x7F` | 8 字节 | 充电电流、红外信号、充电状态 |
| 超声波数据帧 | `0xFA` | `0xFC` | 19 字节 | 6 路超声波距离 A~F |

核心函数 `Get_Sensor_Data_New()` 采用**单字节状态机**解析：

1. **每次调用只从串口读 1 字节**，存入三个接收数组；
2. 用三个计数器（`count/count2/count3`）分别"攒"三种帧；
3. 判断"上一字节是帧尾 && 当前字节是帧头" → 新帧开始，计数从 1 重数；
4. 攒够一帧长度后，先**校验帧尾**，再对前 N 字节做 **BCC 异或校验**，校验通过才解析数据；
5. 只有完整收到一帧底盘数据才返回 `true`，主循环据此决定是否发布话题。

**为什么这样设计？** 串口是字节流，没有明确的"消息边界"，必须靠帧头/帧尾 + 长度 + 校验来切分帧，并丢弃被干扰的脏数据，防止污染里程计。

### 1.3 主循环 Control()：读串口 + 发话题 + 处理回调

```
while(rclcpp::ok())
  ├─ ① Get_Sensor_Data_New()：读1字节，攒帧、解析
  ├─ ② 有完整一帧？→ Publish_Odom() + Publish_ImuSensor() + Publish_Voltage()
  ├─ ③ 有回充帧？→ Publish_Charging/RED/ChargingCurrent
  ├─ ④ 有超声波帧？→ Publish_distance()
  └─ ⑤ rclcpp::spin_some()：非阻塞处理 /cmd_vel 等订阅回调
```

**关键设计**：用 `spin_some()` 而不是 `spin()`——`spin()` 会阻塞在回调等待上，导致串口无人读取而丢数据；`spin_some()` 处理完已排队的回调立即返回，主循环能继续读串口，两不耽误。

### 1.4 里程计积分（Publish_Odom）

里程计位置通过对速度做时间积分得到（差速小车运动学正解）：

$$
\begin{bmatrix} X' \\ Y' \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} Vx \\ Vy \end{bmatrix}
$$

$$
X_{t+1} = X_t + (Vx\cos\theta - Vy\sin\theta)\cdot dt, \quad
Y_{t+1} = Y_t + (Vx\sin\theta + Vy\cos\theta)\cdot dt, \quad
\theta_{t+1} = \theta_t + Vz\cdot dt
$$

其中 $dt$ = 两次循环的时间差（`Sampling_Time`），$\theta$ = 当前偏航角。发布前会乘以修正系数（`odom_x_scale` 等，修正编码器系统误差），并把 yaw 转成四元数。

### 1.5 数据流图

```mermaid
flowchart TD
    subgraph 硬件[底盘硬件]
        MCU[STM32 下位机]
        ENC[轮子编码器]
        IMU_H[板载IMU/MPU6050]
        BAT[电池]
    end

    subgraph 节点[wheeltec_robot_node]
        SER[串口收发<br/>/dev/wheeltec_controller<br/>115200]
        FSM[Get_Sensor_Data_New<br/>三帧状态机<br/>帧头/帧尾/BCC校验]
        POS[位姿积分<br/>V·dt 运动学正解]
    end

    ENC --> MCU
    IMU_H --> MCU
    BAT --> MCU
    MCU -->|24字节帧 0x7B| SER
    MCU -->|8字节帧 0x7C| SER
    MCU -->|19字节帧 0xFA| SER
    SER --> FSM
    FSM -->|速度+IMU+电压| POS
    FSM -->|电压| PUBV[Publish_Voltage<br/>每11次发1次]
    POS --> ODOM[/odom_raw<br/>位置+速度+协方差/]
    POS --> TF1[odom_raw→base_footprint<br/>TF广播]
    FSM --> IMUP[/imu/stm32_raw<br/>四元数+角速度+加速度/]
    PUBV --> VOLT[/PowerVoltage/]

    VEL[/cmd_vel<br/>订阅/] -->|Cmd_Vel_Callback| SER
    SER -->|速度帧指令| MCU
    MCU -->|驱动| ENC
```

---

## 模块二：IMU 去零点漂移（imu_processor.launch.py）

### 2.1 模块功能

启动 `ImuProcessor_node`，把底盘发布的**原始 IMU**（`/imu/stm32_raw`）修正静止时的**零点漂移**问题，输出干净的 `/imu/imu_filtered`，供 EKF 使用。

### 2.2 原理：为什么需要去零点漂移？

陀螺仪（角速度计）存在**静态零偏**——即使静止不动，输出也不为 0。如果直接对原始角速度积分求姿态角，静止时姿态会**缓慢漂移**（越积越偏）。所以需要一个"静止检测 + 姿态解算冻结"策略。

### 2.3 实现：静止检测 + 姿态解算冻结（SetFreeze）

核心逻辑在 `ImuProcessor::imuCallback`：

```python
1. 订阅 /odom_raw 获取线速度 linear_vel_ 与角速度 angular_vel_
2. 若 |linear_vel_| < 阈值 且 |angular_vel_| < 阈值 → 静止计数 static_count++
   否则 → static_count = 0（说明在运动，不是静止）
3. 若 static_count >= 10（连续10次判静止）→ Quaternion_Solution.SetFreeze(true)
   否则 → SetFreeze(false)
4. Quaternion_Solution(out)：用 Mahony 互补滤波解算四元数姿态
5. 发布修正后的 IMU 到 /imu/imu_filtered
```

**冻结（freeze）的含义**：静止时把姿态解算的"角速度积分"冻结，让陀螺仪零偏不再累积进姿态角；同时用加速度计的重力方向来纠正姿态。运动时解除冻结，正常积分。

### 2.4 数据流图

```mermaid
flowchart LR
    subgraph 上游[来自底盘节点]
        A[/imu/stm32_raw<br/>原始IMU/]
        B[/odom_raw<br/>原始里程计/]
    end

    subgraph 处理[ImuProcessor 节点]
        SUB1[imuCallback<br/>订阅原始IMU]
        SUB2[odomCallback<br/>订阅里程计]
        JUDGE{静止判断<br/>|v|<阈值 且 |ω|<阈值}
        COUNT[连续10次静止?]
        FREEZE[SetFreeze<br/>true冻结/false解除]
        SOLVE[QuaternionSolution<br/>Mahony 姿态解算]
    end

    subgraph 下游[输出]
        OUT[/imu/imu_filtered<br/>去漂移IMU/]
    end

    A --> SUB1
    B --> SUB2
    SUB2 -->|v, ω| JUDGE
    JUDGE -->|静止| COUNT
    JUDGE -->|运动| COUNT
    COUNT -->|是| FREEZE
    COUNT -->|否| FREEZE
    FREEZE --> SOLVE
    SUB1 --> SOLVE
    SOLVE --> OUT
    OUT --> EKF[EKF 融合模块]
```

---

## 模块三：机器人模型与关节发布（robot_mode_description.launch.py）

### 3.1 模块功能

启动机器人**模型描述 + 静态 TF**，让上层（RViz、导航、建图）知道"传感器装在机器人哪里"：

- `robot_state_publisher`：加载 URDF 模型，发布各关节/连杆的 TF；
- `joint_state_publisher`：发布 `/joint_states` 关节状态话题（配合 URDF 显示关节）；
- 多个 `static_transform_publisher`：发布 `base_footprint → 各类传感器`（激光、相机、IMU、雷达）的**固定静态 TF**。

### 3.2 实现原理

**关键设计：OpaqueFunction + 读 yaml 动态生成节点**

不同车型（`mini_mec` / `top_akm_dl` 等）传感器安装位置不同，所以：

1. 运行时先读 `wheeltec_param.yaml` 拿到 `car_mode`（车型）；
2. 再读 `robot_model.yaml` 拿到该车型的传感器坐标（`base_to_laser` 等）；
3. 动态生成对应的静态 TF 发布节点。

**静态 TF vs 动态 TF 的区别**：

| 类型 | 发布者 | 用途 | 频率 |
|------|--------|------|------|
| 静态 TF | `static_transform_publisher` | 传感器相对本体的固定位置 | 一次（/tf_static） |
| 动态 TF | 里程计/EKF/AMCL | 本体相对世界的实时位姿 | 高频（/tf） |

**关节 TF**：`robot_state_publisher` 对 URDF 里的 `fixed` 关节直接按 `origin` 发布静态 TF；对 `continuous/revolute` 关节则读取 `joint_state_publisher` 的 `/joint_states`，结合关节 origin 动态发布。

**坐标系关系**（本项目的 TF 树核心）：

```
map → odom_ekfcombined（AMCL 或 EKF 发布）
  └→ base_footprint（EKF 发布）
       ├→ base_link（static，URDF）
       ├→ laser（static，机器人模型配置）
       ├→ camera_link（static）
       ├→ gyro_link（static）
       └→ radar（static）
```

### 3.3 数据流图

```mermaid
flowchart TD
    subgraph 配置[配置文件]
        Y1[wheeltec_param.yaml<br/>car_mode 车型]
        Y2[robot_model.yaml<br/>各车型传感器坐标]
        U1[wheeltec_robot_urdf 包<br/>car_mode_robot.urdf 模型]
    end

    subgraph 动态生成[OpaqueFunction 运行时]
        S1[读 yaml 取车型]
        S2[按车型拼URDF路径]
        S3[按车型生成传感器静态TF节点]
    end

    subgraph 节点[启动的节点]
        RSP[robot_state_publisher<br/>加载URDF发布关节TF]
        JSP[joint_state_publisher<br/>发布/joint_states]
        STF1[base_to_laser<br/>static TF]
        STF2[base_to_camera<br/>static TF]
        STF3[base_to_gyro<br/>static TF]
        STF4[base_to_radar<br/>static TF]
    end

    Y1 --> S1
    Y2 --> S1
    S1 --> S2
    S2 --> S3
    S3 --> RSP
    S3 --> STF1
    S3 --> STF2
    S3 --> STF3
    S3 --> STF4
    U1 --> RSP
    JSP -->|/joint_states| RSP
    RSP -->|关节TF| TF[(TF树)]
    STF1 -->|base_footprint→laser| TF
    STF2 -->|base_footprint→camera_link| TF
    STF3 -->|base_footprint→gyro_link| TF
    STF4 -->|base_footprint→radar| TF
```

---

## 模块四：EKF 里程计融合（wheeltec_ekf.launch.py）

### 4.1 模块功能

启动 `robot_localization` 包的 **EKF 节点**，把 **轮式里程计 + IMU** 按可信度（协方差）加权融合，输出更稳定的融合位姿 `/odom_ekfcombined`，并发布 `odom_ekfcombined → base_footprint` 的 TF。

### 4.2 原理：为什么要融合？什么是 EKF？

**各自的优缺点**：

| 传感器 | 优点 | 缺点 |
|--------|------|------|
| 轮式里程计 /odom | 短期准确、不会发散 | 打滑/磨损导致**长期漂移** |
| IMU | 高频平滑、姿态准 | **零点漂移**、积分发散 |

**EKF（扩展卡尔曼滤波）** 把两者按"预测 + 校正"两步融合：
1. **预测**：用运动模型（速度积分）预测下一个位姿；
2. **校正**：用传感器测量（里程计/IMU）校正预测值，误差由**协方差矩阵**决定谁更可信。

> 协方差越小 = 数据越可信。底盘在发布 `/odom_raw` 时就提供了两套协方差：**静止时用更小的协方差**（编码器没转，误差小），**运动时用较大的协方差**（可能打滑，让 EKF 多参考 IMU）。

### 4.3 三种配置（通过 launch 参数选择）

`wheeltec_ekf.launch.py` 根据 `carto_slam` / `robot_nav` 两个参数，动态启动不同节点与配置文件：

| 模式 | 触发参数 | 节点名 | 配置文件 | 特点 |
|------|---------|--------|---------|------|
| **普通模式** | 默认 | `ekf_filter_node` | `ekf.yaml` | 2D 模式，10Hz |
| **建图模式** | `carto_slam:=true` | `carto_ekf_filter_node` | `ekf_carto.yaml` | **3D 模式**，30Hz，`odom0_differential:false` |
| **导航模式** | `robot_nav:=true` | `nav_ekf_filter_node` | `ekf_nav.yaml` | 2D 模式，10Hz |

三者都输出到 `/odom_ekfcombined`（remap 自 `/odometry/filtered`），都 `publish_tf: true`。

**三种配置的关键差异与原因**：

| 参数 | 普通/导航 | 建图(carto) | 原因 |
|------|----------|------------|------|
| `two_d_mode` | true | **false** | 建图要完整 3D 位姿（含 z/roll/pitch），应对地面起伏；导航只需平面 |
| `frequency` | 10Hz | **30Hz** | Cartographer 位姿插值需要细粒度时间戳，要高频率 |
| `sensor_timeout` | 1.5s | **0.1s** | 建图对实时性敏感，过时数据立即丢弃 |
| `odom0_differential` | true | **false** | 普通/导航用差分平滑抗漂移；建图需要**完整绝对轨迹**做扫描匹配基准 |
| `imu0` | `/imu/imu_filtered` | `/imu/imu_filtered` | 都用去漂移后的 IMU |
| `odom0` | `/odom_raw` | `/odom_raw` | 都用原始里程计 |

**融合内容**（`odom0_config` / `imu0_config` 15 元素数组，顺序为 x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az）：
- odom 融合：x、y、yaw、vx、vy、vyaw（`true,true,false, false,false,true, true,true,false, false,false,true, ...`）
- imu 融合：yaw、vyaw（`false,false,false, false,false,true, ... false,false,true`）

**坐标系（REP-105 规范）**：
- `map_frame: map`：全局固定，可能跳变
- `odom_frame / world_frame: odom_ekfcombined`：连续、短期准、长期漂
- `base_link_frame: base_footprint`：随机器人动

### 4.4 数据流图

```mermaid
flowchart TD
    subgraph 输入[传感器输入]
        ODOM[/odom_raw<br/>原始里程计/]
        IMU_F[/imu/imu_filtered<br/>去漂移IMU/]
    end

    subgraph EKF节点[robot_localization EKF]
        subgraph 选配[launch 参数动态选择]
            P1[普通: ekf.yaml<br/>ekf_filter_node<br/>2D/10Hz]
            P2[建图: ekf_carto.yaml<br/>carto_ekf_filter_node<br/>3D/30Hz]
            P3[导航: ekf_nav.yaml<br/>nav_ekf_filter_node<br/>2D/10Hz]
        end
        PRED[① 预测<br/>运动模型积分]
        CORR[② 校正<br/>加权测量/协方差]
        REJ[异常过滤<br/>马氏距离阈值]
    end

    subgraph 输出[输出]
        OUT[/odom_ekfcombined<br/>融合位姿/]
        TF1[odom_ekfcombined→base_footprint<br/>TF广播]
    end

    ODOM --> P1 & P2 & P3
    IMU_F --> P1 & P2 & P3
    P1 & P2 & P3 --> PRED --> CORR --> REJ --> OUT
    OUT --> TF1
    OUT --> 上层[SLAM / NAV2]
```

---

## 模块五：三种 SLAM 建图方式

> SLAM（Simultaneous Localization And Mapping）= 机器人边移动边"定位自己 + 拼出地图"。

**三种方式对比**：

| 方式 | 包 | 核心算法 | 输入里程计 | 特点 |
|------|----|---------|-----------|------|
| gmapping | `slam_gmapping` | 粒子滤波 | EKF（/odom_ekfcombined） | 经典、老牌、适合小场景 |
| Cartographer | `wheeltec_cartographer` | 图优化 + 扫描匹配 | **原始**（/odom_raw）+原始IMU | Google 出品、精度高、可回环 |
| slam_toolbox | `wheeltec_slam_toolbox` | 图优化（Ceres） | EKF（/odom_ekfcombined） | 轻量、支持重定位模式 |

### 5.1 gmapping 建图（slam_gmapping.launch.py）

#### 原理：粒子滤波

用**一簇粒子**代表机器人可能的位置，每个粒子 = 一个假想位姿：

1. **预测**：按 EKF 里程计（/odom_ekfcombined）更新每个粒子的位姿；
2. **打分**：用激光 `/scan` 与已有地图匹配，给每个粒子打分（越吻合分数越高）；
3. **重采样**：按分数重新抽样，**高分粒子复制多份、低分粒子淘汰**，粒子总数不变；
4. **更新地图**：用高分粒子的位置，把当前激光扫描贴进 `/map` 地图。

粒子最终收敛到最可能的位姿，地图随之逐步完整。

#### 数据流图

```mermaid
flowchart TD
    subgraph 启动[launch 启动]
        BOT[turn_on_wheeltec_robot<br/>底层+EKF(普通配置)]
        LIDAR[wheeltec_lidar<br/>激光雷达]
        GM[slam_gmapping 节点]
    end

    subgraph 输入
        ODOM2[/odom_ekfcombined<br/>EKF里程计/]
        SCAN2[/scan<br/>激光/]
    end

    subgraph 粒子滤波[粒子滤波流程]
        P1[① 预测<br/>按里程计更新每个粒子位姿]
        P2[② 打分<br/>激光与地图匹配评分]
        P3[③ 重采样<br/>高分复制/低分淘汰]
        P4[④ 更新地图<br/>高分粒子叠加扫描]
    end

    BOT --> ODOM2
    LIDAR --> SCAN2
    ODOM2 --> GM
    SCAN2 --> GM
    GM --> P1 --> P2 --> P3 --> P4
    P4 --> MAP[/map<br/>栅格地图/]
```

### 5.2 Cartographer 建图（cartographer.launch.py）

#### 原理：猜 → 纠 → 调（预测 → 扫描匹配 → 回环优化）

Cartographer 使用**原始** odom 和 IMU（remap 到 `/odom_raw`、`/imu/stm32_raw`），因为它自带数据处理，配合激光扫描纠正更精确：

1. **猜（预测）**：用原始里程计 + 原始 IMU 增量预测机器人位置；
2. **纠（扫描匹配）**：在预测位置附近，把激光 `/scan` 与已有**子图**做相关扫描匹配（粗匹配 + Ceres 精修），找到最吻合位置，修正预测，并把新扫描贴进子图；
3. **调（全局优化）**：走到曾来过的地方时做**回环检测**（min_score=0.65），确认后每 50 个节点全局优化一次，调整所有子图位置，消除累积漂移。

**关键配置**（cartographer.lua）：
- `odom_frame = "odom_cartocombined"`、`provide_odom_frame = true`：Cartographer 自己发布 `odom_cartocombined → base_footprint` 的 TF；
- `use_odometry = true`、`use_imu_data = true`：用里程计+IMU 预测；
- `num_laser_scans = 1`：1 路 2D 激光；
- `submaps.num_range_data = 45`：45 帧激光生成一个子图；
- `use_online_correlative_scan_matching = true`：在线相关扫描匹配。

> **注意**：建图模式下 EKF 用 `ekf_carto.yaml`（3D、30Hz、`odom0_differential:false`），但 Cartographer **不用 EKF 的数据**，它吃原始数据。EKF 在此时的作用是**为 AMCL 提供完整位姿轨迹**（双 odom 架构：EKF→`odom_ekfcombined`，Cartographer→`odom_cartocombined`，两条独立的 TF 边指向 base_footprint）。

#### 数据流图

```mermaid
flowchart TD
    subgraph 启动[launch 启动]
        BOT2[turn_on_wheeltec_robot<br/>carto_slam:=true<br/>EKF用ekf_carto 3D/30Hz]
        LIDAR2[wheeltec_lidar<br/>激光雷达]
        CN[cartographer_node]
        OGN[cartographer_occupancy_grid_node<br/>子图→/map]
    end

    subgraph 输入2[原始数据输入]
        ODOMR[/odom_raw<br/>原始里程计/]
        IMUR[/imu/stm32_raw<br/>原始IMU/]
        SCAN3[/scan<br/>激光/]
    end

    subgraph 算法[Cartographer 三步]
        C1[① 猜：里程计+IMU 增量预测位姿]
        C2[② 纠：激光与子图相关扫描匹配<br/>Ceres 精修+贴图]
        C3[③ 调：回环检测 min_score=0.65<br/>每50节点全局优化]
    end

    BOT2 --> ODOMR
    BOT2 --> IMUR
    LIDAR2 --> SCAN3
    ODOMR --> CN
    IMUR --> CN
    SCAN3 --> CN
    CN --> C1 --> C2 --> C3
    C3 --> SUBMAP[内部子图] --> OGN
    OGN --> MAP2[/map<br/>栅格地图/]
    CN --> TF2[odom_cartocombined→base_footprint<br/>provide_odom_frame]
```

### 5.3 slam_toolbox 建图（online_async_launch.py）

#### 原理：与 Cartographer 相同的图优化，但吃 EKF 数据

slam_toolbox 建图原理与 Cartographer 相同（猜→纠→调），区别在于：
- **输入里程计**：用 **EKF 处理好的** `/odom_ekfcombined`（remap 自默认 `/odom`）；
- **更轻量**：实现更简单；
- **支持模式切换**：`mode: mapping`（建图）/ `localization`（重定位，可加载已有地图）；
- **异步（async）**：扫描匹配和建图分线程，响应快（本文件用的是 async 模式）。

关键配置（`mapper_params_online_async.yaml`）：Ceres 求解器、`do_loop_closing: true` 开回环、`resolution: 0.05`（5cm/格）、`map_update_interval: 5.0`（5 秒更新一次地图）。

#### 数据流图

```mermaid
flowchart TD
    subgraph 启动2[launch 启动]
        BOT3[turn_on_wheeltec_robot<br/>底层+EKF(普通配置)]
        LIDAR3[wheeltec_lidar<br/>激光雷达]
        ST[async_slam_toolbox_node]
    end

    subgraph 输入3
        ODOM3[/odom_ekfcombined<br/>remap 自 /odom/]
        SCAN4[/scan<br/>激光/]
    end

    subgraph 算法2[slam_toolbox 流程]
        S1[① 猜：EKF里程计运动预测]
        S2[② 纠：激光与地图相关性搜索匹配]
        S3[③ 调：回环检测+ Ceres 全局优化]
    end

    BOT3 --> ODOM3
    LIDAR3 --> SCAN4
    ODOM3 --> ST
    SCAN4 --> ST
    ST --> S1 --> S2 --> S3
    S3 --> MAP3[/map<br/>栅格地图/]
    S3 --> TF3[odom_ekfcombined→base_footprint<br/>由EKF发布]
```

---

## 模块六：NAV2 多点导航（wheeltec_nav2.launch.py）

### 6.1 模块功能

`wheeltec_nav2.launch.py` 是**导航总入口**，一次启动四组功能：

| 序号 | 启动内容 | 作用 |
|------|---------|------|
| ① | `waypoint_cycle` 节点 | 多点巡航：RViz 点多个航点，按序循环巡逻 |
| ② | `turn_on_wheeltec_robot.launch.py`（robot_nav:=true） | 底盘驱动 + 去漂移 IMU + EKF(导航配置) + 模型TF |
| ③ | `wheeltec_lidar.launch.py` | 激光雷达提供 `/scan` |
| ④ | NAV2 官方 `bringup_launch.py` | AMCL 定位 + 全局规划 + 局部控制 + 行为树 |

### 6.2 导航全流程原理

**第 1 步：加载地图 + 定位**
- `map_server` 加载地图 yaml → 发布 `/map`；
- **AMCL**（粒子滤波定位，原理同 gmapping）订阅 `/scan` + `/odom_ekfcombined`，粒子收敛确定"我在哪"，发布 `map → odom_ekfcombined` 的 TF，修正里程计漂移。

**第 2 步：RViz 点击目标 → 4 个子流程**

1. **bt_navigator**（导航大脑）：接收 `/goal_pose`，按行为树决策："全局规划 → 控制运动 → 卡住就恢复重规划"；
2. **planner_server**（地图师）：用 A\* 算法在全局代价地图上规划无障碍路径 → 发布 `/plan`；
3. **controller_server**（司机）：结合 `/scan` + 局部代价地图实时避障，沿 `/plan` 运动 → 发布 `/cmd_vel`；
4. **behavior_server**（救援队）：运动卡住/规划失败时执行恢复动作（旋转、清代价地图）。

最终 `/cmd_vel` 回到底盘驱动节点 → 下位机 → 电机，完成闭环。

### 6.3 waypoint_cycle 多点巡航原理

`waypoint_cycle` 节点（`nav2_waypoint_cycle` 包）实现**无限循环巡逻**：

- **订阅** `/clicked_point`（RViz 点击点）、`navigate_to_pose/_action/status`（导航结果）；
- **发布** `/path_point`（MarkerArray 可视化箭头+数字）、`/goal_pose`（导航目标）；
- **核心状态机**（`pose_callback`）：
  - 成功（status ≥ SUCCEEDED）→ 发下一个点；走完最后一个 `index=0` 回绕循环；
  - 失败 → 先重试 1 次（`try_again`），再失败就跳过；
- 键盘按 `c` 清空所有航点。

**导航闭环**：`waypoint_cycle` 发 `/goal_pose` → bt_navigator → 内部转成 NavigateToPose action → 规划/控制 → 底盘 → 完成后 status 话题反馈 → waypoint_cycle 发下一个点。

### 6.4 数据流图

```mermaid
flowchart TD
    subgraph 启动[wheeltec_nav2.launch.py 四组启动]
        WP[① waypoint_cycle<br/>多点巡航]
        BOT4[② turn_on_wheeltec_robot<br/>robot_nav:=true<br/>底盘+去漂移IMU+EKF(导航)+模型TF]
        LIDAR4[③ wheeltec_lidar<br/>激光雷达]
        BRING[④ bringup_launch.py<br/>NAV2核心]
    end

    subgraph NAV2核心[NAV2 内部]
        MAPS[map_server<br/>加载地图]
        AMCL[AMCL<br/>粒子滤波定位]
        BT[bt_navigator<br/>行为树决策]
        PLANNER[planner_server<br/>A*全局路径规划]
        CTRL[controller_server<br/>MPPI局部控制+避障]
        BEH[behavior_server<br/>卡住恢复]
    end

    subgraph 输入输出
        SCAN5[/scan/]
        ODOM4[/odom_ekfcombined/]
        MAP4[/map/]
        GOAL[/goal_pose/]
        CMDV[/cmd_vel/]
        CLICK[/clicked_point RViz点击/]
        STATUS[/navigate_to_pose/_action/status/]
    end

    WP --> GOAL
    WP --> CLICK
    BT --> STATUS
    BOT4 --> ODOM4
    LIDAR4 --> SCAN5

    MAPS --> MAP4 --> AMCL
    ODOM4 --> AMCL
    SCAN5 --> AMCL
    AMCL -->|map→odom_ekfcombined TF| TF4[(TF树)]
    ODOM4 --> BT & CTRL

    GOAL --> BT
    BT -->|路径| PLANNER
    PLANNER -->|/plan| CTRL
    SCAN5 --> CTRL
    CTRL -->|局部代价地图避障| CMDV
    BEH -->|恢复指令| BT
    CMDV --> BOT4 -->|电机驱动| 底盘运动
    STATUS --> WP
```

---

## 附录：数据流总览（一次完整导航）

```mermaid
flowchart LR
    A[用户在RViz点目标] -->|/goal_pose| B[bt_navigator]
    B -->|全局规划请求| C[planner_server]
    C -->|/plan 路径| D[controller_server]
    D -->|/cmd_vel| E[wheeltec_robot_node]
    E -->|串口帧| F[STM32 底盘]
    F -->|编码器/IMU上报| E
    E -->|/odom_raw /imu/stm32_raw| G[imu_processor+EKF]
    G -->|/odom_ekfcombined| H[AMCL 定位]
    L[激光雷达] -->|/scan| H
    H -->|map→odom TF 修正| G
    H -->|定位结果| B
    B -->|导航状态反馈| I[waypoint_cycle<br/>发下一个点]
```

**闭环要点**：底层驱动（模块1）→ 数据预处理（模块2/3/4）→ 建图/导航（模块5/6）→ 输出速度回到底盘，形成完整控制系统。
