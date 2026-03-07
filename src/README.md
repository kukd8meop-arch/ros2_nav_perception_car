# ROS-Orin NX 机器人 ROS2 工程说明

> 本工程基于 ROS2，运行于 Jetson Orin NX 平台，是一套完整的智能机器人软件系统。
> 涵盖底层驱动、运动控制、传感器接入、SLAM建图、自主导航、视觉AI应用、大模型交互、语音控制等功能。

---

## 📁 目录结构总览

```
src/
├── app/                        # 🤖 上层智能应用（巡线、避障、目标追踪、自动驾驶等）
├── bringup/                    # 🚀 系统启动引导与开机自检
├── calibration/                # 📐 运动标定（线速度/角速度校准）
├── driver/                     # ⚙️ 底层驱动栈（硬件通信、运动控制、舵机、机械臂）
│   ├── controller/             #     底盘运动控制与里程计
│   ├── kinematics/             #     机械臂正/逆运动学
│   ├── kinematics_msgs/        #     机械臂消息/服务定义
│   ├── ros_robot_controller/   #     主控制板硬件驱动
│   ├── ros_robot_controller_msgs/ #  主控制板消息/服务定义
│   ├── sdk/                    #     公共工具库（PID、FPS、LED等）
│   ├── servo_controller/       #     舵机控制管理器
│   └── servo_controller_msgs/  #     舵机消息定义
├── example/                    # 📚 综合功能示例（视觉检测、AI推理、手势控制等）
├── interfaces/                 # 📨 自定义 ROS2 消息与服务接口定义
├── large_models/               # 🧠 大语言模型 (LLM) 集成模块
│   ├── large_models/           #     核心节点（语音检测、Agent、TTS）
│   └── large_models_msgs/      #     大模型消息/服务定义
├── large_models_examples/      # 💡 大模型应用示例（LLM控制、VLM视觉等）
├── multi/                      # 👥 多机器人协作（编队、TF同步）
├── navigation/                 # 🗺️ 自主导航（Nav2 + RTAB-Map）
├── peripherals/                # 🎮 外设驱动（手柄、键盘、相机、雷达、IMU）
├── simulations/                # 🖥️ 仿真模型描述（URDF + Meshes）
│   └── rosorin_description/
├── slam/                       # 🌍 SLAM 建图（激光雷达 + 视觉）
├── xf_mic_asr_offline/         # 🎤 讯飞离线语音识别模块
└── xf_mic_asr_offline_msgs/    # 📝 语音识别消息/服务定义
```

---

## 📦 各模块详细说明

### 1. `app` — 上层智能应用

机器人的**核心应用层**，集成了多种基于视觉和激光雷达的智能玩法。

| 节点 | 功能 |
|------|------|
| `lidar_controller` | 激光雷达控制：支持**避障**、**跟随**、**警卫**三种模式，使用 PID 控制底盘运动 |
| `line_following` | 视觉巡线：在 LAB 色彩空间中检测目标颜色线条，PID 控制循线行驶，支持雷达辅助避障 |
| `object_tracking` | 颜色目标追踪：检测指定颜色物体，驱动底盘跟踪目标 |
| `ar_app` | AR 增强现实应用 |
| `hand_trajectory` | 手部轨迹追踪 |
| `hand_gesture` | 手势识别控制 |

**Launch 文件：**
- `start_app.launch.py` — 总入口，启动所有子应用
- `self_driving_node.launch.py` — 自动驾驶（YOLOv8 交通标志识别 + 巡线）

---

### 2. `bringup` — 系统启动引导

整个机器人系统的**一键启动与开机自检**模块。

| 组件 | 功能 |
|------|------|
| `startup_check` 节点 | 开机自检：检测麦克风、驱动蜂鸣器提示、OLED 显示 IP 和设备信息 |
| `bringup.launch.py` | **系统主 launch**，统一启动：底盘控制器、深度相机、激光雷达、WebSocket 桥接、视频流服务、应用层、手柄遥控等 |
| `expand_rootfs.service` | 首次启动自动扩展 rootfs 分区 |
| `slam.sh` / `navigation.sh` | 桌面快捷脚本，一键启动 SLAM 建图或自主导航 |

---

### 3. `calibration` — 运动标定

用于校准机器人运动精度的标定工具。

| 节点 | 功能 |
|------|------|
| `calibrate_linear` | 线速度标定：校准直线行驶精度 |
| `calibrate_angular` | 角速度标定：校准旋转精度 |

---

### 4. `driver/` — 底层驱动栈

机器人的**完整硬件驱动层**，从硬件通信到运动控制。

#### 4.1 `controller` — 底盘运动控制
- 支持**阿克曼 (Ackermann)** 和**麦克纳姆轮 (Mecanum)** 两种底盘模型
- 提供里程计发布 (`odom_publisher`) 和初始位姿设定 (`init_pose`)

#### 4.2 `kinematics` — 机械臂运动学
- 正运动学 / 逆运动学（C 扩展库）
- 运动学控制、坐标变换、运动学解搜索

#### 4.3 `kinematics_msgs` — 机械臂消息定义
- 关节范围、连杆信息、机器人位姿等 msg/srv 定义

#### 4.4 `ros_robot_controller` — 主控制板驱动
- 通过 SDK 与底层控制板串口通信
- 统一管理：**电机、舵机、LED、蜂鸣器、按钮、OLED 屏幕**等硬件

#### 4.5 `ros_robot_controller_msgs` — 主控制板消息定义
- 总线舵机、PWM 舵机、电机、LED/RGB、蜂鸣器、OLED、按钮、遥控器等 msg/srv

#### 4.6 `sdk` — 公共工具库
- `common.py` — 通用工具函数
- `fps.py` — 帧率计算
- `led.py` — LED 控制封装
- `pid.py` — PID 控制器
- **无 ROS 节点**，纯 Python 库供其他模块引用

#### 4.7 `servo_controller` — 舵机控制管理器
- 动作组控制器、总线舵机控制、关节位置/轨迹控制
- 由 `controller_manager.py` 统一管理

#### 4.8 `servo_controller_msgs` — 舵机消息定义
- 舵机位置、舵机状态等消息定义

---

### 5. `example` — 综合功能示例

包含**大量视觉和 AI 驱动的机器人应用示例**，适合学习和二次开发。

| 类别 | 示例节点 |
|------|---------|
| 颜色检测与跟踪 | `color_detect`、`color_track`、`color_sorting`（颜色分拣） |
| 手部相关 | `hand_detect`、`hand_track`、`hand_trajectory`、`hand_gesture_control` |
| 人体控制 | `body_control`、`body_track`、`fall_down_detect`（跌倒检测） |
| 巡线与自动驾驶 | `line_follow_clean`（巡线清扫）、`self_driving`、`urban_traffic` |
| YOLO 目标检测 | `yolov5_node`、`yolov8_node`、`yolov11_node`、`yolov11_detect_demo` |
| RGBD 应用 | `cross_bridge`（过桥）、`prevent_falling`（防跌落） |
| AR 码检测 | `ar_detect` |

---

### 6. `interfaces` — 自定义消息与服务

整个系统的**通信接口定义**，是各模块间数据交互的基础。

**消息 (msg)：**
`ColorDetect`、`ColorInfo`、`ObjectInfo`、`ObjectsInfo`、`PixelPosition`、`Point2D`、`Pose2D`、`ROI`、`LineROI` 等

**服务 (srv)：**
`SetColorDetectParam`、`SetPose`、`SetPose2D`、`SetPoint`、`SetFloat64`、`SetInt64`、`SetString`、`SetCircleROI`、`SetLineROI` 等

---

### 7. `large_models` — 大语言模型集成

集成 **LLM（大语言模型）** 实现语音交互链路。

| 子包 | 功能 |
|------|------|
| `large_models` | 核心节点：`vocal_detect`（语音检测）、`agent_process`（AI Agent 处理）、`tts_node`（文本转语音） |
| `large_models_msgs` | Agent 相关消息/服务定义：工具调用、传输对象设置、模型切换等 |

---

### 8. `large_models_examples` — 大模型应用示例

基于大模型的**多种机器人智能应用示例**。

| 节点 | 功能 |
|------|------|
| `llm_control_move` / `llm_control_move_offline` | LLM 控制机器人移动（在线/离线） |
| `llm_color_track` | LLM 驱动的颜色追踪 |
| `llm_visual_patrol` | LLM 视觉巡逻 |
| `vllm_with_camera` | VLM（视觉大模型）摄像头交互 |
| `vllm_track` | VLM 目标跟踪 |
| `vllm_navigation` | VLM 导航 |
| `llm_control` | 基于 Function Calling 的 LLM 控制 |
| `navigation_controller` | 导航控制器辅助节点 |

---

### 9. `multi` — 多机器人协作

实现**多台机器人之间的协同运动和编队控制**。

| 节点 | 功能 |
|------|------|
| `formation_update` | 编队更新与调整 |
| `slave_tf_listener` | 从机 TF 坐标监听 |
| `tf_listen` / `tf_publish` | TF 坐标变换监听与发布 |
| `costmap_publish` | 代价地图发布 |

---

### 10. `navigation` — 自主导航

基于 **Navigation2** 和 **RTAB-Map** 的自主导航模块。

| Launch 文件 | 功能 |
|-------------|------|
| `navigation.launch.py` | 2D 激光雷达导航 |
| `rtabmap_navigation.launch.py` | RGBD 视觉导航（RTAB-Map） |
| `rviz_navigation.launch.py` | 导航 RViz 可视化 |
| `rviz_rtabmap_navigation.launch.py` | RTAB-Map 导航可视化 |

---

### 11. `peripherals` — 外设驱动与控制

各种**外围传感器和输入设备**的驱动。

| 节点/Launch | 功能 |
|-------------|------|
| `joystick_control` | 手柄遥控控制 |
| `teleop_key_control` | 键盘遥控控制 |
| `tf_broadcaster_imu` | IMU TF 坐标广播 |
| `depth_camera.launch.py` | 深度相机启动 |
| `lidar.launch.py` | 激光雷达启动 |
| `usb_cam.launch.py` | USB 摄像头启动 |
| `imu_filter.launch.py` | IMU 数据滤波 |

---

### 12. `simulations` — 仿真模型

| 子包 | 功能 |
|------|------|
| `rosorin_description` | 机器人 **URDF 模型描述**：包含 URDF 文件、3D Mesh 网格、RViz 可视化配置，用于仿真和模型展示 |

---

### 13. `slam` — SLAM 建图

**同时定位与建图**模块，支持两种建图方式。

| Launch 文件 | 功能 |
|-------------|------|
| `slam.launch.py` | 2D 激光雷达 SLAM 建图 |
| `rtabmap_slam.launch.py` | RTAB-Map 视觉 SLAM 建图 |
| `rviz_slam.launch.py` | SLAM RViz 可视化 |
| `map_save` 节点 | 保存构建的地图 |

---

### 14. `xf_mic_asr_offline` — 讯飞离线语音识别

集成**讯飞 (iFlytek) 麦克风阵列 SDK**，实现离线语音控制。

| 组件 | 功能 |
|------|------|
| `voice_control` (C++) | 语音控制核心：底层音频处理、唤醒检测、离线 ASR |
| `awake_node.py` | 唤醒词检测节点 |
| `asr_node.py` | 离线语音识别节点 |
| `voice_control_move.py` | 语音控制移动 |
| `voice_control_navigation.py` | 语音控制导航 |
| `voice_control_color_detect.py` | 语音控制颜色检测 |
| `wonder_echo_pro_node.py` | Wonder Echo Pro 节点 |

### 15. `xf_mic_asr_offline_msgs` — 语音识别消息定义

| 服务 | 功能 |
|------|------|
| `GetOfflineResult.srv` | 获取离线识别结果 |
| `SetString.srv` | 设置字符串参数 |

---

## 🏗️ 系统架构概览

```
┌─────────────────────────────────────────────────────────┐
│                    应用层 (Application)                    │
│  app / example / large_models_examples / multi           │
├─────────────────────────────────────────────────────────┤
│                    AI & 大模型 (AI & LLM)                  │
│  large_models / xf_mic_asr_offline                       │
├─────────────────────────────────────────────────────────┤
│                  导航与建图 (Nav & SLAM)                    │
│  navigation / slam / calibration                         │
├─────────────────────────────────────────────────────────┤
│                   外设与传感器 (Peripherals)                │
│  peripherals (相机、雷达、IMU、手柄、键盘)                    │
├─────────────────────────────────────────────────────────┤
│                   底层驱动 (Driver Stack)                   │
│  controller / ros_robot_controller / servo_controller    │
│  kinematics / sdk                                        │
├─────────────────────────────────────────────────────────┤
│                   接口定义 (Interfaces)                     │
│  interfaces / *_msgs (各模块消息与服务定义)                   │
├─────────────────────────────────────────────────────────┤
│                   系统引导 (Bringup)                       │
│  bringup (一键启动 + 自检 + 自启服务)                        │
└─────────────────────────────────────────────────────────┘
```

---

## 🚀 快速启动

```bash
# 一键启动整个系统
ros2 launch bringup bringup.launch.py

# 单独启动 SLAM 建图
ros2 launch slam slam.launch.py

# 单独启动导航
ros2 launch navigation navigation.launch.py

# 启动应用层（巡线、避障、追踪等）
ros2 launch app start_app.launch.py

# 启动语音控制
ros2 launch xf_mic_asr_offline voice_control_move.launch.py
```

---

## 🔧 环境变量

系统通过环境变量适配不同硬件配置：

| 环境变量 | 说明 | 示例值 |
|---------|------|--------|
| `MACHINE_TYPE` | 底盘类型 | `MecanumWheelChassis` / `AckerChassis` |
| `LIDAR_TYPE` | 激光雷达型号 | `ydlidar` / `rplidar` |
| `DEPTH_CAMERA_TYPE` | 深度相机类型 | `aurora` / `astra` |
| `need_compile` | 是否使用编译安装路径 | `True` / `False` |

---

*📅 文档生成时间：2026-02-25 | 作者：DREAMER818*
