#### 1. 完整 Launch 调用树
> 不断通过 `IncludeLaunchDescription` 向下层调用，最终落实到具体的 `Node`。

```text
【第1层：系统入口】
slam/launch/slam.launch.py
│
├─ 【第2层：基座组装】 slam/launch/include/robot.launch.py         
│  │
│  ├─ 【第3层：底盘+传感器融合】 driver/controller/launch/controller.launch.py
│  │  │
│  │  ├─ 【第4层：里程计流】 driver/controller/launch/odom_publisher.launch.py
│  │  │  │
│  │  │  ├─ simulations/rosorin_description/launch/robot_description.launch.py
│  │  │  │  │  读取: urdf/rosorin.xacro（主 xacro，include mecanum/acker）
│  │  │  │  ├─ Node: robot_state_publisher   ← 解析 xacro，发布静态 TF
│  │  │  │  └─ Node: joint_state_publisher   ← 发布关节状态
│  │  │  │
│  │  │  ├─ driver/ros_robot_controller/launch/ros_robot_controller.launch.py
│  │  │  │  └─ Node: ros_robot_controller    ← 【底盘最底层】读编码器/IMU，控电机
│  │  │  │       发布: /ros_robot_controller/imu_raw, 订阅: /cmd_vel
│  │  │  │
│  │  │  └─ Node: odom_publisher             ← 【计算层】轮速→里程计
│  │  │       配置: controller/config/calibrate_params.yaml
│  │  │       发布: /odom_raw
│  │  │
│  │  ├─ 【第4层：IMU处理流】 peripherals/launch/imu_filter.launch.py
│  │  │  ├─ Node: imu_calib (apply_calib)    ← IMU 零偏校正
│  │  │  │       订阅: /ros_robot_controller/imu_raw → 发布: imu_corrected
│  │  │  └─ Node: imu_filter (complementary_filter_node)  ← 互补滤波，补全姿态
│  │  │       订阅: imu_corrected → 发布: /imu（带 orientation）
│  │  │
│  │  ├─ Node: ekf_filter_node               ← 【计算层】EKF 传感器融合
│  │  │       配置: controller/config/ekf.yaml
│  │  │       订阅: /odom_raw + /imu
│  │  │       发布: /odom（融合后）+ odom→base_footprint TF
│  │  │
│  │  └─ driver/servo_controller/launch/servo_controller.launch.py
│  │       └─ Node: servo_controller         ← 舵机控制（非导航核心）
│  │
│  ├─ 【第3层：激光雷达】 peripherals/launch/lidar.launch.py
│  │  │  (根据环境变量 LIDAR_TYPE 路由)
│  │  └─ peripherals/launch/include/ldlidar_LD19.launch.py（以 LD19 为例）
│  │       └─ Node: ldlidar_stl_ros2_node    ← 【雷达最底层】
│  │            配置: port_name=/dev/lidar
│  │            发布: /scan_raw
│  │
│  └─ peripherals/launch/joystick_control.launch.py
│       └─ Node: joystick_control            ← 手柄遥控节点
│            发布: /controller/cmd_vel
│
└─ 【第2层：建图核心】 slam/launch/include/slam_base.launch.py   (延迟 10s 启动)
    │  配置: slam/config/slam.yaml
    └─ Node: sync_slam_toolbox_node          ← 【计算最底层】SLAM Toolbox 建图
         订阅: /scan_raw + /odom + /tf
         发布: /map + map→odom TF
```

#### 2. 最底层的干活节点（物理源码去这里找）

所有 launch 文件的嵌套最终只是为了启动以下几个真正的代码节点：

| 节点名称 | 源码对应文件 | 职责说明 |
|---------|-------------|---------|
| `ros_robot_controller` | `driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py`<br>`.../ros_robot_controller_sdk.py` | 真正的**硬件死磕层**。通过串口/I2C和底层单片机通信，下发电机 PWM/转速命令，读取编码器脉冲和原生物理 IMU 数据。 |
| `odom_publisher` | `driver/controller/controller/odom_publisher_node.py`<br>`.../mecanum.py` (或 `ackermann.py`) | **运动学解算层**。把编码器的轮速（m/s），乘以轮距轮半径，用麦克纳姆轮的运动学矩阵公式，算出来机器人当前的 `vx, vy, omega`，积分得到 `odom_raw` 坐标。 |
| `ldlidar_stl_ros2_node` | (系统 apt 安装的 ROS 包) | 也就是雷达厂家的 SDK。不用你写，只要配对 `/dev/lidar` 端口。 |
| `sync_slam_toolbox_node`| (系统 apt 安装的 ROS 包) | Nav2 推荐的 SLAM 算法。 |
| `ekf_filter_node`       | (系统 apt 安装的 robot_localization 包) | 也是现成的包。读 `ekf.yaml`，把轮速里程计和 IMU 融到一起。 |

#### 3. 层级调用规律总结

整个工程的组织非常典型，遵守了模块化设计原则：

1. **入口层 (Level 1):** `slam.launch.py` / `navigation.launch.py` 负责决定系统模式。
2. **组装层 (Level 2):** `robot.launch.py` 充当底盘和传感器的代理人。
3. **模块层 (Level 3):** `controller`, `lidar`, `imu_filter` 的 launch 仅负责拉起自己领域的任务。
4. **实现层 (Level 4):** 具体的 python / C++ `Node` 执行逻辑运算和硬件互操作。