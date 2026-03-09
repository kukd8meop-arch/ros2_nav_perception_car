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

#### 4. 整个tf工作流程

**map -> odom -> base_footprint -> base_link -> lidar_frame**

1. 雷达扫到障碍物（lidar_frame）
2. 算出障碍物在机器人身上的位置（base_link，即机器人机身的物理中心）
3. 映射到机器人在地面的位置（base_footprint，即机器人底盘贴在地面的中心点 z=0）
4. 结合里程计算局部位置（odom，即机器人底盘里程计/IMU 算出的局部坐标系）
5. 最后定位到地图的全局位置（map）

#### 5. 节点间数据传递
1. **底层控制板先读到编码器脉冲 -> odom_publisher根据底盘运动学模型做换算 -> 发布成一个 ROS2 标准消息：nav_msgs/Odometry -> /odom_raw**
2. **IMU -> imu_filter -> /imu**
3. **/odom_raw + /imu -> EKF -> /odom**
4. **雷达 -> lidar driver -> /scan_raw**
5. **/scan_raw + /odom + /tf -> slam_toolbox -> /map**

#### 补充
1. **/odom_raw**：

    本质上就是只靠轮子编码器推出来的“原始里程计结果”，里面通常包含两类信息：

    - **pose**：机器人当前估计位置和朝向
    - **twist**：机器人当前估计速度和角速度

    比如对于麦轮，它可能会算出：

    - 当前位置：`x=1.2, y=0.4`
    - 当前朝向：`yaw=0.3`
    - 当前速度：`vx=0.2, vy=0.0`
    - 当前角速度：`vyaw=0.1`

    这些都被打包成 `Odometry` 消息并发布到 `/odom_raw`。

2. **为什么叫raw**：

    因为它还是未经融合的原始里程计，缺点是：


    - 只靠轮子积分
    - 时间一长就会漂
    - 打滑时误差会变大
    - 转弯时累计误差尤其明显

    所以：

    - `/odom_raw` = 原始轮式里程计
    - `/odom` = 融合后的更可信里程计

3. **EKF**：

    EKF 不是简单“打包”，而是在做：

    - **预测**：先根据上一时刻状态和运动模型预测当前位置
    - **校正**：再用 `/odom_raw` 和 `/imu` 把预测拉回更合理的位置

    输出 `/odom` 之后，通常还会顺便发布：

    - `odom -> base_footprint` 这条 TF

    所以 `/odom` 可以理解成：

    - 轮速里程计 + IMU 融合后的底盘位姿估计
4. **/odom**：

    `/odom` 不是原始传感器数据，而是 EKF 融合后的结果。

    它比 `/odom_raw` 更可靠，因为：

    - `/odom_raw`` 只依赖编码器，容易因为打滑、累计积分而漂移
    - `/imu` 能补充姿态变化和角速度信息
    - EKF 会综合两者，给出更平滑、更稳定的底盘状态估计

    所以：

    - `/odom_raw` = 原始轮式里程计
    - `/odom` = 融合后的底盘位姿估计（给 SLAM / Navigation 用）

5. **为什么 `slam_toolbox` 需要 `/scan_raw + /odom + /tf`**：

    这里最容易卡住。核心原因是：

    > 雷达只知道“障碍物相对雷达在哪”，但不知道“障碍物在地图上哪”。

    所以 `slam_toolbox` 建图时必须同时拿三样东西：

    - **`/scan_raw`**：当前这一帧雷达扫描数据，告诉它“这一圈看到了什么”
    - **`/odom`**：机器人相对上一时刻大概移动了多少，给 scan matching 一个初始位姿估计
    - **`/tf`**：雷达相对机器人本体的安装位置，以及机器人在各个坐标系中的变换关系

    你可以把它理解成拼图：

    - `/scan_raw`：当前这块拼图长什么样
    - `/odom`：这块拼图大概应该放在哪附近
    - `/tf`：这块拼图是装在机器人身上哪个位置扫出来的
    - `slam_toolbox`：负责把当前拼图和历史地图对齐，最终拼成整张地图

6. **`slam_toolbox` 最后一步实际做了什么**：

    每收到一帧 `/scan_raw`，`slam_toolbox` 大致会做这几步：

    1. 查 `/tf`，把雷达坐标系中的数据换到机器人相关坐标系下
    2. 参考 `/odom`，估计机器人相对上一帧大概移动了多少
    3. 用 scan matching / 图优化方法，把当前雷达帧和历史地图对齐
    4. 更新地图
    5. 更新机器人在地图中的位置
    6. 发布 `/map`
    7. 发布 `map -> odom` TF

    所以最后一步并不是“雷达数据直接变地图”，而是：

    > 雷达数据 + 机器人运动估计 + 坐标变换关系，一起送给 `slam_toolbox`，由它完成建图与定位。

7. **一句话串起来**：

    - `/odom_raw`：编码器算出来的原始里程计
    - `/imu`：经过校准和滤波后的 IMU 数据
    - `/odom`：EKF 融合后的稳定里程计
    - `/scan_raw`：雷达扫描数据
    - `/tf`：各坐标系之间的变换关系
    - `/map`：`slam_toolbox` 最终构建出来的地图

    也就是说，建图模式本质上是在做：

    **“用雷达看环境，用里程计估计自己走了多少，用 TF 统一坐标系，最后交给 `slam_toolbox` 把这些信息拼成地图。”**


