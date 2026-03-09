#### 1. 完整 Launch 调用树
> 导航模式和建图模式共用同一套“基座层”，区别在于上层不再启动 `slam_toolbox`，而是启动 `map_server + AMCL + Nav2`。

```text
【第1层：系统入口】
navigation/launch/navigation.launch.py
│
├─ 【第2层：基座组装】 slam/launch/include/robot.launch.py
│  │
│  ├─ driver/controller/launch/controller.launch.py
│  │  ├─ driver/controller/launch/odom_publisher.launch.py
│  │  │  ├─ simulations/rosorin_description/launch/robot_description.launch.py
│  │  │  │  ├─ Node: robot_state_publisher   ← 发布机器人静态/关节 TF
│  │  │  │  └─ Node: joint_state_publisher
│  │  │  ├─ driver/ros_robot_controller/launch/ros_robot_controller.launch.py
│  │  │  │  └─ Node: ros_robot_controller    ← 读编码器/IMU，控电机
│  │  │  └─ Node: odom_publisher             ← 编码器/运动学 → /odom_raw
│  │  │
│  │  ├─ peripherals/launch/imu_filter.launch.py
│  │  │  ├─ Node: imu_calib                  ← IMU 静态校准
│  │  │  └─ Node: imu_filter                 ← 输出 /imu
│  │  │
│  │  ├─ Node: ekf_filter_node               ← /odom_raw + /imu → /odom
│  │  │                                        同时发布 odom→base_footprint TF
│  │  │
│  │  └─ driver/servo_controller/launch/servo_controller.launch.py
│  │
│  ├─ peripherals/launch/lidar.launch.py
│  │  └─ Node: ldlidar_stl_ros2_node         ← 发布 /scan_raw
│  │
│  └─ peripherals/launch/joystick_control.launch.py
│       └─ Node: joystick_control            ← 手柄调试可发布 /cmd_vel
│
└─ 【第2层：导航核心】 navigation/launch/include/bringup.launch.py   (延迟 10s 启动)
   │
   ├─ Node: nav2_container                    ← 容器，后续 Nav2 组件加载进这里
   │
   ├─ 【第3层：定位层】 navigation/launch/include/localization.launch.py
   │  ├─ ComposableNode: map_server          ← 读取 map.yaml，发布 /map
   │  ├─ ComposableNode: amcl                ← /scan_raw + /map + /odom → map→odom
   │  └─ ComposableNode: lifecycle_manager_localization
   │
   └─ 【第3层：导航执行层】 navigation/launch/include/navigation_base.launch.py
      ├─ ComposableNode: controller_server   ← 局部控制器（DWB 或 TEB）
      ├─ ComposableNode: smoother_server     ← 路径平滑
      ├─ ComposableNode: planner_server      ← 全局路径规划
      ├─ ComposableNode: behavior_server     ← 恢复/行为逻辑
      ├─ ComposableNode: bt_navigator        ← 行为树调度总控
      ├─ ComposableNode: waypoint_follower   ← 多航点执行
      ├─ ComposableNode: velocity_smoother   ← cmd_vel_nav → cmd_vel
      └─ ComposableNode: lifecycle_manager_navigation
```

#### 2. 最底层的干活节点（物理源码去这里找）

导航模式最终真正干活的，不只是底盘节点，还包括 Nav2 的几个核心组件：

| 节点名称 | 源码/来源 | 职责说明 |
|---------|----------|---------|
| `ros_robot_controller` | `driver/ros_robot_controller/...` | 底层硬件驱动。接收 `/cmd_vel`，控制电机；读取编码器和原始 IMU。 |
| `odom_publisher` | `driver/controller/controller/odom_publisher_node.py` | 运动学解算。将轮速转换成 `/odom_raw`。 |
| `ekf_filter_node` | `robot_localization` 系统包 | 融合 `/odom_raw + /imu`，输出 `/odom`。 |
| `ldlidar_stl_ros2_node` | 雷达系统包 | 输出 `/scan_raw`。 |
| `map_server` | `nav2_map_server` | 读取保存好的地图文件并发布 `/map`。 |
| `amcl` | `nav2_amcl` | 粒子滤波定位。利用 `/scan_raw + /map + /odom` 估计机器人在地图中的位置。 |
| `planner_server` | `nav2_planner` | 计算从当前位置到目标点的全局路径。 |
| `controller_server` | `nav2_controller` | 根据全局路径和局部代价地图生成速度指令。 |
| `velocity_smoother` | `nav2_velocity_smoother` | 对控制器输出的速度做平滑，再发给底盘。 |
| `bt_navigator` | `nav2_bt_navigator` | 导航任务总控，协调规划、控制、恢复等模块。 |

#### 3. 层级调用规律总结

导航模式的组织方式和建图模式非常像，只是上层目标不一样：

1. **入口层 (Level 1)**：`navigation.launch.py` 决定当前进入导航模式。
2. **组装层 (Level 2)**：仍然先启动 `robot.launch.py`，把底盘、雷达、IMU、EKF 这些基础设施准备好。
3. **定位层 (Level 3)**：`localization.launch.py` 负责加载地图并启动 `AMCL` 定位。
4. **导航执行层 (Level 3)**：`navigation_base.launch.py` 负责路径规划、局部控制、行为树调度。
5. **实现层 (Level 4)**：真正干活的是各个 Nav2 组件节点和底层控制节点。

#### 4. 整个 tf 工作流程

**map -> odom -> base_footprint -> base_link -> lidar_frame**

导航模式下这条 TF 链仍然存在，但和建图模式相比，最关键的区别是：

- 建图模式下：`map -> odom` 主要由 `slam_toolbox` 发布
- 导航模式下：`map -> odom` 主要由 `AMCL` 发布

可以这样理解：

1. `robot_state_publisher` 负责发布机器人本体结构关系，例如 `base_link -> lidar_frame`
2. `ekf_filter_node` 负责发布 `odom -> base_footprint`
3. `map_server` 只负责提供地图，不负责机器人位姿
4. `AMCL` 结合 `/scan_raw + /map + /odom` 推断机器人在地图中的位置，并发布 `map -> odom`

因此导航模式下，机器人在地图中的最终位姿是：

> **地图约束（AMCL） + 局部连续运动估计（EKF / odom） + 机器人自身结构 TF**

#### 5. 节点间数据传递

1. **编码器 -> `odom_publisher` -> `/odom_raw`**
2. **IMU -> `imu_filter` -> `/imu`**
3. **`/odom_raw + /imu` -> `EKF` -> `/odom`**
4. **雷达 -> lidar driver -> `/scan_raw`**
5. **地图文件 `map.yaml` -> `map_server` -> `/map`**
6. **`/scan_raw + /map + /odom` -> `AMCL` -> `map -> odom` TF**
7. **目标点 -> `planner_server` -> 全局路径**
8. **全局路径 + local/global costmap + `/odom` -> `controller_server` -> `cmd_vel_nav`**
9. **`cmd_vel_nav` -> `velocity_smoother` -> `cmd_vel`**
10. **`cmd_vel` -> `ros_robot_controller` -> 电机转动**

#### 补充

1. **`/map` 是什么**：

    `/map` 不是传感器原始数据，而是已经保存好的环境地图被 `map_server` 重新加载后发布出来的栅格地图。

    它的作用是：

    - 给 `AMCL` 做定位参考
    - 给全局代价地图作为静态底图
    - 给规划器计算从起点到终点的可行路径

2. **`AMCL` 在导航模式下干什么**：

    `AMCL` 不是建图，它是在“已有地图”上做定位。

    它会综合：

    - `/scan_raw`：当前雷达看到的环境轮廓
    - `/map`：事先保存好的地图
    - `/odom`：机器人相对上一时刻移动了多少

    然后不断回答一个问题：

    > 机器人现在在地图上的哪个位置？

    最终它通过发布 `map -> odom` TF，把这个结果告诉整个导航系统。

3. **为什么导航模式也需要 `/odom`**：

    因为 `AMCL` 和局部控制器都不可能只靠一帧雷达数据工作。

    - `AMCL` 需要 `/odom` 提供粒子运动预测
    - `controller_server` 需要 `/odom` 知道机器人当前速度和姿态变化
    - `velocity_smoother` 也会参考 `/odom` 做速度平滑

    所以 `/odom` 在导航模式里仍然是一个非常核心的“局部连续运动估计”。

4. **全局规划和局部控制怎么分工**：

    - `planner_server`：负责从起点到目标点规划一条全局路径
    - `controller_server`：负责沿着这条路径实时跟踪，并根据局部障碍调整速度

    你可以理解成：

    - 全局规划 = “路线怎么走”
    - 局部控制 = “这一刻轮子应该怎么转”

5. **`velocity_smoother` 为什么还要再来一层**：

    因为局部控制器输出的速度指令可能变化太快，如果直接发到底盘，机器人会显得很冲，甚至产生机械抖动。

    所以这里加了一层：

    - `cmd_vel_nav`：Nav2 控制器给出的速度
    - `cmd_vel`：经过平滑后的最终底盘控制指令

6. **一句话串起来**：

    导航模式本质上是在做：

    **“先把底盘、雷达、IMU、EKF 这些基础设施跑起来，再加载已有地图，用 `AMCL` 在地图中定位自己，用 Nav2 规划路径、生成速度，并最终把速度指令发给底盘执行。”**

7. **和建图模式的主要区别**：

    如果和 `SLAM_Launch_Tree_Architecture.md` 对照来看，最关键的差异只有一条：

    - 建图模式：上层核心是 `slam_toolbox`
    - 导航模式：上层核心是 `map_server + AMCL + Nav2`

    换句话说：

    - 建图模式解决“地图从哪里来”
    - 导航模式解决“我在地图上哪，以及我该怎么走过去”
