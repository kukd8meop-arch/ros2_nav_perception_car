# Gazebo 闭环仿真搭建计划

## 1. 文档目的

这份文档用于记录：

- 当前仓库已经具备了哪些仿真基础
- 从当前状态到“能在电脑上完成闭环导航仿真”还差哪些环节
- 每个阶段应该先做什么、后做什么
- 每个阶段完成后如何验证
- 闭环仿真跑通之后，后续还能继续优化哪些内容

这份计划的目标不是一步到位做出“高保真工业级仿真”，而是先让本仓库尽快具备一条**可运行、可验证、可迭代**的 Gazebo + Nav2 闭环仿真链路。

---

## 2. 当前状态梳理

### 2.1 已经具备的基础

当前仓库并不是从零开始，已经有一些非常关键的基础：

- 已有机器人描述包：`src/simulations/rosorin_description`
- 已有机器人本体 xacro：`src/simulations/rosorin_description/urdf/rosorin.xacro`
- 已有 Gazebo 版本 xacro：`src/simulations/rosorin_description/urdf/rosorin.gazebo.xacro`
- 已有描述启动文件：`src/simulations/rosorin_description/launch/robot_description.launch.py`
- 已有导航包：`src/navigation`
- 已有导航参数：`src/navigation/config/nav2_params.yaml`
- 已有地图文件：`src/slam/maps/map_01.yaml` 等
- 已有 RViz 配置：`src/navigation/rviz/`、`src/simulations/rosorin_description/rviz/`

### 2.2 已识别出的可复用仿真元素

`rosorin.gazebo.xacro` 中已经定义了 Gazebo 插件，说明仓库作者原本就考虑过仿真扩展。原始文件使用的是 ROS1 风格的插件名，在阶段 A/B 中已升级为 ROS2 Humble 兼容版本：

- 平面运动插件：`libgazebo_ros_planar_move.so`（ROS2 原生，无需更改）
- 激光插件：`libgazebo_ros_laser.so` → 已升级为 `libgazebo_ros_ray_sensor.so`
- IMU 插件：`libgazebo_ros_imu.so` → 已升级为 `libgazebo_ros_imu_sensor.so`

这意味着：

- 不需要从头重新造机器人仿真模型
- 不需要从头重新写激光和 IMU 仿真插件
- 重点工作会落在 **启动链整合、话题统一、导航接入、验证闭环** 上

### 2.3 当前还缺失的关键环节

从“已有描述能力”到“闭环导航仿真”之间，目前还差这些关键拼图：

- 缺少 Gazebo world 文件
- 缺少专门的 Gazebo 启动文件
- 缺少 `spawn_entity` 生成机器人到 Gazebo 世界中的启动链
- 缺少仿真专用的导航入口，例如 `navigation_sim.launch.py`
- 缺少仿真话题与现有导航链路的统一策略
- 缺少一套明确的验证步骤与验收标准

---

## 3. 总体目标

整个仿真建设分成三个层次目标：

### 3.1 第一层目标：最小可运行仿真

达到以下状态：

- Gazebo 能启动
- 机器人能成功生成到 Gazebo 世界中
- 能看到机器人模型、TF、激光、里程计、IMU 话题
- RViz2 能同时显示仿真机器人和地图

这一步的核心是：

> 先把“机器人在仿真世界里活起来”。

### 3.2 第二层目标：闭环导航仿真

达到以下状态：

- Nav2 能正常启动
- AMCL / map_server / planner / controller 能运行
- RViz2 中可以使用 `2D Pose Estimate`
- RViz2 中可以使用 `Nav2 Goal`
- 机器人能在 Gazebo 世界中移动到指定目标点

这一步的核心是：

> 让 Gazebo、定位、导航、控制形成闭环。

### 3.3 第三层目标：逐步逼近实车链路

达到以下状态：

- 话题命名尽可能与实车一致
- 可以选择是否接入 `imu_filter`、`ekf_filter_node`、`rf2o` 等中间层
- 仿真链与实车链尽量复用相同的 launch 架构
- 未来支持算法验证、参数调优、性能对比和回归测试

这一步的核心是：

> 不只是“能跑”，而是让仿真真正服务于开发和优化。

---

## 4. 分阶段实施计划

---

## 阶段 A：准备 Gazebo 基础环境

### A.1 目标

确认电脑具备 Gazebo Classic 运行条件，并让仓库具备世界文件与仿真入口。

### A.2 需要完成的事项

1. 安装 Gazebo Classic 相关依赖
2. 新建 world 文件目录与最小 world 文件
3. 新建 Gazebo 启动 launch 文件
4. 验证 Gazebo 能空场景启动

### A.3 建议新增文件

建议新增：

- `src/simulations/rosorin_description/worlds/empty.world`
- `src/simulations/rosorin_description/launch/gazebo.launch.py`

### A.4 验收标准

满足以下条件即可通过：

- `gazebo.launch.py` 能启动 Gazebo
- 世界能正常打开，不报插件缺失错误
- Gazebo GUI 可见

### A.5 风险点

- 本机未安装 Gazebo Classic 相关 ROS 包
- `gazebo_ros` 插件缺失
- 显卡/OpenGL 或远程桌面环境对 Gazebo GUI 支持不稳定

### 阶段 A 当前已完成记录

当前这一阶段已经完成了基础落地，具体包括：

- 已确认本机存在 Gazebo Classic 相关依赖：`gazebo_ros`、`gazebo_plugins`、`gazebo_msgs` 以及 `gazebo` 可执行程序均可用；
- 已新增最小 world 文件：`src/simulations/rosorin_description/worlds/empty.world`；
- 已新增 Gazebo 启动文件：`src/simulations/rosorin_description/launch/gazebo.launch.py`；
- 已补充 `rosorin.gazebo.xacro` 中缺失的参数声明：`odom_frame`、`base_frame`、`lidar_frame`；
- 已更新 `rosorin_description` 的安装配置，使 `worlds/` 目录能够随包一起安装；
- 已补充 `package.xml` 中与 Gazebo 启动相关的依赖声明。

同时已经完成了两轮基础验证：

- `colcon build --packages-select rosorin_description` 通过；
- 使用 `ros2 launch rosorin_description gazebo.launch.py use_gui:=false` 做短时验证时，`gzserver`、`robot_state_publisher` 和 `spawn_entity.py` 均成功启动，且机器人实体已成功生成到 Gazebo 世界中。

说明阶段 A 的最小基础环境已经基本打通，可以继续进入阶段 B。

---

## 阶段 B：生成机器人并打通基础仿真话题

### B.1 目标

把机器人成功 spawn 到 Gazebo 中，并让仿真提供导航需要的核心基础话题。

### B.2 需要完成的事项

1. 在 Gazebo launch 中使用 `rosorin.gazebo.xacro`
2. 启动 `robot_state_publisher`
3. 使用 `spawn_entity.py` 把机器人加入世界
4. 检查以下话题是否存在：
   - `/tf`
   - `/tf_static`
   - `/odom`
   - `/scan` 或 `/scan_raw`
   - `/imu_data` 或 `/imu`
5. 检查机器人是否响应 `cmd_vel`

### B.3 当前重点

建议先采用“最小仿真链路”，不要一上来就完全复刻实车：

- Gazebo 直接产出 `/odom`
- Gazebo 直接产出 `/scan`
- Gazebo 直接产出 `/imu`

先证明：

> 仿真机器人能动、能出传感器、能形成基本 TF。

### B.4 验收标准

满足以下条件即可通过：

- Gazebo 中能看见机器人
- `ros2 topic list` 中能看见核心仿真话题
- 向速度话题发送速度命令后，机器人会在 Gazebo 中移动

### B.5 风险点

- `rosorin.gazebo.xacro` 的参数与 launch 中实际传入参数不一致
- frame 名与现有导航要求不一致
- 插件发布的话题名与现有工程配置不一致

### 阶段 B 当前已完成记录

本阶段已于 2026-03-10 全部验证通过，具体情况：

**已完成的修改：**
- 将 `rosorin.gazebo.xacro` 中的激光插件从 ROS1 风格 `libgazebo_ros_laser.so` 升级为 ROS2 `libgazebo_ros_ray_sensor.so`，并配置 `<output_type>sensor_msgs/LaserScan</output_type>`
- 将 IMU 插件从 `libgazebo_ros_imu.so` 升级为 `libgazebo_ros_imu_sensor.so`
- 为所有三个插件添加了 ROS2 风格的 `<ros>` 配置块（含 `<remapping>` 标签）
- planar_move 插件配置了 `cmd_vel:=controller/cmd_vel` 和 `odom:=odom` 重映射

**关键发现：**
- `imu_link` 和 `lidar_sim_frame` 在 URDF 中没有 `<inertial>` 属性，URDF→SDF 转换时会被合并到 `base_footprint` link 中，但传感器定义会被正确保留
- `planar_move` 插件是 lazy publisher — 只在收到第一条 `cmd_vel` 后才开始发布 `/odom` 话题

**验证结果：**

| 话题 | 消息类型 | frame_id | 状态 |
|------|----------|----------|------|
| `/scan_raw` | `sensor_msgs/LaserScan` | `lidar_frame` | ✅ 360 rays，空世界全 `.inf`，正常 |
| `/imu` | `sensor_msgs/Imu` | `base_footprint` | ✅ z 轴加速度 = 9.8 m/s²，正常 |
| `/odom` | `nav_msgs/Odometry` | `odom` → `base_footprint` | ✅ 发 cmd_vel 后位置随之变化 |
| `/controller/cmd_vel` | 订阅端 | — | ✅ planar_move 正确订阅并响应 |
| `/tf` | `tf2_msgs/TFMessage` | — | ✅ odom→base_footprint 由 planar_move 发布 |
| `/tf_static` | `tf2_msgs/TFMessage` | — | ✅ 由 robot_state_publisher 发布 |
| `/clock` | `rosgraph_msgs/Clock` | — | ✅ Gazebo 仿真时钟 |

**ROS 节点验证：**
- `/gazebo` — gzserver 主节点
- `/gazebo_rplidar` — 激光传感器插件节点
- `/imu_plugin` — IMU 传感器插件节点
- `/robot_state_publisher` — TF 静态树
- `/joint_state_publisher` — 关节状态

**gzserver 已加载的插件库（通过 `/proc/<pid>/maps` 确认）：**
- `libgazebo_ros_init.so` ✅
- `libgazebo_ros_factory.so` ✅
- `libgazebo_ros_force_system.so` ✅
- `libgazebo_ros_ray_sensor.so` ✅
- `libgazebo_ros_imu_sensor.so` ✅

阶段 B 全部验收标准满足，可以继续进入阶段 C。

---

## 阶段 C：统一仿真话题与现有导航链路

### C.1 目标

让 Gazebo 提供的话题尽量和当前仓库已有导航链路兼容。

### C.2 需要完成的事项

1. 统一激光话题名
2. 统一 IMU 话题名
3. 确认 `cmd_vel` 订阅入口
4. 确认 `odom` 与 `base_frame` 的 frame 命名
5. 确认 TF 树是否满足导航需求

### C.3 推荐原则

优先让仿真侧向现有导航配置靠齐，例如：

- 尽量发布 `/odom`
- 尽量发布 `/scan_raw` 或统一 remap 到导航实际使用的话题
- 尽量发布 `/imu`
- 保证 `base_link` / `base_footprint` / `odom` / `map` 命名一致

### C.4 验收标准

满足以下条件即可通过：

- 现有导航包不需要大改就能接仿真数据
- 使用 `ros2 topic echo`、`tf2_tools` 或 RViz2 可以确认话题与 TF 正常

### C.5 风险点

- 实车 launch 默认依赖真实硬件节点，不适合直接复用
- 当前导航 launch 可能默认包含实车底盘、雷达、EKF 启动链，需要拆分出仿真版入口

---

## 阶段 D：新增仿真版导航入口

### D.1 目标

新建专门的仿真导航启动文件，使其不依赖真实硬件驱动，而是接入 Gazebo 仿真话题。

### D.2 建议新增文件

建议新增：

- `src/navigation/launch/navigation_sim.launch.py`

### D.3 这个文件应负责的内容

`navigation_sim.launch.py` 建议负责以下几部分：

1. 启动 Gazebo
2. 生成机器人模型
3. 启动地图服务器
4. 启动 AMCL
5. 启动 Nav2 基础节点
6. 启动 RViz2

### D.4 设计原则

不要直接复用实车版 `navigation.launch.py` 去拉起所有底盘与硬件节点，因为它默认更偏向：

- 实车雷达
- 实车 IMU
- 实车底盘
- 实车 EKF

仿真版入口应该做到：

- 明确只接 Gazebo 仿真话题
- 明确不依赖 `/dev/lidar` 等硬件设备
- 明确区分 `sim=true` 和 `sim=false`

### D.5 验收标准

满足以下条件即可通过：

- 一条命令能启动仿真导航链
- RViz2 能正常看到地图、激光、机器人
- `Nav2 Goal` 工具可用

---

## 阶段 E：跑通闭环导航仿真

### E.1 目标

实现完整的“设初始位姿 -> 发目标点 -> 机器人运动到目标点”的闭环。

### E.2 需要完成的事项

1. 启动地图与 AMCL
2. 在 RViz2 中设置初始位姿
3. 检查 `map -> odom -> base_link/base_footprint` 链是否成立
4. 检查是否存在 `navigate_to_pose` action
5. 用 `Nav2 Goal` 发送目标点
6. 验证机器人在 Gazebo 中实际运动

### E.3 验收标准

满足以下条件即可通过：

- RViz2 中能完成 `2D Pose Estimate`
- RViz2 中能发送 `Nav2 Goal`
- 机器人能在 Gazebo 世界中移动到目标区域
- 规划路径、局部控制、目标完成状态可见

### E.4 风险点

- 激光与地图坐标不一致
- 初始位姿设置后定位漂移严重
- Gazebo 的简化运动模型与 Nav2 控制参数不匹配
- `cmd_vel` 话题名或速度约束不一致

---

## 阶段 F：把仿真逐步逼近实车链路

### F.1 目标

在第一版闭环跑通后，逐步让仿真结构更贴近实车结构。

### F.2 可逐步引入的内容

1. 接入 `imu_filter`
2. 接入 `ekf_filter_node`
3. 评估是否在仿真中引入 `rf2o` 链路
4. 统一实车与仿真话题名
5. 统一 frame 命名
6. 统一 launch 参数风格

### F.3 原则

不是所有实车链路都必须立刻搬进仿真。应该遵循：

- 先闭环
- 再逼真
- 先验证功能
- 再提高结构一致性

---

## 5. 第一版建议采用的最小闭环方案

为了尽快看到成果，推荐第一版采用以下最小方案：

### 5.1 数据来源

- `odom`：直接由 Gazebo 平面运动插件提供
- `scan`：直接由 Gazebo 激光插件提供
- `imu`：直接由 Gazebo IMU 插件提供

### 5.2 导航组件

- `map_server`
- `amcl`
- `planner_server`
- `controller_server`
- `bt_navigator`
- `velocity_smoother`
- `rviz2`

### 5.3 暂时不强求的内容

第一版可以暂时不接入：

- 实车底盘驱动
- 实车串口硬件
- 复杂 EKF 融合
- 复杂雷达里程计中间层
- 语音模块
- 其他与导航闭环无关的感知功能

这能最大程度降低初期复杂度。

---

## 6. 每个阶段的验证方法

### 阶段 A 验证

- Gazebo 是否能启动
- world 是否正常显示

### 阶段 B 验证

- 机器人是否能生成
- 是否存在 `/odom`、`/scan`、`/tf`
- 发 `cmd_vel` 后机器人是否会动

### 阶段 C 验证

- 话题名是否与导航配置匹配
- frame 是否一致

### 阶段 D 验证

- 是否能一条命令起仿真导航链
- 是否不再依赖真实硬件设备

### 阶段 E 验证

- 是否能在 RViz2 中设初始位姿
- 是否能发 `Nav2 Goal`
- 是否能在 Gazebo 中看到机器人移动

### 阶段 F 验证

- 仿真链和实车链差异是否减小
- 同一套导航参数是否更容易复用

---

## 7. 后续优化路线

闭环仿真跑通后，后续优化建议按优先级推进：

### 7.1 第一优先级：可用性优化

- 统一一键启动脚本
- 明确仿真与实车模式切换参数
- 增加 README 使用说明
- 增加常见问题说明

### 7.2 第二优先级：结构一致性优化

- 统一仿真与实车的话题命名
- 统一 frame 体系
- 尽量复用实车 launch 结构

### 7.3 第三优先级：保真度优化

- 增强 Gazebo 世界复杂度
- 加入障碍物、狭窄通道、动态目标
- 调整传感器噪声模型
- 调整运动学参数与控制器参数

### 7.4 第四优先级：工程化优化

- 增加自动化测试脚本
- 增加 bag 回放验证流程
- 增加仿真回归测试清单
- 为后续算法实验建立标准基线场景

---

## 8. 建议的实施顺序

如果按照最小成本、最快见效的思路推进，建议顺序如下：

1. 先安装 Gazebo Classic 相关依赖
2. 新建 `empty.world`
3. 新建 `gazebo.launch.py`
4. 在 Gazebo 中生成机器人
5. 验证 `odom / scan / tf`
6. 新建 `navigation_sim.launch.py`
7. 接入地图与 Nav2
8. 在 RViz2 中完成 `2D Pose Estimate`
9. 在 RViz2 中完成 `Nav2 Goal`
10. 机器人在 Gazebo 中完成闭环导航
11. 再逐步引入 EKF、IMU filter、更多实车一致性优化

---

## 9. 当前建议的下一步实际动作

从当前状态出发，最建议的下一批落地工作是：

### 9.1 优先新增的文件

- `src/simulations/rosorin_description/worlds/empty.world`
- `src/simulations/rosorin_description/launch/gazebo.launch.py`
- `src/navigation/launch/navigation_sim.launch.py`

### 9.2 第一轮目标

第一轮不要追求复杂，先达到：

- Gazebo 世界打开
- 机器人生成成功
- 仿真话题存在
- RViz2 能看见机器人和地图

### 9.3 第二轮目标

在第一轮成功的基础上，再实现：

- AMCL 初始定位
- Nav2 Goal
- 机器人在 Gazebo 中自主到达目标点

---

## 10. 一句话总结

当前仓库已经具备了机器人描述、导航配置和 Gazebo 插件的基础，因此 Gazebo 闭环仿真并不是从零开始，而是需要把“世界、生成、话题统一、仿真导航入口、闭环验证”这几块补齐。最合理的路线是：

> 先搭出最小可运行仿真，再跑通闭环导航，最后逐步向实车链路逼近并持续优化。
