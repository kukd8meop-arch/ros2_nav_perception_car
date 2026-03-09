# ROS2 机器人项目知识扫盲

> 这份文档用于总结本次对话中涉及到的核心知识，帮助从“完全没思路”快速进入“知道每层在干什么、出了问题去哪里看”的状态。

---

## 1. 先建立总思路：机器人系统到底是在搭什么

如果你从零看这种 ROS2 机器人项目，很容易被一堆 `launch.py`、`yaml`、`xacro`、`Node` 吓住。

其实整套系统可以浓缩成一句话：

**先让轮子转起来，再让机器人知道自己动了多少，再让它感知外界，最后完成建图和导航。**

按功能拆开就是：

1. **底盘驱动**：接收速度指令，控制电机旋转。
2. **里程计**：根据编码器推算机器人移动了多少。
3. **IMU 处理**：得到更稳定的角速度、姿态信息。
4. **EKF 融合**：把轮速里程计和 IMU 融起来，得到更可靠的 `/odom`。
5. **雷达驱动**：读取雷达数据，发布 `/scan` 或 `/scan_raw`。
6. **SLAM / Navigation**：建图、定位、规划路径、控制机器人走到目标点。

所以你现在看到的复杂工程，本质上只是把这 6 类功能通过 ROS2 组织起来。

---

## 2. launch 文件到底是什么

### 2.1 `launch.py` 不是业务算法，而是“启动清单”

ROS2 的 `launch.py` 文件，核心职责不是处理传感器数据，也不是控制电机。
它更像是一个“系统编排脚本”：

- 启动哪些节点
- 每个节点用什么参数
- 节点之间的话题名怎么重映射
- 是否延时启动
- 是否继续嵌套包含别的 launch 文件

所以你看见很多：

- `IncludeLaunchDescription`
- `DeclareLaunchArgument`
- `LaunchConfiguration`
- `Node(...)`
- `return LaunchDescription([...])`

这些都属于“告诉 ROS2：该把哪些模块拉起来”。

### 2.2 为什么叫 `.launch.py`

这是 ROS2 社区的命名习惯：

- `.py`：说明它是 Python 写的
- `.launch`：说明它是启动文件，不是普通脚本

所以 `slam.launch.py` 一眼就能看出它是 “SLAM 的入口启动文件”。

### 2.3 launch 的调用层级

在这个工程里，launch 的结构大致是：

- `slam.launch.py` / `navigation.launch.py`：系统入口
- `robot.launch.py`：基座层，把底盘、传感器、EKF 组装起来
- `controller.launch.py`、`lidar.launch.py`、`imu_filter.launch.py`：模块层
- 具体 `Node(...)`：真正干活的执行节点

你可以把它理解为：

**入口 launch 只是总导演，真正干活的是最底层 node。**

---

## 3. 谁才是真正“执行代码”的地方

很多 launch 文件最终只是为了启动下面这些真正的节点：

- `ros_robot_controller`
- `odom_publisher`
- `imu_filter`
- `ekf_filter_node`
- `ldlidar_stl_ros2_node`
- `sync_slam_toolbox_node`

其中最值得重点理解的是：

### 3.1 `ros_robot_controller`

作用：

- 订阅速度指令（如 `/cmd_vel`）
- 通过串口给底层控制板发电机控制命令
- 读取编码器、IMU 原始数据
- 发布底层传感器数据

这是真正接触硬件的部分。

### 3.2 `odom_publisher`

作用：

- 读取轮速 / 编码器信息
- 用麦轮或阿克曼运动学模型换算出机器人速度
- 积分得到里程计
- 发布 `/odom_raw`

### 3.3 `ekf_filter_node`

来自 `robot_localization` 包。

作用：

- 订阅 `/odom_raw`、`/imu` 等传感器输入
- 用 EKF 融合成更稳定的 `/odom`
- 发布 `odom -> base_footprint` 的 TF

### 3.4 `ldlidar_stl_ros2_node`

这是雷达厂家的 ROS2 驱动节点。

作用：

- 打开雷达对应串口
- 解析雷达原始数据
- 发布 ROS2 标准的 `LaserScan`

### 3.5 `sync_slam_toolbox_node`

来自 `slam_toolbox` 包。

作用：

- 订阅 `/scan_raw`、`/odom`、`/tf`
- 构建地图
- 发布 `/map` 和 `map -> odom` TF

---

## 4. `robot_description.launch.py` 在干什么

这个文件**不负责控制电机**。

它做的是两件事：

1. 读取 `xacro/URDF`
2. 让 ROS2 知道机器人各个部件之间的空间关系

对应的关键节点是：

- `robot_state_publisher`
- `joint_state_publisher`

### 4.1 `robot_state_publisher` 干什么

它会解析 xacro/URDF 中每个 link 和 joint 的关系，然后发布 TF。

比如：

- `base_link -> laser_link`
- `base_link -> imu_link`
- `base_link -> wheel_link`

这些关系如果是固定不变的，就会通过静态 TF 发布出去。

### 4.2 `joint_state_publisher` 干什么

它负责发布关节状态，比如轮子转了多少角度。

这些关节状态再配合 URDF 模型，就可以让 `robot_state_publisher` 推出动态的部件姿态变化。

---

## 5. TF 从数学上到底是什么

TF 的本质是一棵**坐标变换树**。

每一条边，本质上都是一个刚体变换：

$$
T = \begin{bmatrix}
R & t \\
0 & 1
\end{bmatrix}
$$

其中：

- $R$：旋转矩阵
- $t$：平移向量

### 5.1 直观理解

例如：

- 雷达装在底盘前方 10 cm、高 15 cm
- 那么 `base_link -> laser_link` 就是一条固定变换

也就是说：

**雷达测到的数据，最初是在雷达自己的坐标系里；要拿去建图，必须一步一步变换回机器人坐标系、odom 坐标系、map 坐标系。**

### 5.2 一条典型计算链

当 SLAM 需要把激光点从雷达坐标系变成地图坐标系时，本质上做的是：

$$
P_{map} = T_{map}^{odom} \cdot T_{odom}^{base} \cdot T_{base}^{laser} \cdot P_{laser}
$$

所以：

- `T_base^laser`：来自 URDF/xacro，安装位置决定
- `T_odom^base`：来自 EKF 或里程计
- `T_map^odom`：来自 SLAM 或定位模块

这就是为什么 URDF 里的尺寸、雷达安装位置、IMU 位置都不能瞎填。

---

## 6. xacro / URDF 是什么关系

### 6.1 URDF 是机器人结构描述

URDF 描述的是：

- 机器人有哪些部件（link）
- 部件之间如何连接（joint）
- 每个部件的几何尺寸和相对位置

### 6.2 xacro 是 URDF 的“宏版本”

xacro 允许：

- 写变量
- 写宏
- include 其他文件
- 参数化生成 URDF

所以实际工程里一般不是手写巨大 URDF，而是写 xacro，再在 launch 里通过：

```python
robot_description = Command(['xacro ', urdf_path])
```

动态展开成 URDF 给 `robot_state_publisher` 使用。

---

## 7. 电机控制到底是怎么接上的

### 7.1 不是 `robot_description` 控电机

真正控制电机的链路是：

```text
/cmd_vel
  ↓
ros_robot_controller_node.py
  ↓
ros_robot_controller_sdk.py
  ↓
串口协议
  ↓
STM32 单片机
  ↓
电机驱动器
  ↓
轮子转动
```

### 7.2 `ros_robot_controller_sdk.py` 在做什么

这个文件是一个**底层串口 SDK**。

它的职责是：

- 把高层命令打包成二进制串口协议
- 发给 STM32
- 从 STM32 收取回传数据
- 解析成 Python 可以理解的数值

例如：

- `set_motor_speed(...)`：设置电机转速
- `get_imu()`：读取 IMU 原始数据
- `get_gamepad()`：读取手柄数据

### 7.3 串口协议怎么组织

大致格式是：

```text
0xAA 0x55 | 功能码 | 数据长度 | 数据体 | CRC8
```

其中：

- `0xAA 0x55`：帧头
- 功能码：比如 MOTOR、IMU、SERVO
- 数据体：具体的电机速度、IMU 数据等
- CRC8：校验

也就是说，Python 并不直接驱动电机，它只是通过串口把命令送给 STM32，真正的 PWM、电机驱动都在单片机固件那边完成。

---

## 8. `ros2_control` 和这个工程的关系

你提到过 ROS 官方的控制框架，其实你说的是 **`ros2_control`**。

### 8.1 `ros2_control` 是什么

这是 ROS2 官方的标准硬件抽象层。

正规用法一般是：

- 写 `HardwareInterface`
- 接入 `controller_manager`
- 使用标准控制器插件，例如：
  - `diff_drive_controller`
  - `joint_trajectory_controller`

### 8.2 这个工程有没有用 `ros2_control`

**没有。**

这个工程采用的是更直接的方式：

- 自己写 Python 节点
- 自己连串口
- 自己收发电机和 IMU 数据
- 自己发布关节状态 / 里程计

### 8.3 为什么厂家常这么做

因为：

- 上手快
- 不用写复杂的硬件接口插件
- 对小车项目来说够用

代价是：

- 标准化程度低一些
- 可复用性比 `ros2_control` 差

所以你现在看到的是一个“自定义驱动方案”，不是 ROS2 官方控制栈方案。

---

## 9. 雷达 SDK 到底是什么，怎么接进来的

### 9.1 SDK 是什么

SDK（Software Development Kit）就是厂家给你的驱动包或开发包。

它已经帮你写好了：

- 如何打开雷达串口
- 如何按私有协议解析雷达数据
- 如何转成 ROS2 消息

所以你不需要从零解析雷达每一帧原始字节流。

### 9.2 这个工程里的雷达驱动怎么接入

在 launch 文件里会写：

```python
Node(
    package='ldlidar_stl_ros2',
    executable='ldlidar_stl_ros2_node',
    parameters=[{
        'port_name': '/dev/lidar',
        'port_baudrate': 230400,
    }],
)
```

这说明：

- `package='ldlidar_stl_ros2'`：使用厂家或第三方提供的 ROS2 包
- `executable='ldlidar_stl_ros2_node'`：启动其中的驱动节点
- `port_name='/dev/lidar'`：告诉驱动去打开哪个串口设备

### 9.3 源码一定在你仓库里吗

不一定。

有两种情况：

1. 驱动源码在你的工作区源码包里
2. 驱动已经通过 `apt` 或系统安装好了，你的工程只是调用它

在你这个项目里，LD19 驱动节点更像是后者：工程里只有 launch 引用，不一定把完整驱动源码放在仓库里。

---

## 10. `/dev/lidar` 究竟是什么意思

这是 Linux 设备文件机制。

### 10.1 串口不是“只在单片机上”

串口是一种**通信接口**，两边都存在：

- 雷达 / 单片机那边有硬件串口
- Jetson 这边会把串口设备映射成 `/dev/ttyUSB0`、`/dev/ttyACM0` 等设备文件

程序只要打开这个设备文件，就相当于和硬件串口通信。

### 10.2 为什么不是直接写 `/dev/ttyUSB0`

因为 Linux 每次开机时，设备编号可能会变：

- 今天雷达是 `/dev/ttyUSB0`
- 明天可能就成了 `/dev/ttyUSB1`

这会导致 launch 配置不稳定。

### 10.3 为什么会有 `/dev/lidar`

这是通过 **udev 规则**创建的固定别名。

规则的意思通常是：

- 如果某个设备插在指定的 USB 口上
- 或者具有某个厂商 ID / 产品 ID
- 就自动给它创建一个软链接别名，例如 `/dev/lidar`

这样程序永远只用打开 `/dev/lidar`，不关心它底层实际是 `/dev/ttyUSB0` 还是 `/dev/ttyUSB1`。

### 10.4 这意味着什么

对于雷达而言：

**最好固定插在 Jetson 的同一个 USB 物理口上。**

因为有些 udev 规则是按 USB 物理端口位置匹配的，换口之后别名可能就失效。

### 10.5 底盘控制板 `/dev/rrc`

类似地，底盘单片机也会被映射成一个固定别名，例如 `/dev/rrc`。

这样 `Board(device="/dev/rrc")` 就不用每次跟着真实串口号变化而改代码。

---

## 11. EKF 配置文件怎么看

`ekf.yaml` 第一次看会很吓人，但你真正要抓住的是三层：

### 11.1 第一层：输入源是谁

比如：

- `odom0: odom_raw`
- `odom1: odom_rf2o`
- `imu0: imu`

这表示 EKF 的输入源来自哪些话题。

### 11.2 第二层：每个输入贡献哪些状态量

最核心的是 `_config` 数组。

15 个布尔值对应：

```text
x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az
```

它们的含义和常见传感器来源可以这样理解：

- `x`：机器人在坐标系中的 x 方向位置，通常来自轮式里程计、激光里程计、视觉里程计、GPS
- `y`：机器人在坐标系中的 y 方向位置，通常来自轮式里程计、激光里程计、视觉里程计、GPS
- `z`：机器人在坐标系中的 z 方向位置，高度相关，地面机器人一般很少用，可能来自视觉、GPS、3D 激光雷达
- `roll`：绕 x 轴的转角，也就是横滚角，通常来自 IMU 姿态解算
- `pitch`：绕 y 轴的转角，也就是俯仰角，通常来自 IMU 姿态解算
- `yaw`：绕 z 轴的转角，也就是航向角，通常来自 IMU、激光里程计、视觉里程计、磁罗盘
- `vx`：x 方向线速度，通常来自编码器里程计、视觉里程计、激光里程计
- `vy`：y 方向线速度，麦轮/全向底盘会特别关注，通常来自编码器里程计、视觉里程计、激光里程计
- `vz`：z 方向线速度，地面机器人通常不用，常见于无人机或 3D 运动平台
- `vroll`：绕 x 轴角速度，通常来自 IMU 的陀螺仪
- `vpitch`：绕 y 轴角速度，通常来自 IMU 的陀螺仪
- `vyaw`：绕 z 轴角速度，也就是偏航角速度，通常来自 IMU 的陀螺仪，也可能来自轮式里程计估算
- `ax`：x 方向线加速度，通常来自 IMU 加速度计
- `ay`：y 方向线加速度，通常来自 IMU 加速度计
- `az`：z 方向线加速度，通常来自 IMU 加速度计

你可以把这些量再按“谁最常提供它们”粗略分组：

- **编码器/轮式里程计**：常提供 `vx`、`vy`、`vyaw`，有时也能间接积分出 `x`、`y`
- **IMU**：最常提供 `roll`、`pitch`、`yaw`、`vroll`、`vpitch`、`vyaw`、`ax`、`ay`、`az`
- **激光里程计 / 视觉里程计**：常提供 `x`、`y`、`z`、`yaw` 以及部分速度量
- **GPS / RTK**：常提供全局位置 `x`、`y`、`z`，但一般不会直接提供姿态和角速度

- `true`：使用这个量
- `false`：忽略这个量

例如：

- 轮速里程计通常提供 `vx`、`vy`、`vyaw`
- IMU 通常提供 `yaw`、`vyaw`

### 11.3 第三层：坐标系设置

例如：

- `map_frame`
- `odom_frame`
- `base_link_frame`
- `world_frame`

单机器人场景下，经常需要把 `namespace/...` 改成实际坐标系名，例如：

- `map`
- `odom`
- `base_footprint`

---

## 12. EKF 原理学了到底有没有用

有用，而且很有用，只是不是体现在“你亲手写了 EKF 代码”，而是体现在：

### 12.1 你能看懂配置背后的意义

学过 EKF 原理后，你就知道：

- `frequency`：滤波器更新频率
- `process_noise_covariance`：过程噪声协方差，对应 $Q$
- `initial_estimate_covariance`：初始误差协方差，对应 $P_0$
- 为什么某些传感器只给速度、不直接给位置
- 为什么 yaw 配错后，x/y 位姿也会漂

### 12.2 你出了问题知道怎么调

比如：

- 机器人转弯后位置发散
- 横移估计不准
- 融合后的 `/odom` 抖动

懂原理的人知道去检查：

- 哪个传感器状态量开错了
- 协方差是不是设置不合理
- 哪个话题坐标系不对

不懂原理的人就只能“蒙着改参数”。

### 12.3 工程上最重要的是“会调包”

现实工程里，大多数时候不是你从零写 EKF，而是：

- 选对成熟包
- 知道它为什么这样配置
- 出问题能定位
- 能让它在真实机器人上稳定运行

这本身就是很硬的能力。

---

## 13. 为什么说“有现成工程反而是最好的学习状态”

你现在不是“白学了”，而是在用一种更高效的方式学：

- 不是空对空学算法
- 而是对着真实工程去理解每一层
- 每学一个知识点，都能立刻找到它在工程里的落点

例如：

- 学 TF → 看 `robot_state_publisher`
- 学运动学 → 看 `mecanum.py`
- 学串口协议 → 看 `ros_robot_controller_sdk.py`
- 学 EKF → 看 `ekf.yaml`
- 学设备映射 → 看 `lidar.rules`

这比只看教材更接近实际研发。

---

## 14. 整个系统里最该先掌握的主线

如果你后面继续深入，建议按这个顺序掌握：

### 第 1 步：先把 launch 树看明白

你要知道：

- 哪个 launch 是入口
- 谁 include 了谁
- 最后真正启动了哪些 node

### 第 2 步：把数据流看明白

重点看：

- `/cmd_vel`
- `/odom_raw`
- `/imu`
- `/odom`
- `/scan_raw`
- `/tf`
- `/map`

### 第 3 步：把硬件接入点看明白

重点看：

- `/dev/rrc`
- `/dev/lidar`
- udev 规则
- 串口参数

### 第 4 步：把 TF 树看明白

重点看：

- `base_footprint`
- `base_link`
- `laser_link`
- `imu_link`
- `odom`
- `map`

### 第 5 步：把 EKF 和 SLAM 看明白

重点看：

- `/odom_raw` 和 `/odom` 的区别
- `odom -> base` 谁来发
- `map -> odom` 谁来发
- SLAM 为什么依赖前面的所有链路

---

## 15. 最后给自己的一个判断标准

如果你后面能回答清楚下面这些问题，就说明你已经不是“完全没思路”了：

1. `slam.launch.py` 最终启动了哪些真正干活的节点？
2. `/cmd_vel` 是怎么一路传到电机上的？
3. `robot_description.launch.py` 为什么不控制电机？
4. TF 树在数学上是什么？为什么 SLAM 必须依赖 TF？
5. xacro 和 URDF 的关系是什么？
6. `/dev/lidar` 为什么不是系统自动原生的名字？
7. udev 规则为什么要求某些设备固定插在同一个 USB 口？
8. 这个工程为什么没用 `ros2_control`？
9. `ekf.yaml` 里最关键该先看哪几项？
10. 学过 EKF 原理后，在工程里到底用在什么地方？

如果这些你都能说顺，那你就已经完成了“扫盲”阶段，后面就可以开始真正的部署、调参与排错了。

---

## 16. 建议的下一步

建议你接下来进入“从理解到实操”的阶段，按这个顺序动手：

1. 确认环境变量（如 `LIDAR_TYPE`）
2. 检查 `/dev/rrc` 和 `/dev/lidar` 是否正常映射
3. 检查 TF 树是否完整
4. 单独验证 `/odom_raw`、`/imu`、`/scan_raw`
5. 再启动 EKF 看 `/odom`
6. 最后再跑 SLAM 或导航

---

## 17. 一句话总复盘

**你现在学到的，不只是“怎么调用厂家现成代码”，而是在学一整套真实机器人系统是如何被分层组织、如何把硬件接进 ROS2、如何把数学模型落到工程配置里的。**
