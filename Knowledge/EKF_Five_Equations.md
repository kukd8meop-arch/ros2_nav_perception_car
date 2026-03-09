# EKF 五大公式在本工程中的实现与应用

本文不是单纯讲教科书里的扩展卡尔曼滤波（EKF），而是专门结合本工程来说明：

- EKF 节点是谁启动的
- 它到底在融合哪些传感器
- 五大公式分别落在工程的哪个环节
- 最终为什么会输出 `/odom` 和 `odom -> base_footprint` 这条 TF

---

## 1. 本工程里 EKF 在哪里

本工程的 EKF 由 `robot_localization` 包提供，节点名是 `ekf_filter_node`。

对应启动文件：

- `src/driver/controller/launch/controller.launch.py`

其中最关键的一段是：

```python
ekf_filter_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_param, {'use_sim_time': use_sim_time}],
    remappings=[
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('odometry/filtered', 'odom'),
        ('cmd_vel', 'controller/cmd_vel')
    ],
    condition=IfCondition(enable_odom),
)
```

这说明：

- 本工程真正运行的是 `robot_localization/ekf_node`
- 它的输出话题 `odometry/filtered` 被重映射成了 `/odom`
- 当 `publish_tf: true` 时，它还会发布 `odom -> base_footprint` 这条 TF
- 它读取的参数文件是：`src/driver/controller/config/ekf.yaml`

所以，从工程角度看，EKF 的位置非常明确：

> 它处在“底盘原始里程计 / IMU / 激光里程计”与“上层 SLAM/Navigation”之间，负责产出更稳定的机器人局部位姿估计。

---

## 2. 本工程里 EKF 的输入和输出

根据 `src/driver/controller/config/ekf.yaml`，本工程的 EKF 主要配置如下：

```yaml
odom0: odom_raw
odom1: odom_rf2o
imu0: imu
publish_tf: true
world_frame: odom
odom_frame: odom
base_link_frame: base_footprint
```

这意味着本工程中 EKF 的数据流大致是：

```text
轮速/底盘编码器  --->  odom_publisher  ---> /odom_raw ---\
                                                     \
IMU 原始数据 ---> imu_filter/calib ---> /imu -----------> ekf_filter_node ---> /odom
                                                     /
激光匹配里程计 RF2O -----------------> /odom_rf2o ----/

同时发布 TF: odom -> base_footprint
```

在继续往下看 `odom0_config`、`odom1_config`、`imu0_config` 之前，必须先补一个关键数学桥梁：

这些布尔数组并不是“经验参数表”，它们本质上是在描述 **观测矩阵** $\mathbf{H}$。

---

## 2.0 为什么这里会引出 $\mathbf{H}$ 观测矩阵

EKF 的测量更新并不是直接拿传感器值往状态里“硬塞”，而是先写成统一的观测方程：

$$
\mathbf{z}_k = h(\mathbf{x}_k) + \mathbf{v}_k
$$

如果在线性化之后写成局部线性形式，就是：

$$
\mathbf{z}_k \approx \mathbf{H}_k \mathbf{x}_k + \mathbf{v}_k
$$

其中：

- $\mathbf{x}_k$：EKF 当前维护的状态向量
- $\mathbf{z}_k$：某一路传感器当前给出的测量向量
- $\mathbf{H}_k$：观测矩阵，表示“这路传感器到底观测了状态里的哪些量”
- $\mathbf{v}_k$：测量噪声

本工程里 `robot_localization` 的状态向量顺序固定为：

$$
\mathbf{x} =
[x, y, z, roll, pitch, yaw, v_x, v_y, v_z, \omega_x, \omega_y, \omega_z, a_x, a_y, a_z]^T
$$

因此，只要你先回答这个问题：

> “这一路传感器到底直接、可靠地测到了状态向量里的哪些分量？”

就能反推出它的观测模型，也就能反推出对应的 $\mathbf{H}$。

---

## 2.0.1 从 `config` 掩码到 $\mathbf{H}$ 的关系

例如：

```yaml
odom0_config: [false, false, false,
               false, false, false,
               true, true, false,
               false, false, true,
               false, false, false]
```

它的意思不是“随手勾了三个 true”，而是在说：

- `odom0` 这一路只参与观测 `v_x`
- `odom0` 这一路只参与观测 `v_y`
- `odom0` 这一路只参与观测 `\omega_z`

于是，这一路测量向量就可以写成：

$$
\mathbf{z}_{odom0} =
\begin{bmatrix}
v_x \\
v_y \\
\omega_z
\end{bmatrix}
$$

对应的观测矩阵本质上就是“从 15 维状态里把第 7、8、12 个量挑出来”：

$$
\mathbf{H}_{odom0} =
\begin{bmatrix}
0&0&0&0&0&0&1&0&0&0&0&0&0&0&0 \\
0&0&0&0&0&0&0&1&0&0&0&0&0&0&0 \\
0&0&0&0&0&0&0&0&0&0&0&1&0&0&0
\end{bmatrix}
$$

所以：

- **数学上**：`odom0_config` 对应观测矩阵 $\mathbf{H}$ 的列选择
- **工程上**：它是 `robot_localization` 为了方便配置而提供的布尔掩码写法
- **理解上**：它在声明“这一类传感器对哪些状态量有发言权”

也正因为如此，后面你看到的 `odom0_config`、`odom1_config`、`imu0_config`，本质上都不是孤立参数，而是在分别定义三路传感器的观测模型。

### 2.1 `odom0: odom_raw`

`odom_raw` 通常来自底盘运动学/轮速积分，是最基础的局部里程计。

在本工程中，这一路被配置为：

```yaml
odom0_config: [false, false, false,
               false, false, false,
               true, true, false,
               false, false, true,
               false, false, false]
```

这表示 EKF 主要从 `odom_raw` 使用：

- `vx`：机器人在机体前向轴上的线速度，简单理解就是“前后走得多快”
- `vy`：机器人在机体侧向轴上的线速度，简单理解就是“左右横移得多快”
- `vyaw`：机器人绕竖直轴的角速度，也就是“转向转得多快”；从物理上更常写作 `\omega_z`

也就是说，这一路主要提供“速度信息”，而不是直接提供绝对位置。

这里要特别注意：

- `vx`、`vy` 属于**线速度**
- `vyaw` 属于**角速度**
- 它们描述的是“此刻运动得多快”，不是“此刻在什么位置”

所以 `odom0` 在本工程里的角色，更像是在告诉 EKF：

> 机器人此刻的底盘运动趋势是什么，而不是机器人在地图上的绝对位置在哪里。

---

### 2.2 `odom1: odom_rf2o`

这一项表示激光里程计（常见是基于激光扫描匹配得到的 2D 位姿增量）。

配置为：

```yaml
odom1_config: [true, true, false,
               false, false, true,
               false, false, false,
               false, false, false,
               false, false, false]
odom1_relative: true
```

这说明它主要提供：

- `x`：机器人在局部平面坐标系中的前后方向位置
- `y`：机器人在局部平面坐标系中的左右方向位置
- `yaw`：机器人在平面内的朝向角，也就是“车头朝哪边”

也就是“平面位姿信息”。

这里的“位姿”可以拆成两部分理解：

- `x`、`y` 是**位置**
- `yaw` 是**姿态中的平面朝向**

因此 `odom1` 提供的不是“速度趋势”，而是：

> 机器人在二维平面里当前大概位于哪里、面朝哪个方向。

另外：

- `odom1_relative: true` 表示这一路激光里程计更强调“相对起点的位姿变化”
- 所以它特别适合拿来约束累计漂移，而不是充当全局地图坐标

因此可以把它理解为：

- `odom_raw` 更擅长反映底盘瞬时速度
- `odom_rf2o` 更擅长约束平面位姿漂移

---

### 2.3 `imu0: imu`

IMU 在本工程中并不是把所有姿态都塞给 EKF，而是有选择地使用：

```yaml
imu0_config: [false, false, false,
              false, false, true,
              false, false, false,
              false, false, true,
              false, false, false]
imu0_relative: true
```

这表示主要使用：

- `yaw`：机器人当前的航向角，表示车头此刻朝向哪一个方向
- `vyaw`：机器人绕竖直轴的角速度，表示机器人此刻正在以多快的速度旋转

可以把这两个量理解成：

- `yaw` 回答“现在朝哪儿”
- `vyaw` 回答“现在正在多快地转”

这里之所以不用 IMU 去主导 `x`、`y`，是因为：

- IMU 更擅长感知角速度、姿态变化
- 仅靠 IMU 去积分位置很容易漂移
- 所以在本工程里，IMU 主要负责“朝向”和“转向动态”这部分信息

另外：

- `imu0_relative: true` 表示 EKF 更看重 IMU 相对初始时刻的姿态变化
- 这很适合地面机器人做局部连续航向估计

换句话说，IMU 在本工程 EKF 里最核心的作用是：

- 给出航向角约束
- 给出角速度约束
- 帮助抑制仅靠轮速积分带来的转向漂移

---

### 2.4 输出 `/odom`

`ekf_filter_node` 的融合结果输出为 `/odom`，它是比 `/odom_raw` 更稳、更适合上层使用的局部里程计。

后面的模块会依赖它：

- `slam_toolbox` 在建图时需要它
- `AMCL` / `Nav2` 在导航时也依赖这条局部连续坐标参考
- TF 树中的 `odom -> base_footprint` 也通常由它发布

所以可以简单理解成：

- `/odom_raw`：底盘原始估计
- `/odom`：EKF 融合后的工程级局部位姿

---

## 3. 本工程里 EKF 实际在估计什么状态

`robot_localization` 的标准状态向量是 15 维：

$$
\mathbf{x} =
[x, y, z, roll, pitch, yaw, v_x, v_y, v_z, \omega_x, \omega_y, \omega_z, a_x, a_y, a_z]^T
$$

在 `ekf.yaml` 中也明确给出了这个顺序：

- `x, y, z`
- `roll, pitch, yaw`
- `vx, vy, vz`
- `vroll, vpitch, vyaw`
- `ax, ay, az`

但是本工程设置了：

```yaml
two_d_mode: true
```

这里非常容易和前面的 `odom0_config`、`odom1_config`、`imu0_config` 混淆，但它们其实不是一回事。

### 3.1 `two_d_mode` 和 `*_config` 的区别

- `odom0_config`、`odom1_config`、`imu0_config` 这些布尔数组，回答的是：**某一路传感器要不要去观测某个状态量**
- `two_d_mode: true` 回答的是：**整个滤波器要不要按二维地面机器人模型来运行**

也就是说：

- `*_config` 决定的是某一路传感器对应的观测矩阵 $\mathbf{H}$ 选哪些列
- `two_d_mode` 决定的是整个 EKF 的状态传播、约束方式、以及对三维自由度的总体处理策略

所以 `*_config` 更像是在说：

> “这路传感器能测什么？”

而 `two_d_mode` 更像是在说：

> “这个机器人所处的世界，本来就是二维平面运动，不应该把自己当成会飞、会上下起伏、会左右翻滚的三维系统。”

### 3.2 为什么把三维相关观测都设成 `false` 还不够

很多人第一次看 `ekf.yaml` 时都会有这个疑问：

> “既然 `z`、`roll`、`pitch` 这些量在各个 `config` 里都没有启用，那为什么还需要 `two_d_mode: true`？”

原因是：

- `false` 只表示**这一路传感器不拿这个量来更新状态**
- 但这并不等于 **EKF 内部就完全没有这些状态维度**

`robot_localization` 内部仍然维护完整的 15 维状态：

- `z`
- `roll`
- `pitch`
- `vz`
- `vroll`
- `vpitch`
- 以及相关加速度项

即使这些量没有被某一路观测直接更新，它们仍然会在以下环节里“活着”：

- 预测步骤会继续传播这些状态
- 协方差矩阵会继续传播这些维度的不确定性
- IMU 或其他传感器里的微小三维噪声，仍可能通过系统模型耦合进来

也就是说：

> `config = false` 只是“没人专门测它”，并不等于“它在滤波器里被彻底消失掉了”。

### 3.3 `two_d_mode: true` 实际在干什么

`two_d_mode: true` 的本质作用，就是强制告诉 EKF：

- 这是一个二维平面机器人
- `z` 不应该乱漂
- `roll`、`pitch` 不应该成为主要运动自由度
- 三维相关状态不应该干扰平面定位结果

从工程效果上看，它相当于在做这些事情：

- 把 `z`、`roll`、`pitch` 及相关三维运动量压回二维假设下处理
- 避免机器人因为 IMU 小噪声、地面轻微不平或数值误差，就“估计出自己飞起来了”
- 降低三维自由度对 `x`、`y`、`yaw` 这些核心平面状态的干扰

因此，`two_d_mode` 不是重复配置，而是在**滤波器模型层面**施加二维约束。

### 3.4 用一句话区分两者

可以这样记：

- `*_config`：管的是“传感器说什么”
- `two_d_mode`：管的是“滤波器相信自己处在什么物理世界里”

所以这两个设置并不是重复，而是互补：

- 前者负责选择观测输入
- 后者负责约束系统模型

### 3.5 为什么本工程特别适合开 `two_d_mode`

本项目是典型地面移动机器人：

- 主要关心 `x`
- 主要关心 `y`
- 主要关心 `yaw`
- 主要关心线速度和偏航角速度

而不关心：

- `z` 的高度变化
- `roll` 的横滚姿态
- `pitch` 的俯仰姿态

因此开启 `two_d_mode: true` 后，工程上会更稳定：

- `/odom` 更平滑
- 航向估计更干净
- 更适合给 `slam_toolbox`、`AMCL`、`Nav2` 使用

因此在实际应用上，本工程重点关心的是二维移动机器人状态：

$$
[x, y, yaw, v_x, v_y, \omega_z]
$$

其余三维分量会被忽略、约束或不参与实际融合。

这也非常符合本项目的机器人类型：

- 它是地面小车，不是无人机
- 不需要重点估计 `z/roll/pitch`
- 重点在平面位置与朝向

---

## 4. EKF 五大公式在本工程里的对应关系

下面进入核心部分：五大公式不是孤立的数学式，而是在 `ekf_filter_node` 的运行循环里不断重复执行。

---

## 4.1 第一个公式：状态预测方程

EKF 的第一步是根据上一时刻状态，预测当前时刻状态：

$$
\hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1}, \mathbf{u}_k)
$$

其中：

- $\hat{\mathbf{x}}_{k-1}$：上一时刻最优状态估计
- $\hat{\mathbf{x}}_k^-$：当前时刻预测状态
- $f(\cdot)$：非线性运动模型
- $\mathbf{u}_k$：控制输入

### 在本工程中的含义

在本工程里，这一步可以理解为：

- EKF 先假设机器人会按照上一时刻的速度、角速度继续运动
- 根据时间间隔 $\Delta t$ 把状态往前推一小步
- 如果当前时刻还没收到新的传感器测量，系统仍然可以“先预测一个姿态出来”

### 但本工程有一个重要特征

`ekf.yaml` 里配置了：

```yaml
use_control: false
```

所以本工程的预测阶段并**不直接使用**控制指令 `cmd_vel` 作为控制输入项。

也就是说，这里的预测更接近于：

- 根据上一时刻内部状态做时间推进
- 而不是根据电机命令做显式运动学外推

因此本工程里第一公式的工程理解是：

> `ekf_filter_node` 依据上一时刻融合状态和内部运动模型，对机器人当前的 `x, y, yaw, vx, vy, vyaw` 做先验预测。

---

## 4.2 第二个公式：协方差预测方程

状态预测完，还要预测“不确定性”如何扩散：

$$
\mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^T + \mathbf{Q}_k
$$

其中：

- $\mathbf{P}_{k-1}$：上一时刻状态协方差
- $\mathbf{P}_k^-$：当前时刻预测协方差
- $\mathbf{F}_k$：状态转移模型的雅可比矩阵
- $\mathbf{Q}_k$：过程噪声协方差

### 在本工程中的含义

这一步表达的是：

- 即使机器人什么都没测到，随着时间推移，不确定性也会累积
- 预测走得越久，EKF 对自己“猜出来的状态”就越不自信
- 一旦有新测量进来，测量就更有机会把状态拉回去

### 在本工程里最直接的落点

就是 `ekf.yaml` 里的：

```yaml
process_noise_covariance: [...]
initial_estimate_covariance: [...]
```

它们分别对应：

- `process_noise_covariance`：上式中的 $\mathbf{Q}_k$
- `initial_estimate_covariance`：滤波器启动时的初始 $\mathbf{P}_0$

### 工程理解

如果你把 `process_noise_covariance` 某些对角线调大，等价于告诉 EKF：

- “你的模型没那么准”
- “别太迷信预测结果”
- “更快相信新的传感器观测”

如果把它调太小，则表示：

- “模型很靠谱”
- EKF 会更依赖预测，收敛更平滑，但可能更迟钝

在本工程里，这组参数直接决定：

- `/odom` 的平滑程度
- 转弯时姿态响应快不快
- 面对短时异常测量时是稳住还是跟着跳

---

## 4.3 第三个公式：创新（残差）方程

当新的测量 $\mathbf{z}_k$ 到来后，EKF 要先算“测量值”和“预测值”差了多少：

$$
\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_k^-)
$$

其中：

- $\mathbf{z}_k$：传感器实际测量
- $h(\hat{\mathbf{x}}_k^-)$：把预测状态映射到测量空间后的预测测量
- $\mathbf{y}_k$：创新，也叫残差

### 在本工程中的含义

这一步的本质是问：

- 当前 IMU 说机器人 yaw 是多少？
- 当前 RF2O 说机器人 x、y、yaw 在哪里？
- 当前轮速里程计说机器人 vx、vy、vyaw 多大？
- 这些测量和 EKF 刚刚预测出来的结果，相差多少？

这份“差值”就是 EKF 后续修正的依据。

### 放到本工程三个输入里理解

#### 对 `odom_raw`

EKF 会比较：

- 预测出来的 `vx, vy, vyaw`
- 与 `odom_raw` 提供的 `vx, vy, vyaw`

差多少。

#### 对 `odom_rf2o`

EKF 会比较：

- 预测出来的 `x, y, yaw`
- 与激光里程计给出的 `x, y, yaw`

差多少。

#### 对 `imu`

EKF 会比较：

- 预测出来的 `yaw, vyaw`
- 与 IMU 测得的 `yaw, vyaw`

差多少。

### 为什么这一步很关键

如果某一时刻某个传感器突然跳变，那么它和预测值之间的残差会突然变大。

这正是后续判断“该不该信它”“信多少”的前提。

---

## 4.4 第四个公式：卡尔曼增益

在得到残差之后，EKF 要计算本次修正应该相信测量多少：

$$
\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R}_k)^{-1}
$$

其中：

- $\mathbf{K}_k$：卡尔曼增益
- $\mathbf{H}_k$：测量模型的雅可比矩阵
- $\mathbf{R}_k$：测量噪声协方差

### 在本工程中的含义

卡尔曼增益决定：

- 这次更信预测，还是更信测量
- 是轻轻修一下，还是大幅拉过去

### 工程化解释

如果当前情况是：

- 预测协方差 $\mathbf{P}_k^-$ 很大，说明 EKF 对自己的预测没把握
- 某传感器测量噪声 $\mathbf{R}_k$ 很小，说明这路测量比较可靠

那么算出来的 $\mathbf{K}_k$ 就会偏大，意味着：

> 本次修正更偏向相信传感器。

反过来，如果：

- 预测本身很稳定
- 某一路测量噪声很大或跳变明显

那么 $\mathbf{K}_k$ 会偏小，EKF 就不会被那一路数据轻易带偏。

### 在本工程里是谁提供 $\mathbf{R}_k$

通常来自各个传感器消息本身携带的协方差，例如：

- `nav_msgs/Odometry` 里的 pose covariance / twist covariance
- `sensor_msgs/Imu` 里的 orientation covariance / angular_velocity covariance

所以这里有一个非常重要的工程事实：

> EKF 是否“聪明”，不仅取决于公式本身，还强烈取决于输入消息里的协方差是否合理。

如果某个上游节点把协方差乱填，那么即使 EKF 公式完全正确，融合结果也可能不理想。

---

## 4.5 第五个公式：状态更新与协方差更新

有了卡尔曼增益之后，EKF 用测量来修正预测状态：

$$
\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k \mathbf{y}_k
$$

同时更新协方差：

$$
\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^-
$$

其中：

- $\hat{\mathbf{x}}_k$：融合后的最终状态估计
- $\mathbf{P}_k$：更新后的不确定性

### 在本工程中的含义

这一步结束后，EKF 就得到了当前时刻最可信的机器人状态。

然后它会把这个结果用于：

- 发布 `/odom`
- 发布 `odom -> base_footprint` TF
- 作为下一次预测的起点

所以你在工程里真正看到的 `/odom`，本质上就是第五步更新后的输出结果。

可以直接把它理解为：

> `/odom = EKF 把轮速里程计、IMU、激光里程计综合折中之后给出的当前最优局部位姿`

---

## 5. 五大公式在本工程中的完整闭环

把上面五步连起来，本工程 EKF 每个周期都在做下面这件事：

### 第 1 步：先预测

利用上一时刻的状态，预测当前机器人应该在什么位置、朝什么方向、速度多大。

### 第 2 步：同时预测不确定性

随着时间推进，不确定性会增加，尤其是在没有新传感器输入的时候。

### 第 3 步：等测量来了，计算残差

拿 `odom_raw`、`odom_rf2o`、`imu` 的观测值，与预测值做比较。

### 第 4 步：计算该信谁更多

根据预测不确定性和测量协方差，算出卡尔曼增益。

### 第 5 步：完成融合更新

得到新的最优状态，并发布 `/odom` 与 TF。

这五步不断循环，于是形成一个持续在线工作的局部定位器。

---

## 6. 为什么本工程需要 EKF，而不是直接用 `/odom_raw`

如果不用 EKF，系统可能只能直接依赖某一路原始估计：

- 只用轮速里程计：容易累计漂移，尤其转向时更明显
- 只用 IMU：角度短时好，但无法独立给出稳定平面位置
- 只用激光里程计：局部位姿约束强，但可能受环境纹理、遮挡、动态物体影响

而 EKF 的价值就在于把三者优势拼起来：

- 轮速里程计提供连续速度
- IMU 提供短时姿态与角速度约束
- 激光里程计提供平面位置/航向修正

最终得到一个兼顾：

- 连续性
- 平滑性
- 抗漂移能力
- 实时性

的 `/odom`。

这正是 SLAM 和 Navigation 最需要的底层能力。

---

## 7. EKF 输出在本工程后续模块中的作用

### 7.1 在 SLAM 模式里

`slam_toolbox` 通常需要：

- `/scan_raw`
- `/odom`
- TF

其中 `/odom` 就是前端短时连续运动估计。

它的作用不是“代替建图”，而是：

- 告诉 SLAM：机器人刚刚大概走了多少
- 帮助 scan matching 更稳定收敛
- 提高建图过程中的位姿连续性

如果 `/odom` 很抖、延迟大、漂移严重，SLAM 的建图质量也会明显下降。

---

### 7.2 在 Navigation 模式里

导航阶段一般已经有地图和全局定位模块（如 AMCL）。

这时：

- AMCL 更偏全局定位，负责 `map -> odom`
- EKF 更偏局部连续定位，负责 `odom -> base_footprint`

两者组合后形成完整定位链：

```text
map -> odom -> base_footprint -> base_link -> lidar_frame
```

这也是 ROS 地面机器人导航中的经典结构。

---

## 8. 本工程 EKF 的几个关键工程特征

### 8.1 `two_d_mode: true`

说明它是一个典型平面移动机器人滤波器。

因此：

- 重点融合 `x, y, yaw`
- 以及对应速度量
- 不强调 `z, roll, pitch`

---

### 8.2 `world_frame: odom`

说明这是一个“局部连续坐标系”滤波器，而不是全局地图定位器。

这非常重要，因为它决定了 EKF 的职责是：

- 输出平滑、连续、短时可靠的局部位姿
- 允许长期漂移
- 把全局纠偏交给 SLAM 或 AMCL

---

### 8.3 `publish_tf: true`

说明 EKF 不仅发里程计消息，还直接承担了一部分 TF 发布职责。

因此它在工程里不是“纯数学模块”，而是：

- 位姿估计器
- TF 发布者
- 上层导航/建图的基础坐标桥梁

---

### 8.4 `sensor_timeout: 2.0`

表示如果某路传感器一段时间没数据，EKF 会退化成只做预测。

这很符合实际工程：

- 某路输入临时掉线时，系统不会立刻崩
- 但随着时间变长，预测不确定性会增大
- 输出 `/odom` 会逐渐变得不那么可信

这正是第二公式里协方差扩散在工程中的直接体现。

---

## 9. 从“数学公式”到“工程现象”的对应表

| EKF 数学步骤 | 数学表达 | 本工程对应环节 | 工程现象 |
|---|---|---|---|
| 1. 状态预测 | $\hat{x}_k^- = f(\hat{x}_{k-1}, u_k)$ | `ekf_filter_node` 依据上一时刻状态向前推演 | 即使没有新测量，机器人状态也能连续输出 |
| 2. 协方差预测 | $P_k^- = F_k P_{k-1} F_k^T + Q_k$ | `process_noise_covariance`、内部模型线性化 | 时间越久没校正，EKF 越不自信 |
| 3. 创新计算 | $y_k = z_k - h(\hat{x}_k^-)$ | 比较 `odom_raw` / `odom_rf2o` / `imu` 与预测值 | 传感器跳变时残差突然增大 |
| 4. 卡尔曼增益 | $K_k = P_k^- H_k^T (H_k P_k^- H_k^T + R_k)^{-1}$ | 根据预测不确定性和测量协方差决定信任比例 | 有时更信 IMU，有时更信里程计 |
| 5. 状态更新 | $\hat{x}_k = \hat{x}_k^- + K_k y_k$ | 输出 `/odom` 和 `odom -> base_footprint` | 得到平滑且连续的融合位姿 |

---

## 10. 一句话理解本工程中的 EKF

如果要用一句最贴近工程的话概括：

> 本工程中的 EKF，本质上就是一个“实时裁判员”：它先根据上一时刻状态做预测，再让 `odom_raw`、`imu`、`odom_rf2o` 分别发言，最后按可信度加权融合，输出给整个机器人系统使用的局部标准位姿 `/odom`。

---

## 11. 你读这个工程时，应该重点盯住哪几个文件

如果你想继续顺着代码深挖，建议重点看这几个文件：

- `src/driver/controller/launch/controller.launch.py`
  - 看 EKF 节点是怎么被启动的
- `src/driver/controller/config/ekf.yaml`
  - 看输入源、状态掩码、噪声矩阵、坐标系定义
- `src/peripherals/launch/imu_filter.launch.py`
  - 看 IMU 是怎样被预处理后送入 EKF 的
- `src/slam/launch/slam.launch.py`
  - 看上层 SLAM 怎样使用 `/odom`
- `src/navigation/launch/navigation.launch.py`
  - 看导航模式怎样建立 `map -> odom -> base_footprint` 链路

---

## 12. 最后总结

本工程中的 EKF 不是一个孤立算法，而是整个机器人局部定位链的核心枢纽。

它把五大公式真正落成了工程闭环：

1. 先预测机器人下一刻状态
2. 再预测这份预测有多不确定
3. 用传感器测量和预测做差
4. 算出这次该信传感器多少
5. 更新成新的最优状态并发布 `/odom` 与 TF

因此，EKF 在本工程里的实际价值不是“做了一组数学运算”，而是：

- 把多个不完美传感器融合成一个更可靠的底层位姿来源
- 给 SLAM 提供连续运动先验
- 给 Navigation 提供稳定的局部坐标基准
- 把机器人从“原始传感器堆砌”提升到“可用的定位系统”

这就是 EKF 五大公式在本工程中的真正落地方式。
