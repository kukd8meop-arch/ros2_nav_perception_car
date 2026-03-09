IMU（惯性测量单元）包含加速度计和陀螺仪，但它们的测量值会随时间产生缓慢漂移，这个漂移的核心就是偏置（bias）：
ba​：加速度计偏置（accelerometer bias）

    加速度计理论上静止时应输出重力加速度，但实际会有固定偏差，ba​ 就是这个偏差的估计值（单位：m/s²）

bg​：陀螺仪偏置（gyroscope bias）

    陀螺仪理论上静止时应输出 0，但实际会有缓慢漂移，bg​ 是这个漂移的估计值（单位：rad/s）

本工程下面是这样处理的：

1. 原始 IMU 数据先由 `ros_robot_controller_node.py` 从底层控制板读取，并发布到 `/ros_robot_controller/imu_raw`。

    这一层只是把底层读到的 `ax, ay, az, gx, gy, gz` 封装成 ROS2 的 `sensor_msgs/Imu` 消息，
    本身**没有做偏置扣除**。

2. 接着会启动 `src/peripherals/launch/imu_filter.launch.py`，这里面串了两层处理：

    - 第一层：`imu_calib/apply_calib`
    - 第二层：`imu_complementary_filter/complementary_filter_node`

3. 第一层 `imu_calib/apply_calib` 会读取标定文件：

    - `src/calibration/config/imu_calib.yaml`

    然后把：

    - 输入 `raw -> /ros_robot_controller/imu_raw`
    - 输出 `corrected -> imu_corrected`

    这里的 `imu_calib.yaml` 中已经写入了一组标定参数，主要包括：

    - `SM`：标定矩阵（可理解为比例因子 + 轴间耦合修正）
    - `bias`：静态偏置补偿值

    也就是说，这一层已经在做**静态校准**。

4. 第二层 `imu_complementary_filter` 继续处理 `imu_corrected`，并输出最终的 `/imu`。

    它的参数中明确写了：

    - `do_bias_estimation: True`

    这说明除了前面的静态标定外，它还会在运行过程中继续做一定程度的**在线偏置估计**。

所以本工程的 IMU 链路可以概括成：

```text
底层原始IMU
  -> /ros_robot_controller/imu_raw
  -> imu_calib/apply_calib（读取 imu_calib.yaml 做静态校准）
  -> imu_corrected
  -> imu_complementary_filter（继续做姿态解算 + bias estimation）
  -> /imu
```

结论：

- 这个工程**不是没有处理 IMU 偏置**
- 而是已经接入了“静态标定 + 在线偏置估计”两层机制

但是要注意：

**“工程里有偏置参数” ≠ “这些参数一定适合你当前这台实机”。**

因为这些偏置值可能是：

- 厂家预设值
- 某台样机的历史标定结果
- 旧设备上测出来后直接沿用的结果

如果你换了 IMU 模块、换了整车、改变了安装姿态，甚至只是换了同型号但不同个体的传感器，偏置都可能发生变化。

所以更准确的说法应该是：

- **机制上已经配好了**
- **参数上也确实写进去了**
- **但是否真正适合你的车，还需要靠实测验证**

可以这样验证：

1. 让机器人静止不动
2. 观察 `/ros_robot_controller/imu_raw`、`imu_corrected`、`/imu`
3. 看看校正后的角速度是否更接近 0
4. 看看姿态是否明显更稳定
5. 如果静止时 yaw 还在明显漂、角速度长期不接近 0，就说明偏置参数可能还不够准

一句话总结：

> 本工程里的 IMU 偏置处理链路已经搭好了，但“是否配准到你这台实机”不能只看文件存在与否，最终还得靠静态测试来确认。
