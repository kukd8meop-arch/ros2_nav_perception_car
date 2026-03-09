激光雷达（LiDAR）本质上是在做一件事：
它不断向周围发射激光束，测量激光从发出到返回的时间或相位差，从而得到周围障碍物的距离信息。

对于 ROS2 机器人系统来说，雷达最核心的价值就是：

- 感知周围环境轮廓
- 提供建图和定位所需的几何信息
- 为避障与导航提供基础输入

本工程下面是这样处理的：

1. 雷达本身通过串口接到 Jetson，Linux 会把这个串口设备映射成一个设备文件。

    但系统自动分配的名字可能每次开机都不同，例如：

    - `/dev/ttyUSB0`
    - `/dev/ttyUSB1`

    为了避免每次启动都改配置，本工程用 `udev` 规则给雷达固定了一个别名：

    - `/dev/lidar`

2. 这个别名来自规则文件：

    - `src/peripherals/scripts/lidar.rules`

    它的作用是：

    - 当雷达插在指定的 USB 物理口上时
    - 自动创建一个软链接别名 `/dev/lidar`

    这样 launch 文件和驱动节点就不用关心底层到底是 `/dev/ttyUSB0` 还是 `/dev/ttyUSB1`。

3. 雷达启动入口是：

    - `src/peripherals/launch/lidar.launch.py`

    这个 launch 文件会先读取环境变量：

    - `LIDAR_TYPE`

    然后根据不同型号路由到对应的驱动 launch。

    例如当：

    - `LIDAR_TYPE=LD19`

    它会进一步调用：

    - `src/peripherals/launch/include/ldlidar_LD19.launch.py`

4. 在 `ldlidar_LD19.launch.py` 中，真正启动的驱动节点是：

    - `package='ldlidar_stl_ros2'`
    - `executable='ldlidar_stl_ros2_node'`

    这个节点会使用下面这些关键参数：

    - `port_name='/dev/lidar'`
    - `port_baudrate=230400`
    - `frame_id=lidar_frame`

    这意味着：

    - 驱动程序去打开 `/dev/lidar`
    - 按 LD19 的协议读取雷达原始数据
    - 再封装成 ROS2 标准的激光扫描消息

5. 驱动节点最终发布的话题是：

    - `/scan_raw`

    在 `ldlidar_LD19.launch.py` 里，厂家驱动内部原本发布的话题名是 `scan`，
    但通过 remap：

    - `('scan', scan_raw)`

    最终被映射成了 `/scan_raw`。

所以本工程的 LiDAR 链路可以概括成：

```text
物理雷达
  -> Linux 串口设备（如 /dev/ttyUSB0）
  -> udev 规则映射成 /dev/lidar
  -> ldlidar_stl_ros2_node
  -> /scan_raw
```

这里的 `/scan_raw` 是什么意思？

- 它是雷达当前这一圈扫描到的原始距离数据
- 消息类型通常是 `sensor_msgs/LaserScan`
- 它表达的是：
  - 在雷达自身坐标系下
  - 每个角度方向上障碍物距离雷达有多远

也就是说，`/scan_raw` 只会告诉系统：

- 前方 1.2 米有障碍物
- 左前方 0.8 米有障碍物
- 右边 2.5 米是空的

但它并不知道：

- 机器人当前在地图的什么位置
- 这一圈数据该放到地图上的哪里

所以它必须结合其他信息一起用。

在本工程里，雷达数据后续会和下面这些信息配合：

- `/odom`：机器人此刻大概走到了哪里
- `/tf`：雷达相对机器人本体的安装位置，以及机器人坐标系之间的变换关系

然后一起送给：

- `slam_toolbox`

由 `slam_toolbox` 去完成：

- scan matching
- 地图更新
- 机器人位姿修正
- 发布 `/map`
- 发布 `map -> odom` TF

所以在建图模式下，LiDAR 的作用不是“直接生成地图”，而是：

> **不断提供当前这一帧环境几何信息，再结合里程计和 TF，让 `slam_toolbox` 把每一帧扫描正确拼接到地图里。**

结论：

- 本工程**已经把 LiDAR 驱动链路接好了**
- 你不需要自己从零写雷达协议解析
- 你真正要关心的是：
  - `/dev/lidar` 是否正常存在
  - `LIDAR_TYPE` 是否设置正确
  - `/scan_raw` 是否正常发布
  - `lidar_frame` 和 TF 是否正确

但是要注意：

**“驱动能启动” ≠ “雷达数据一定就能被系统正确使用”。**

因为还可能出现这些问题：

- `/dev/lidar` 没有成功映射
- 插错了 USB 口，导致 udev 规则没命中
- `LIDAR_TYPE` 设置错了，调起了错误的驱动
- `frame_id` 不对，TF 树接不上
- `/scan_raw` 有数据，但建图时 `/tf` 或 `/odom` 对不上

可以这样验证：

1. 确认设备别名是否存在
2. 启动雷达驱动
3. 观察 `/scan_raw` 是否持续有数据
4. 看 `frame_id` 是否和 TF 树一致
5. 再让 `slam_toolbox` 订阅它，检查能否正常建图

一句话总结：

> 本工程里的 LiDAR 接入链路已经搭好了：通过 `/dev/lidar` 找到物理设备，驱动节点把原始扫描数据发布到 `/scan_raw`，然后再交给上层建图与导航模块继续使用。
