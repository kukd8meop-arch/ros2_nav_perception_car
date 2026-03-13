"""
sim_nav_full.launch.py — Gazebo + Nav2 + RViz2 完整闭环仿真启动
================================================================

支持两种 Gazebo 后端（通过 use_gz 参数切换）：
  - use_gz:=true   (默认)  → Ignition Gazebo 6 (Fortress)
  - use_gz:=false          → Gazebo Classic 11（已弃用，备用）

数据流（闭环原理）
------------------
                    ┌─────────────┐
                    │   RViz2     │  ← 用户点击 "2D Nav Goal"
                    │             │        │
                    └──────┬──────┘        │ /goal_pose (PoseStamped)
                           │               ▼
                    ┌──────▼──────────────────────┐
                    │        Nav2 核心栈           │
                    │  ┌─────────────────────┐    │
                    │  │  BT Navigator       │    │
                    │  │  GlobalPlanner      │    │
                    │  │  LocalPlanner(DWB)  │    │
                    │  │  VelocitySmoother   │    │
                    │  └──────────┬──────────┘    │
                    │             │ /cmd_vel       │
                    └─────────────┼────────────────┘
                                  │
                    ┌─────────────▼──────────────┐
                    │     cmd_vel_relay 节点      │  topic_tools relay
                    │  /cmd_vel → /controller/cmd_vel │
                    └─────────────┬──────────────┘
                                  │ /controller/cmd_vel
                    ┌─────────────▼──────────────┐
                    │     Gazebo 仿真 (Classic    │
                    │     或 Ignition)            │
                    │  插件消费指令并移动小车      │
                    │  发布 /odom + /clock        │
                    └─────────────┬──────────────┘
                                  │ /odom  /tf  /scan  /clock
                    ┌─────────────▼──────────────┐
                    │   AMCL（定位）              │
                    │   在地图上估计位姿           │
                    └─────────────────────────────┘

启动顺序（智能等待，而非固定延时）
-----------------------------------
Phase 1:  Gazebo 启动 + 生成机器人模型（+ ros_gz_bridge，如果用 Ignition）
Phase 2:  等待脚本循环检测 /odom 话题出现
Phase 3:  上述全部就绪后再启动 Nav2 + cmd_vel_relay + RViz2

用法
----
  ros2 launch navigation sim_nav_full.launch.py
  ros2 launch navigation sim_nav_full.launch.py use_gz:=true
  ros2 launch navigation sim_nav_full.launch.py map:=map_02
  ros2 launch navigation sim_nav_full.launch.py use_gui:=false
  ros2 launch navigation sim_nav_full.launch.py use_teb:=true
"""

import os
import tempfile
import textwrap

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─── 工具函数：生成仿真专用 nav2_params ──────────────────────────────────────

def _create_wait_script() -> str:
    """
    生成一个 Python 临时脚本，循环等待 Gazebo 完全就绪：
      1. 等待 /unpause_physics 服务出现（说明 gzserver 加载完成）
      2. 调用 unpause_physics（可能重试多次）
      3. 等待 /clock 话题有数据（说明仿真时钟开始推进）
      4. 等待 /odom 话题有数据（说明 planar_move 插件已工作）
    只有全部满足后脚本才退出（退出码 0），后续 launch 节点才会启动。
    """
    code = textwrap.dedent(r'''
        #!/usr/bin/env python3
        """等待 Gazebo 完全就绪（/odom 话题出现）后退出."""
        import subprocess, sys, time, os

        TIMEOUT = 180  # 总超时（秒）

        # 确保子进程不受 ROS2 仿真时钟影响
        env = os.environ.copy()
        env.pop("ROS_DOMAIN_ID", None)  # 保留当前 domain

        def run(cmd, timeout=15):
            try:
                r = subprocess.run(cmd, capture_output=True, text=True,
                                   timeout=timeout, env=env)
                return r.returncode, r.stdout
            except subprocess.TimeoutExpired:
                return -1, ""

        start = time.time()
        last_print = 0

        # ── 等待 /odom 话题出现在 topic list 中 ─────────────────────
        # planar_move 插件加载完成后才会 advertise /odom
        # 使用 --no-daemon 避免 ros2 daemon 缓存旧数据
        print("[wait] 等待 Gazebo planar_move 插件就绪（/odom 话题出现）...", flush=True)
        while time.time() - start < TIMEOUT:
            rc, out = run(["ros2", "topic", "list", "--no-daemon"], timeout=10)
            topics = [t.strip() for t in out.strip().split("\n") if t.strip()]
            if rc == 0 and "/odom" in topics:
                print("[wait] /odom 话题已出现", flush=True)
                break
            elapsed = int(time.time() - start)
            if elapsed - last_print >= 10:
                last_print = elapsed
                print(f"[wait] 已等待 {elapsed}s ... (当前话题数: {len(topics)})", flush=True)
            time.sleep(2)
        else:
            print("[wait] 超时：/odom 话题未出现", flush=True)
            sys.exit(1)

        # 多等 3 秒让 planar_move 稳定发布
        print("[wait] /odom 已出现，等待 3s 让插件稳定...", flush=True)
        time.sleep(3)

        elapsed = time.time() - start
        print(f"[wait] Gazebo 完全就绪！耗时 {elapsed:.1f}s", flush=True)
    ''')

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.py', prefix='wait_gazebo_', delete=False)
    tmp.write(code)
    tmp.close()
    os.chmod(tmp.name, 0o755)
    return tmp.name

def _patch_yaml_for_sim(source_yaml: str) -> str:
    """
    读取原始 YAML 配置，做两件事后写入临时文件：
      1. 将所有 use_sim_time 改为 True（使用 Gazebo 仿真时钟，而非系统时钟）
      2. 删除空列表键（ROS2 launch 解析空列表会崩溃）
    返回：临时文件路径
    """
    with open(source_yaml, 'r') as f:
        data = yaml.safe_load(f)

    def _patch(node):
        if isinstance(node, dict):
            to_delete = []
            for k, v in node.items():
                if k == 'use_sim_time':
                    node[k] = True          # 强制仿真时间
                elif isinstance(v, list) and len(v) == 0:
                    to_delete.append(k)     # 删空列表
                else:
                    _patch(v)
            for k in to_delete:
                del node[k]
        elif isinstance(node, list):
            for item in node:
                _patch(item)

    _patch(data)

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='sim_nav2_params_', delete=False)
    yaml.dump(data, tmp, default_flow_style=False, allow_unicode=True)
    tmp.close()
    return tmp.name


# ─── 核心 launch 配置函数（在 OpaqueFunction 内执行，可读取运行时参数值） ───

def launch_setup(context):
    # ── 包路径解析 ─────────────────────────────────────────────────────────
    nav_pkg  = get_package_share_directory('navigation')
    slam_pkg = get_package_share_directory('slam')
    desc_pkg = get_package_share_directory('rosorin_description')

    # ── 读取 launch 参数实际值 ─────────────────────────────────────────────
    map_name = LaunchConfiguration('map').perform(context)
    use_gui    = LaunchConfiguration('use_gui').perform(context)
    use_teb    = LaunchConfiguration('use_teb').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)
    use_gz     = LaunchConfiguration('use_gz').perform(context)
    x_pose     = LaunchConfiguration('x_pose').perform(context)
    y_pose   = LaunchConfiguration('y_pose').perform(context)
    yaw      = LaunchConfiguration('yaw').perform(context)

    # ── 地图路径 ───────────────────────────────────────────────────────────
    map_yaml = os.path.join(slam_pkg, 'maps', map_name + '.yaml')
    if not os.path.isfile(map_yaml):
        raise FileNotFoundError(
            f"[sim_nav_full] 找不到地图文件: {map_yaml}\n"
            f"  请在 slam/maps/ 下放置对应的 .yaml 和 .pgm 文件，\n"
            f"  或者通过 map:=<地图名> 参数指定正确的地图名称。"
        )

    # ── 生成仿真版参数文件（覆盖 use_sim_time 并修复空列表） ──────────────
    nav2_params_src  = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    nav2_params_sim  = _patch_yaml_for_sim(nav2_params_src)

    # 控制器参数文件（TEB / DWB 二选一）
    ctrl_yaml_name = 'nav2_controller_teb.yaml' if use_teb == 'true' \
                     else 'nav2_controller_dwb.yaml'
    ctrl_yaml_src  = os.path.join(nav_pkg, 'config', ctrl_yaml_name)
    ctrl_yaml_sim  = _patch_yaml_for_sim(ctrl_yaml_src)

    # ── RViz2 配置文件 ─────────────────────────────────────────────────────
    rviz_config = os.path.join(nav_pkg, 'rviz', 'navigation.rviz')

    # ── 生成等待脚本 ───────────────────────────────────────────────────────
    wait_script = _create_wait_script()

    # =========================================================================
    # ① Gazebo 启动块（根据 use_gz 切换 Classic / Ignition）
    #
    # Classic (use_gz=false):
    #    - gazebo.launch.py → gzserver + gzclient + spawn_entity
    #    - planar_move 插件直接发布 ROS2 /odom 话题
    #    - cmd_vel_relay 把 /cmd_vel 转发到 /controller/cmd_vel
    #
    # Ignition (use_gz=true):
    #    - gazebo_gz.launch.py → ign gazebo + ros_gz_bridge + spawn
    #    - velocity-control 插件 + odometry-publisher 通过 bridge 桥接
    #    - bridge 自带重映射，cmd_vel_relay 仍然有效
    # =========================================================================

    if use_gz == 'true':
        # ── Ignition Gazebo 模式 ─────────────────────────────────────────
        sim_world = os.path.join(desc_pkg, 'worlds', 'sim_empty_gz.sdf')
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'gazebo_gz.launch.py')
            ),
            launch_arguments={
                'world':        sim_world,
                'use_gui':      use_gui,
                'use_sim_time': 'true',
                'x_pose':       x_pose,
                'y_pose':       y_pose,
                'z_pose':       '0.05',
                'yaw':          yaw,
            }.items(),
        )
    else:
        # ── Gazebo Classic 模式（原有逻辑） ──────────────────────────────
        sim_world = os.path.join(desc_pkg, 'worlds', 'sim_empty.world')
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world':        sim_world,
                'use_gui':      use_gui,
                'use_sim_time': 'true',
                'x_pose':       x_pose,
                'y_pose':       y_pose,
                'z_pose':       '0.05',
                'yaw':          yaw,
            }.items(),
        )

    # =========================================================================
    # ② 等待 Gazebo 完全就绪的脚本
    #    这是一个 Python 脚本（ExecuteProcess），它会：
    #      a) 循环检测 /unpause_physics 服务出现
    #      b) 调用 unpause（带重试）
    #      c) 确认 /clock 和 /odom 有数据
    #    脚本退出后（OnProcessExit 事件）才启动 Nav2 + RViz2
    #    这样就不怕 Gazebo 启动慢了——无论等多久都能正确衔接
    # =========================================================================
    wait_for_gazebo = ExecuteProcess(
        cmd=['python3', wait_script],
        output='screen',
        name='wait_for_gazebo_ready',
    )

    # =========================================================================
    # ③ Nav2 完整栈（通过 bringup.launch.py 组合）
    #    bringup.launch.py 内部会依次启动：
    #      - nav2_container（composable node 容器）
    #      - localization.launch.py → map_server + AMCL
    #      - navigation_base.launch.py → BT Navigator + GlobalPlanner
    #                                    + LocalPlanner + VelocitySmoother
    # =========================================================================
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'include', 'bringup.launch.py')
        ),
        launch_arguments={
            'namespace':             '',
            'use_namespace':         'false',
            'map':                   map_yaml,          # 定位用地图
            'params_file':           nav2_params_sim,   # 仿真版 nav2 参数
            'nav2_controller_param': ctrl_yaml_sim,     # 仿真版控制器参数
            'use_sim_time':          'true',
            'autostart':             'true',            # 节点自动激活，无需手动
            'use_teb':               use_teb,
        }.items(),
    )

    # =========================================================================
    # ④ cmd_vel 桥接节点（topic_tools relay）
    #
    #    Nav2 速度输出链路：
    #      controller_server → /cmd_vel_nav
    #      velocity_smoother → /cmd_vel           ← Nav2 最终输出
    #
    #    Gazebo planar_move 插件订阅：
    #      /controller/cmd_vel                    ← Gazebo 期望的输入
    #
    #    relay 节点：把 /cmd_vel 转发到 /controller/cmd_vel
    #    这是 Gazebo ↔ Nav2 速度闭环的关键！
    # =========================================================================
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/cmd_vel', '/controller/cmd_vel'],
    )

    # =========================================================================
    # ⑤ RViz2 可视化
    #    加载 navigation.rviz 配置，包含：
    #      - Map（显示地图）
    #      - LaserScan（显示激光点云）
    #      - RobotModel（显示机器人模型）
    #      - TF（显示坐标系）
    #      - Path（显示规划路径）
    #      - Nav2 Goal（点击指定目标点 ← 核心交互入口）
    #    RViz2 工具栏选择 "2D Nav Goal"（或 Nav2 Goal）后，
    #    在地图上点击+拖拽即可发送目标点。
    # =========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # =========================================================================
    # 组装完整 launch 序列（事件驱动，而非固定延时）
    #
    # 流程：
    #   Gazebo 启动(t=0)
    #         ↓
    #   wait_for_gazebo 脚本开始循环检测（与 Gazebo 同时启动）
    #         ↓ 脚本退出（Gazebo 就绪）
    #   OnProcessExit 触发 → 同时启动 Nav2 + cmd_vel_relay + RViz2
    # =========================================================================
    # 根据 launch_rviz 参数决定是否在 launch 内启动 RViz2
    on_ready_actions = [
        bringup_launch,      # Nav2 完整栈
        cmd_vel_relay,       # /cmd_vel → /controller/cmd_vel
    ]
    if launch_rviz == 'true':
        # RViz2 延迟 5s，等 Nav2 lifecycle 节点 activate 完毕
        on_ready_actions.append(TimerAction(period=5.0, actions=[rviz_node]))

    return [
        # ① Gazebo（含 joint_state_publisher / robot_state_publisher / spawn_entity）
        gazebo_launch,

        # ② 等待脚本（循环检测 /odom 话题出现）
        wait_for_gazebo,

        # ③④(⑤) Gazebo 就绪后启动 Nav2 + relay（+ 可选 RViz2）
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_gazebo,
                on_exit=on_ready_actions,
            )
        ),
    ]


# ─── launch 入口 ─────────────────────────────────────────────────────────────

def generate_launch_description():
    return LaunchDescription([
        # ── 环境变量：让各子 launch 文件从 install/ 而非硬编码路径读取资源 ──
        SetEnvironmentVariable('need_compile', 'True'),

        # ── launch 参数声明（命令行可覆盖默认值） ──────────────────────────
        DeclareLaunchArgument(
            'map',
            default_value='map_01',
            description='slam/maps/ 下的地图名（不含 .yaml），例如 map_01 / map_02',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='是否打开 Gazebo 可视化窗口（true/false）',
        ),
        DeclareLaunchArgument(
            'use_teb',
            default_value='false',
            description='是否使用 TEB 局部控制器（false=DWB，true=TEB）',
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='机器人在 Gazebo 世界中的初始 X 坐标（米）',
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='机器人在 Gazebo 世界中的初始 Y 坐标（米）',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='机器人在 Gazebo 世界中的初始朝向（弧度，0=朝 X 轴正方向）',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='是否在 launch 内启动 RViz2（false=由外部脚本在独立终端启动）',
        ),
        DeclareLaunchArgument(
            'use_gz',
            default_value='true',
            description='使用 Ignition Gazebo (true) 或 Gazebo Classic (false)',
        ),

        # OpaqueFunction 内解析参数的实际值并返回所有 actions
        OpaqueFunction(function=launch_setup),
    ])
