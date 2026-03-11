"""
仿真专用导航入口 (阶段 D)
=========================
一条命令拉起完整的 Gazebo 闭环导航链路：
  Gazebo  →  nav2_container + map_server + AMCL + Nav2  →  RViz2

用法:
  ros2 launch navigation navigation_sim.launch.py
  ros2 launch navigation navigation_sim.launch.py map:=map_02 use_gui:=false

设计原则:
  1. 完全不依赖实车硬件，不 include robot.launch.py
  2. 不修改实车版 nav2_params.yaml，仿真侧通过 RewrittenYaml 动态覆盖 use_sim_time
  3. cmd_vel 差异在本文件内通过 topic_tools relay 解决
  4. 实车入口 navigation.launch.py 保持不变
  5. 复用 bringup.launch.py 来启动 nav2_container + localization + navigation_base
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_sim_params(source_yaml: str) -> str:
    """读取 nav2_params.yaml，覆盖 use_sim_time 并修复空列表，写入临时文件."""
    with open(source_yaml, 'r') as f:
        data = yaml.safe_load(f)

    def _patch(node):
        """递归遍历: 1) use_sim_time→True  2) 删除空列表键（避免 ROS2 crash）"""
        if isinstance(node, dict):
            keys_to_delete = []
            for k, v in node.items():
                if k == 'use_sim_time':
                    node[k] = True
                elif isinstance(v, list) and len(v) == 0:
                    keys_to_delete.append(k)
                else:
                    _patch(v)
            for k in keys_to_delete:
                del node[k]
        elif isinstance(node, list):
            for item in node:
                _patch(item)

    _patch(data)

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='nav2_sim_params_', delete=False)
    yaml.dump(data, tmp, default_flow_style=False, allow_unicode=True)
    tmp.close()
    return tmp.name


def _create_sim_controller_params(source_yaml: str) -> str:
    """读取 nav2_controller_dwb/teb.yaml，覆盖 use_sim_time 为 True，写入临时文件."""
    with open(source_yaml, 'r') as f:
        data = yaml.safe_load(f)

    def _patch(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k == 'use_sim_time':
                    node[k] = True
                else:
                    _patch(v)
        elif isinstance(node, list):
            for item in node:
                _patch(item)

    _patch(data)

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='nav2_sim_controller_', delete=False)
    yaml.dump(data, tmp, default_flow_style=False, allow_unicode=True)
    tmp.close()
    return tmp.name


def launch_setup(context):
    """在 OpaqueFunction 内解析所有 LaunchConfiguration 以便做字符串拼接."""
    # ── 包路径 ──────────────────────────────────────────────
    nav_pkg = get_package_share_directory('navigation')
    slam_pkg = get_package_share_directory('slam')
    desc_pkg = get_package_share_directory('rosorin_description')

    # ── 解析参数 ─────────────────────────────────────────
    map_name = LaunchConfiguration('map').perform(context)
    use_gui  = LaunchConfiguration('use_gui').perform(context)
    x_pose   = LaunchConfiguration('x_pose').perform(context)
    y_pose   = LaunchConfiguration('y_pose').perform(context)
    yaw      = LaunchConfiguration('yaw').perform(context)

    # ── 拼接地图全路径 ───────────────────────────────────
    map_yaml = os.path.join(slam_pkg, 'maps', map_name + '.yaml')

    # ── 生成仿真版参数文件 ───────────────────────────────
    # 1) 读取原始 YAML  2) 覆盖 use_sim_time  3) 修复空列表（ROS2 launch bug）
    nav2_params_file = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    configured_params_file = _create_sim_params(nav2_params_file)

    # 也处理 controller 参数文件（它是独立加载的，不走 RewrittenYaml）
    use_teb  = LaunchConfiguration('use_teb').perform(context)
    if use_teb == 'true':
        ctrl_yaml = os.path.join(nav_pkg, 'config', 'nav2_controller_teb.yaml')
    else:
        ctrl_yaml = os.path.join(nav_pkg, 'config', 'nav2_controller_dwb.yaml')
    configured_ctrl_file = _create_sim_controller_params(ctrl_yaml)

    # ── ① 启动 Gazebo + 生成机器人 ────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_gui': use_gui,
            'use_sim_time': 'true',
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    # ── ② 通过 bringup.launch.py 启动完整 Nav2 栈 ────────
    # bringup 会创建 nav2_container 并 include localization + navigation_base
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'include', 'bringup.launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'map': map_yaml,
            'params_file': configured_params_file,
            'nav2_controller_param': configured_ctrl_file,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_teb': use_teb,
        }.items(),
    )

    # ── ③ RViz2 ──────────────────────────────────────────
    rviz_config = os.path.join(nav_pkg, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # ── ④ cmd_vel 桥接 ───────────────────────────────────
    # Nav2 输出链路：controller_server → cmd_vel_nav → velocity_smoother → cmd_vel
    # Gazebo 订阅：/controller/cmd_vel
    # 所以需要把 Nav2 最终输出的 /cmd_vel 转发到 /controller/cmd_vel
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/cmd_vel', '/controller/cmd_vel'],
    )

    # ── ⑤ Unpause Gazebo 物理引擎 ─────────────────────────
    # Gazebo spawn_entity 完成后物理引擎处于 paused 状态。
    # 必须调用 /unpause_physics 才能让仿真时间推进、/clock 发布数据、
    # planar_move 开始订阅 cmd_vel 并发布 /odom 和 TF。
    unpause_physics = ExecuteProcess(
        cmd=['ros2', 'service', 'call',
             '/unpause_physics', 'std_srvs/srv/Empty', '{}'],
        output='screen',
    )

    # ── 组装 ─────────────────────────────────────────────
    # 先让 Gazebo 稳定（5s），unpause 物理引擎，再启 Nav2
    return [
        # ① Gazebo
        gazebo_launch,

        # ② 等 Gazebo spawn 完成后 unpause 物理引擎（延时 3s）
        TimerAction(period=3.0, actions=[unpause_physics]),

        # ③ Nav2 完整栈（延时等 Gazebo + odom 稳定）
        TimerAction(period=5.0, actions=[bringup_launch]),

        # ④ cmd_vel 桥接（与 Nav2 同时）
        TimerAction(period=5.0, actions=[cmd_vel_relay]),

        # ⑤ RViz2（再延时一点等 Nav2 起来）
        TimerAction(period=8.0, actions=[rviz_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        # ── 环境变量：让子 launch 用 install 路径而非硬编码 ──
        SetEnvironmentVariable('need_compile', 'True'),

        # ── Launch 参数声明 ────────────────────────────────
        DeclareLaunchArgument(
            'map', default_value='map_01',
            description='slam/maps 下的地图名（不含 .yaml）'),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='是否打开 Gazebo GUI'),
        DeclareLaunchArgument(
            'use_teb', default_value='false',
            description='是否使用 TEB 局部控制器'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # OpaqueFunction 内解析参数并返回所有 actions
        OpaqueFunction(function=launch_setup),
    ])
