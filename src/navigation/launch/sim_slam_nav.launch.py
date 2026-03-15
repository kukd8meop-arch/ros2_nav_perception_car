"""
sim_slam_nav.launch.py — Gazebo + SLAM 建图 + Nav2 规划/控制 + RViz2
=====================================================================

与 sim_nav_full.launch.py 的区别：
  - 不预先加载地图文件
  - 用 slam_toolbox（sync 模式）实时建图，同时发布 map → odom TF
  - Nav2 只启动规划/控制部分（navigation_base），不启动 AMCL / map_server
  - 机器人一边开着 Nav2 目标点导航，一边在 RViz2 里看到地图实时生长

数据流
------
  Gazebo → /scan_raw → slam_toolbox → /map + map→odom TF
                    ↘
                     Nav2 (planner + controller) → /cmd_vel → Gazebo

用法
----
  bash sim_nav.sh mode:=slam                     # 默认世界 sim_empty
  bash sim_slam_nav.sh map:=map_test             # 指定 Gazebo 世界文件（不再预加载地图）
  ros2 launch navigation sim_slam_nav.launch.py use_gui:=false
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
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes


# ─── 工具函数：给 yaml 注入 use_sim_time=true ─────────────────────────────────
def _patch_yaml_for_sim(src_path: str) -> str:
    with open(src_path) as f:
        cfg = yaml.safe_load(f)
    def _set_sim_time(d):
        if isinstance(d, dict):
            if 'ros__parameters' in d:
                d['ros__parameters']['use_sim_time'] = True
                # 修复 nav2 空列表序列化 bug：
                # LoadComposableNodes 不支持空 list（被转为 tuple()）
                keys_to_fix = []
                for k, v in d['ros__parameters'].items():
                    if v == ['']:
                        keys_to_fix.append((k, []))
                    elif isinstance(v, list) and len(v) == 0:
                        keys_to_fix.append((k, None))  # 标记删除
                for k, newv in keys_to_fix:
                    if newv is None:
                        del d['ros__parameters'][k]
                    else:
                        d['ros__parameters'][k] = newv
            for v in d.values():
                _set_sim_time(v)
    _set_sim_time(cfg)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='nav2_sim_')
    yaml.dump(cfg, tmp)
    tmp.flush()
    return tmp.name


# ─── 等待 Gazebo 就绪脚本（与 sim_nav_full 相同逻辑） ────────────────────────
def _create_wait_script() -> str:
    code = r"""
import os, sys, time, subprocess

TIMEOUT = 180

env = os.environ.copy()

def run(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env)
        return r.returncode, r.stdout
    except subprocess.TimeoutExpired:
        return -1, ""

start = time.time()
last_print = 0

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
        print(f"[wait] 已等待 {elapsed}s ...", flush=True)
    time.sleep(2)
else:
    print("[wait] 超时：/odom 话题未出现", flush=True)
    sys.exit(1)

print("[wait] /odom 已出现，等待 3s 让插件稳定...", flush=True)
time.sleep(3)
print("[wait] Gazebo 完全就绪！", flush=True)
"""
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.py', delete=False, prefix='wait_gz_')
    tmp.write(code)
    tmp.flush()
    return tmp.name


# ─── 核心 launch 组装 ─────────────────────────────────────────────────────────
def launch_setup(context):
    nav_pkg  = get_package_share_directory('navigation')
    slam_pkg = get_package_share_directory('slam')
    desc_pkg = get_package_share_directory('rosorin_description')

    # ── 读取参数 ───────────────────────────────────────────────────────────
    map_name    = LaunchConfiguration('map').perform(context)
    use_gui     = LaunchConfiguration('use_gui').perform(context)
    use_teb     = LaunchConfiguration('use_teb').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)
    use_gz      = LaunchConfiguration('use_gz').perform(context)
    x_pose      = LaunchConfiguration('x_pose').perform(context)
    y_pose      = LaunchConfiguration('y_pose').perform(context)
    yaw         = LaunchConfiguration('yaw').perform(context)

    # ── 参数文件（注入 use_sim_time） ──────────────────────────────────────
    nav2_params_src = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    nav2_params_sim = _patch_yaml_for_sim(nav2_params_src)

    ctrl_yaml_name = 'nav2_controller_teb.yaml' if use_teb == 'true' \
                     else 'nav2_controller_dwb.yaml'
    ctrl_yaml_sim = _patch_yaml_for_sim(
        os.path.join(nav_pkg, 'config', ctrl_yaml_name))

    # slam_toolbox 参数：复用 slam 包的 slam.yaml，覆盖仿真相关字段
    slam_yaml_src = os.path.join(slam_pkg, 'config', 'slam.yaml')
    with open(slam_yaml_src) as f:
        slam_cfg = yaml.safe_load(f)
    # 注入仿真参数
    params = slam_cfg.setdefault('/**', {}).setdefault('ros__parameters', {})
    params['use_sim_time'] = True
    params['map_frame']    = 'map'
    params['odom_frame']   = 'odom'
    params['base_frame']   = 'base_footprint'
    params['scan_topic']   = '/scan_raw'
    params['mode']         = 'mapping'
    slam_params_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='slam_sim_')
    yaml.dump(slam_cfg, slam_params_tmp)
    slam_params_tmp.flush()
    slam_params_file = slam_params_tmp.name

    # ── RViz2 配置 ─────────────────────────────────────────────────────────
    rviz_config = os.path.join(nav_pkg, 'rviz', 'navigation.rviz')

    # ── 等待脚本 ───────────────────────────────────────────────────────────
    wait_script = _create_wait_script()

    # ── ① Gazebo 世界文件（与 sim_nav_full 相同的同名匹配逻辑） ───────────
    if use_gz == 'true':
        candidate = os.path.join(desc_pkg, 'worlds', map_name + '_gz.sdf')
        sim_world = candidate if os.path.isfile(candidate) \
            else os.path.join(desc_pkg, 'worlds', 'sim_empty_gz.sdf')
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'gazebo_gz.launch.py')),
            launch_arguments={
                'world': sim_world, 'use_gui': use_gui,
                'use_sim_time': 'true',
                'x_pose': x_pose, 'y_pose': y_pose, 'z_pose': '0.05', 'yaw': yaw,
            }.items(),
        )
    else:
        candidate = os.path.join(desc_pkg, 'worlds', map_name + '.world')
        sim_world = candidate if os.path.isfile(candidate) \
            else os.path.join(desc_pkg, 'worlds', 'sim_empty.world')
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': sim_world, 'use_gui': use_gui,
                'use_sim_time': 'true',
                'x_pose': x_pose, 'y_pose': y_pose, 'z_pose': '0.05', 'yaw': yaw,
            }.items(),
        )

    # ── ② 等待 Gazebo 就绪 ─────────────────────────────────────────────────
    wait_for_gazebo = ExecuteProcess(
        cmd=['python3', wait_script],
        output='screen',
        name='wait_for_gazebo_ready',
    )

    # ── ③ slam_toolbox：实时建图，同时发布 map→odom TF ────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    # ── ④ Nav2：直接内联启动规划+控制（不通过 bringup/navigation_base）───
    #    slam_toolbox 已经在发布 /map 和 map→odom TF
    #    避免 navigation_base.launch.py 内部 RewrittenYaml tuple 兼容性问题

    nav2_container_name = 'nav2_container'
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    bt_xml_path = os.path.join(nav_pkg, 'config', 'navigate_to_pose_minimal.xml')

    nav_container = Node(
        name=nav2_container_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[nav2_params_sim, {'autostart': True, 'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=remappings,
        output='screen',
    )

    lifecycle_nodes = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother',
    ]

    load_nav2_nodes = LoadComposableNodes(
        target_container=nav2_container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[ctrl_yaml_sim],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[nav2_params_sim],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[nav2_params_sim],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[nav2_params_sim],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[nav2_params_sim, {
                    'default_nav_to_pose_bt_xml': bt_xml_path,
                }],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[nav2_params_sim],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[nav2_params_sim],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                }]),
        ],
    )

    # ── ⑤ cmd_vel 转发 ────────────────────────────────────────────────────
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/cmd_vel', '/controller/cmd_vel'],
    )

    # ── ⑥ RViz2 ───────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # ── 组装：Gazebo 就绪后同时启动 slam + Nav2 + relay ───────────────────
    on_ready = [
        slam_node,        # 建图（提供 /map 和 map→odom TF）
        cmd_vel_relay,
        TimerAction(period=3.0, actions=[nav_container]),   # 先启动容器
        TimerAction(period=5.0, actions=[load_nav2_nodes]), # 容器启动后加载 composable nodes
    ]
    if launch_rviz == 'true':
        on_ready.append(TimerAction(period=5.0, actions=[rviz_node]))

    return [
        gazebo_launch,
        wait_for_gazebo,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_gazebo,
                on_exit=on_ready,
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('need_compile', 'True'),

        DeclareLaunchArgument('map',         default_value='sim_empty',
            description='Gazebo 世界文件名前缀（worlds/<map>_gz.sdf），不再用于加载预建地图'),
        DeclareLaunchArgument('use_gui',     default_value='true'),
        DeclareLaunchArgument('use_teb',     default_value='false'),
        DeclareLaunchArgument('x_pose',      default_value='0.0'),
        DeclareLaunchArgument('y_pose',      default_value='0.0'),
        DeclareLaunchArgument('yaw',         default_value='0.0'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('use_gz',      default_value='true'),

        OpaqueFunction(function=launch_setup),
    ])
