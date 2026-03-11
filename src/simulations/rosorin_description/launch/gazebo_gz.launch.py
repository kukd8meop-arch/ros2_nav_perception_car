"""
gazebo_gz.launch.py — Ignition Gazebo (gz-sim6 / Fortress) 仿真启动
====================================================================

替代 gazebo.launch.py（Classic 版），适配 ros-humble-ros-gz。

与 Classic 版的区别：
  - 使用 ign gazebo 替代 gzserver + gzclient
  - 使用 ros_gz_sim create 替代 gazebo_ros spawn_entity
  - 使用 ros_gz_bridge 桥接 Ignition Transport ↔ ROS2 话题
  - 渲染与物理完全解耦，大型 STL 不再拖慢仿真

话题桥接映射：
  Ignition 话题                      ROS2 话题             方向
  ──────────────────────────────────────────────────────────────
  /controller/cmd_vel                /model/rosorin/cmd_vel ROS→IGN
  /odom                              /model/rosorin/odometry IGN→ROS
  /scan_raw                          /lidar                 IGN→ROS
  /imu                               /imu                   IGN→ROS
  /clock                             /clock                 IGN→ROS

环境变量：
  IGN_GAZEBO_RESOURCE_PATH — 让 Ignition 找到 package:// 引用的 mesh 文件
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 包路径解析 ─────────────────────────────────────────────────────────
    compiled = os.environ.get('need_compile', 'True')
    if compiled == 'True':
        description_package_path = get_package_share_directory('rosorin_description')
    else:
        description_package_path = '/home/ubuntu/ros2_ws/src/simulations/rosorin_description'

    # 默认世界文件和 Ignition 版 xacro
    default_world = os.path.join(description_package_path, 'worlds', 'sim_empty_gz.sdf')
    xacro_file = os.path.join(description_package_path, 'urdf', 'rosorin.gz.xacro')

    # ── Ignition 资源路径 ─────────────────────────────────────────────────
    # Ignition 把 URDF 中的 package://pkg_name/path 转换为在
    # IGN_GAZEBO_RESOURCE_PATH 中查找 pkg_name/path
    # 所以我们需要把 share/ 目录的父目录加入资源路径
    # 例如: share/rosorin_description/meshes/base_link_diff.stl
    #       对应 IGN_GAZEBO_RESOURCE_PATH 里要有 share/ 所在的目录
    ign_resource_path = os.path.dirname(description_package_path)  # share/ 目录
    existing_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if existing_path:
        ign_resource_path = ign_resource_path + ':' + existing_path

    # ── launch 参数 ────────────────────────────────────────────────────────
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    machine_type = LaunchConfiguration('machine_type')
    use_gui = LaunchConfiguration('use_gui')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')

    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_machine_type = DeclareLaunchArgument('machine_type', default_value='ROSOrin_Differential')
    declare_use_gui = DeclareLaunchArgument('use_gui', default_value='true')
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z_pose = DeclareLaunchArgument('z_pose', default_value='0.05')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    declare_odom_frame = DeclareLaunchArgument('odom_frame', default_value='odom')
    declare_base_frame = DeclareLaunchArgument('base_frame', default_value='base_footprint')
    declare_lidar_frame = DeclareLaunchArgument('lidar_frame', default_value='lidar_frame')

    # =========================================================================
    # ① Ignition Gazebo 启动
    #    使用 ros_gz_sim 提供的 gz_sim.launch.py
    #    gz_args 格式：<world_file> [-r] [-s] [--headless-rendering]
    #      -r   = 启动后自动运行仿真（不暂停）
    #      -s   = headless（无 GUI），仅运行服务端
    # =========================================================================
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': [world, ' -r'],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # =========================================================================
    # ② 用 xacro 生成 robot_description（Ignition 版插件）
    # =========================================================================
    robot_description = Command([
        'xacro ', xacro_file,
        ' odom_frame:=', odom_frame,
        ' base_frame:=', base_frame,
        ' lidar_frame:=', lidar_frame,
    ])

    # =========================================================================
    # ③ joint_state_publisher + robot_state_publisher
    # =========================================================================
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # =========================================================================
    # ④ 在 Ignition 世界中生成机器人模型
    #    使用 ros_gz_sim create 节点，从 /robot_description 话题读取 SDF/URDF
    # =========================================================================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rosorin',
        output='screen',
        arguments=[
            '-world', 'sim_empty_world',       # 与 SDF 世界名一致
            '-topic', 'robot_description',     # 从 robot_state_publisher 获取模型
            '-name', 'rosorin',                # 实体名
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ],
    )

    # =========================================================================
    # ⑤ ros_gz_bridge：桥接 Ignition Transport ↔ ROS2 话题
    #
    #    语法：topic@ROS2_type@Ign_type
    #      @  = 双向
    #      [  = Ign → ROS
    #      ]  = ROS → Ign
    #
    #    注意：remappings 在 ros_gz_bridge 中作用于 ROS2 侧的话题名。
    #    桥接参数中写的是 Ignition Transport 侧的话题名。
    #    remappings 把桥接后的 ROS2 话题重命名。
    #
    #    关键桥接：
    #      cmd_vel:  ROS → Ign（Nav2 → Gazebo 运动控制）
    #      odometry: Ign → ROS（Gazebo → AMCL 定位）
    #      lidar:    Ign → ROS（激光扫描数据）
    #      imu:      Ign → ROS（IMU 数据）
    #      clock:    Ign → ROS（仿真时钟）
    # =========================================================================
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # 速度指令：ROS → Ignition（Nav2 → Gazebo 运动控制）
            '/model/rosorin/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # 里程计：Ignition → ROS（Gazebo → AMCL 定位）
            '/model/rosorin/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # 激光雷达：Ignition → ROS
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # IMU：Ignition → ROS
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # 仿真时钟：Ignition → ROS
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        # 话题重映射：让桥接后的 ROS 话题名与 Classic 版一致
        remappings=[
            ('/model/rosorin/cmd_vel', '/controller/cmd_vel'),
            ('/model/rosorin/odometry', '/odom'),
            ('/lidar', '/scan_raw_gz'),
        ],
    )

    # =========================================================================
    # ⑥-b scan_frame_fix：修正 Ignition 的 scoped frame_id
    #    Ignition 的 gpu_lidar 使用 scoped 命名，例如：
    #      rosorin/base_footprint/lidar
    #    但 Nav2 / AMCL 的 costmap 和 TF 树期望的是 URDF 中定义的 link 名：
    #      lidar_frame
    #    scan_frame_fix 节点：/scan_raw_gz (bridge) → /scan_raw (frame_id=lidar_frame)
    # =========================================================================
    scan_frame_fix = Node(
        package='rosorin_description',
        executable='scan_frame_fix',
        name='scan_frame_fix',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': lidar_frame,
        }],
    )

    # =========================================================================
    # ⑥ odom → base_footprint TF 发布
    #    Ignition 的 odometry-publisher 通过 Ignition Transport 发布 TF，
    #    但 ros_gz_bridge 无法直接桥接 Ignition TF 到 ROS2 /tf。
    #    用 odom_to_tf 节点：订阅 /odom → 发布 odom→base_footprint TF
    # =========================================================================
    odom_to_tf = Node(
        package='rosorin_description',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # =========================================================================
    # 组装 launch
    # =========================================================================
    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        declare_machine_type,
        declare_use_gui,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_yaw,
        declare_odom_frame,
        declare_base_frame,
        declare_lidar_frame,

        # 环境变量
        SetEnvironmentVariable('MACHINE_TYPE', machine_type),
        # Ignition 资源路径：让 Ignition 找到 package:// 引用的 mesh
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),

        # 启动节点
        gz_sim_launch,          # ① Ignition Gazebo
        joint_state_publisher,  # ③ 关节状态
        robot_state_publisher,  # ③ 机器人状态 + TF
        spawn_entity,           # ④ 生成机器人
        gz_bridge,              # ⑤ 话题桥接
        scan_frame_fix,         # ⑥-b scan frame_id 修正
        odom_to_tf,             # ⑥ odom→base_footprint TF
    ])
