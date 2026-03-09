import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 根据当前运行模式，决定是从已安装包路径还是源码路径读取资源文件。
    compiled = os.environ.get('need_compile', 'True')

    if compiled == 'True':
        description_package_path = get_package_share_directory('rosorin_description')
    else:
        description_package_path = '/home/ubuntu/ros2_ws/src/simulations/rosorin_description'

    # 解析默认 world 文件和 Gazebo 版机器人 xacro 文件路径。
    default_world = os.path.join(description_package_path, 'worlds', 'empty.world')
    xacro_file = os.path.join(description_package_path, 'urdf', 'rosorin.gazebo.xacro')

    # 定义本 launch 会用到的动态参数，便于命令行覆盖默认值。
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

    # 把上述参数正式声明为 launch 参数，提供默认值。
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

    # 用来开 Gazebo 仿真软件，并且加载地图和机器人模型的 launch 文件。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world, # 指令1：加载哪个地图
            'verbose': 'false', # 指令2：关闭啰嗦日志
            'gui': use_gui, # 指令3：是否打开可视化界面
        }.items(), # 把 3 条指令，转换成 ROS2 能看懂的格式（系统要求的，固定写法）
    )

    # 调用 xacro 生成带 Gazebo 插件的 robot_description，并把关键 frame 传入模型。
    robot_description = Command([
        'xacro ',
        xacro_file,
        ' odom_frame:=', odom_frame,
        ' base_frame:=', base_frame,
        ' lidar_frame:=', lidar_frame,
    ])

    # 启动 joint_state_publisher，给机器人模型提供基础关节状态。
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 启动 robot_state_publisher，把 robot_description 发布为 TF 树。
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

    # 调用 gazebo_ros 的 spawn_entity.py，把机器人实体真正生成到 Gazebo 世界中。
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rosorin',
        output='screen',
        arguments=[
            '-entity', 'rosorin',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ],
    )

    # 组装整个 launch：先声明参数，再设置车型环境变量，最后启动 Gazebo 和机器人相关节点。
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
        SetEnvironmentVariable('MACHINE_TYPE', machine_type),
        gazebo,
        joint_state_publisher,
        robot_state_publisher,
        spawn_entity,
    ])
