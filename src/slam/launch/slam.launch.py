import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace       # 给组内所有节点加命名空间前缀，多机器人隔离用
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration  # launch 参数的占位符，perform(context) 后变成真实字符串
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,   # 声明参数，使其可从命令行用 param:=value 覆盖
    IncludeLaunchDescription, # 嵌套调用另一个 launch 文件
    GroupAction,              # 把多个 action 打包，统一加 namespace
    OpaqueFunction,           # 把普通 Python 函数包装成 launch action，让参数在运行时才求值
    TimerAction,              # 延迟若干秒后再执行某些 action
)


def launch_setup(context):
    # need_compile=True 时从 install 目录找包路径（colcon build 之后）
    # need_compile=False 时直接用源码路径，适合边改边调
    compiled = os.environ.get('need_compile', 'False')

    # perform(context) 把占位符立即解析成字符串，default= 是命令行未传时的兜底值
    enable_save = LaunchConfiguration('enable_save', default='true').perform(context)        # 是否允许保存地图
    slam_method = LaunchConfiguration('slam_method', default='slam_toolbox').perform(context) # SLAM 算法选择
    sim         = LaunchConfiguration('sim',         default='false').perform(context)        # 是否仿真模式
    master_name = LaunchConfiguration('master_name', default=os.environ.get('MASTER', '/')).perform(context) # 多机主机名
    robot_name  = LaunchConfiguration('robot_name',  default=os.environ.get('HOST',   '/')).perform(context) # 本机名

    # 重新声明为 launch 参数，否则命令行传入的值会被忽略
    enable_save_arg = DeclareLaunchArgument('enable_save', default_value=enable_save)
    slam_method_arg = DeclareLaunchArgument('slam_method', default_value=slam_method)
    sim_arg         = DeclareLaunchArgument('sim',         default_value=sim)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg  = DeclareLaunchArgument('robot_name',  default_value=robot_name)

    # 单机时 robot_name='/'，frame_prefix 为空，坐标系名就是 map/odom/base_footprint
    # 多机时 robot_name='robot1'，坐标系变成 robot1/map 等，防止不同机器人 TF 冲突
    frame_prefix = '' if robot_name == '/' else f'{robot_name}/'
    use_sim_time = 'true' if sim == 'true' else 'false'
    map_frame    = f'{frame_prefix}map'
    odom_frame   = f'{frame_prefix}odom'
    base_frame   = f'{frame_prefix}base_footprint'

    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'

    # 基座 launch：启动底盘驱动 + 传感器 + EKF（对应 slam/launch/include/robot.launch.py）
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim':         sim,
            'master_name': master_name,
            'robot_name':  robot_name
        }.items(),
    )

    # SLAM launch：启动 SLAM Toolbox 建图节点（对应 slam/launch/include/slam_base.launch.py）
    # scan_topic 必须和雷达驱动实际发布的话题名一致，不一致会报 TF 错误
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/slam_base.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_frame':    map_frame,
            'odom_frame':   odom_frame,
            'base_frame':   base_frame,
            'scan_topic':   f'{frame_prefix}scan_raw',  # 雷达话题名变了就改这里
            'enable_save':  enable_save
        }.items(),
    )

    if slam_method == 'slam_toolbox':
        bringup_launch = GroupAction(
            actions=[
                PushRosNamespace(robot_name),
                base_launch,              # 立即启动底盘 + 传感器 + EKF
                TimerAction(
                    period=10.0,          # 延迟 10 秒，等底盘和雷达就绪后再启动 SLAM
                    actions=[slam_launch],
                ),
            ]
        )

    # 返回参数声明 + 启动动作（参数声明必须包含在返回列表里才能被命令行识别）
    return [sim_arg, master_name_arg, robot_name_arg, slam_method_arg, bringup_launch]


def generate_launch_description():
    # OpaqueFunction 的作用：让 launch_setup 在运行时才执行
    # 这样 perform(context) 才能拿到真实的参数值，而不是占位符字符串
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    # 直接用 python 运行此文件时的入口（非 ros2 launch 命令）
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

