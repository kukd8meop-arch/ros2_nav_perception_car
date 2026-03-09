import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction

<<<<<<< HEAD

def launch_setup(context):
    # 判断当前是使用 colcon 安装后的包路径，还是直接使用源码路径。
=======
def launch_setup(context):
>>>>>>> 53588069a7a4986dfeeaf6de6cf4822d10442588
    compiled = os.environ['need_compile']
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
        navigation_package_path = get_package_share_directory('navigation')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        navigation_package_path = '/home/ubuntu/ros2_ws/src/navigation'

<<<<<<< HEAD
    # 从 launch 参数中取出运行模式、地图名、机器人名、主控名以及是否使用 TEB 控制器。
=======
>>>>>>> 53588069a7a4986dfeeaf6de6cf4822d10442588
    sim = LaunchConfiguration('sim', default='false').perform(context)
    map_name = LaunchConfiguration('map', default='map_01').perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST']).perform(context)
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER']).perform(context)
    use_teb = LaunchConfiguration('use_teb', default='true').perform(context)

<<<<<<< HEAD
    # 把这些参数声明出来，这样在命令行 ros2 launch 时就可以覆盖默认值。
=======
>>>>>>> 53588069a7a4986dfeeaf6de6cf4822d10442588
    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)
    use_teb_arg = DeclareLaunchArgument('use_teb', default_value=use_teb)

<<<<<<< HEAD
    # 仿真模式下开启 use_sim_time；多机器人场景下根据 robot_name 决定是否启用命名空间。
    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'
    
    # 先启动底盘与传感器基座层：底盘驱动、雷达、IMU、EKF 等都在这里面。
=======
    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'
    
>>>>>>> 53588069a7a4986dfeeaf6de6cf4822d10442588
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim,
            'master_name': master_name,
            'robot_name': robot_name
        }.items(),
    )
    
<<<<<<< HEAD
    # 再启动导航层：加载地图、Nav2 参数、命名空间以及局部控制器配置。
=======
>>>>>>> 53588069a7a4986dfeeaf6de6cf4822d10442588
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': os.path.join(slam_package_path, 'maps', map_name + '.yaml'),
            'params_file': os.path.join(navigation_package_path, 'config', 'nav2_params.yaml'),
            'namespace': robot_name,
            'use_namespace': use_namespace,
            'autostart': 'true',
            'use_teb': use_teb,
        }.items(),
    )

<<<<<<< HEAD
    # 用 GroupAction 把整套导航系统包起来，统一挂到机器人命名空间下。
    bringup_launch = GroupAction(
     actions=[
         # 例如 robot_name=robot_1 时，下面所有节点都会进入 /robot_1 命名空间。
=======
    bringup_launch = GroupAction(
     actions=[
>>>>>>> 53588069a7a4986dfeeaf6de6cf4822d10442588
         PushRosNamespace(robot_name),
         base_launch,
         TimerAction(
             period=10.0,  # 延时等待其它节点启动好(delay for enabling other nodes)
<<<<<<< HEAD
             # 等底盘、TF、雷达、EKF 都稳定后，再启动 Nav2，避免一上来就找不到依赖话题或 TF。
=======
>>>>>>> 53588069a7a4986dfeeaf6de6cf4822d10442588
             actions=[navigation_launch],
         ),
      ]
    )

<<<<<<< HEAD
    # 返回本文件声明的参数和最终组装好的启动动作列表。
    return [sim_arg, map_name_arg, master_name_arg, robot_name_arg, use_teb_arg, bringup_launch]


def generate_launch_description():
    # OpaqueFunction 允许在真正 launch 执行时读取 context，并动态生成启动项。
=======
    return [sim_arg, map_name_arg, master_name_arg, robot_name_arg, use_teb_arg, bringup_launch]

def generate_launch_description():
>>>>>>> 53588069a7a4986dfeeaf6de6cf4822d10442588
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
