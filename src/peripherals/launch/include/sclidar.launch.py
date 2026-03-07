#!/usr/bin/env python3
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    lidar_frame = LaunchConfiguration('lidar_frame', default='base_laser')
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')

    serial_port = LaunchConfiguration('serial_port', default='/dev/lidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='230400')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    version = LaunchConfiguration('version', default='4')

    min_angle = LaunchConfiguration('min_angle', default='0.0')
    max_angle = LaunchConfiguration('max_angle', default='360.0')
    min_range = LaunchConfiguration('min_range', default='0.1')
    max_range = LaunchConfiguration('max_range', default='10.0')
    flip_enable = LaunchConfiguration('flip_enable', default='true')

    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value=lidar_frame)
    scan_raw_arg = DeclareLaunchArgument('scan_raw', default_value=scan_raw)

    serial_port_arg = DeclareLaunchArgument('serial_port', default_value=serial_port)
    serial_baudrate_arg = DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate)
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value=frame_id)
    version_arg = DeclareLaunchArgument('version', default_value=version)

    min_angle_arg = DeclareLaunchArgument('min_angle', default_value=min_angle)
    max_angle_arg = DeclareLaunchArgument('max_angle', default_value=max_angle)
    min_range_arg = DeclareLaunchArgument('min_range', default_value=min_range)
    max_range_arg = DeclareLaunchArgument('max_range', default_value=max_range)
    flip_enable_arg = DeclareLaunchArgument('flip_enable', default_value=flip_enable)

    sclidar_node = Node(
        package='sclidar_ros2',
        executable='sclidar',
        name='sclidar_node',
        output='screen',
        parameters=[
            {
                'topic_name': 'scan',             
                'frame_id': frame_id,            
                'port': serial_port,
                'baudrate': serial_baudrate,
                'version': version,
                'min_angle': min_angle,
                'max_angle': max_angle,
                'min_range': min_range,
                'max_range': max_range,
                'flip_enable': flip_enable,
            }
        ],
        remappings=[
            ('scan', scan_raw)
        ]
    )

    return LaunchDescription([
        lidar_frame_arg,
        scan_raw_arg,

        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        version_arg,
        min_angle_arg,
        max_angle_arg,
        min_range_arg,
        max_range_arg,
        flip_enable_arg,

        sclidar_node,
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
