from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='192.168.4.1',
                              description='Car TCP host'),
        DeclareLaunchArgument('port', default_value='23',
                              description='Car TCP port'),
        DeclareLaunchArgument('auto_connect', default_value='true',
                              description='Connect on startup'),

        Node(
            package='umbreon_bridge',
            executable='bridge_node',
            name='umbreon_bridge',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'auto_connect': LaunchConfiguration('auto_connect'),
            }],
        ),
    ])
