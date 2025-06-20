from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os



def generate_launch_description():

    plugin_file_arg = DeclareLaunchArgument(
        'plugin_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_world_bringup'), 'config', 'plugin.yaml']),
        description='Path to the World Plugin parameter file'
    )

    pose_node = Node(
        package='fbot_world_bringup',
        executable='pose',
        name='pose',
        parameters=[LaunchConfiguration('plugin_config'),
                    {'config_file_name': LaunchConfiguration('config_file_name')}],
    )
    
    return LaunchDescription([
        plugin_file_arg,
        pose_node
    ])
