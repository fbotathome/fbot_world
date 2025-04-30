from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from scripts.merge_yaml_files import mergeYamlFiles
import os


def node_setup(context):
    config_file_name = LaunchConfiguration('config_file_name').perform(context)
<<<<<<< HEAD:fbot_world_bringup/launch/pose.launch.py
    print(f'LaunchConfiguration: {config_file_name}')
=======
>>>>>>> 0c11a0f4beef5193ef9075dd6107f32026321734:launch/pose.launch.py
    plugin_config_file_path = os.path.join(
        get_package_share_directory('fbot_world'),
        'config',
        'plugin.yaml'
    )

    config_launch_file_path = os.path.join(
        get_package_share_directory('fbot_world'),
        'config',
        config_file_name + '.yaml'
    )

    data = mergeYamlFiles(config_launch_file_path,
                    plugin_config_file_path,)

    pose_node = Node(
        package='fbot_world',
        executable='pose',
        name='pose',
        parameters=[data],
    )
    return [pose_node]

def generate_launch_description():
    opfunc = OpaqueFunction(function = node_setup)
    
    return LaunchDescription([
        opfunc
    ])
