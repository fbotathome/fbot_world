from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from scripts.marge_yaml_files import mergeYamlFiles
import os


def generate_launch_description():
    node_name = 'pose'
    #Definir node_name vindo do behavior

    plugin_config_file_path = os.path.join(
        get_package_share_directory('fbot_world'),
        'config',
        'plugin.yaml'
    )

    config_launch_file_path = os.path.join(
        get_package_share_directory('fbot_world'),
        'config',
        node_name + '.yaml'
    )

    data = mergeYamlFiles(config_launch_file_path,
                    plugin_config_file_path,)

    pose_node = Node(
        package='fbot_world',
        executable=node_name,
        name=node_name,
        parameters=[data],
    )
    
    return LaunchDescription([
        pose_node
    ])