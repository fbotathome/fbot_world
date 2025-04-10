from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from scripts.marge_yaml_files import MergeYamlFiles
import os





def generate_launch_description():
    node_name = 'face_recognition_writer'
    plugin_config_file_path = os.path.join(
        get_package_share_directory('fbot_world'),
        'config',
        'plugin.yaml'
    )

    #Definir caminho vindo do behavior
    config_launch_file_path = os.path.join(
        get_package_share_directory('fbot_world'),
        'config',
        'face_recognition.yaml'
    )

    config_file_path = os.path.join(get_package_share_directory('fbot_world'),
        'config',
        node_name+'.yaml'
    )
    
    MergeYamlFiles(config_launch_file_path,
                    plugin_config_file_path,
                    config_file_path,
                    node_name)

    

    config_file_arg = DeclareLaunchArgument(
        'config',
        default_value=config_file_path,
        description='Path to the parameter file'
    )

    
    face_recognition_writer_node = Node(
        package='fbot_world',
        executable=node_name,
        name=node_name,
        parameters=[LaunchConfiguration('config')],
    )
    return LaunchDescription([
        config_file_arg,
        face_recognition_writer_node
    ])
