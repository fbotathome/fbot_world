# #!/usr/bin/env python3
# import os
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

# def generate_launch_description():
#     config_dir = get_package_share_directory('fbot_world')

#     DeclareLaunchArgument(
#             'config_name',
#             default_value='pose',
#             description='Configuration file with poses.'
#         ),

#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(config_dir,"config", 'plugin.yaml'))
#         ),
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(config_dir,"config", 'pose.yaml'))
#         ),

#         OpaqueFunction(function=lambda context: [
#             SetLaunchConfiguration(
#                 'param', os.path.join(
#                     config_dir, 'config',
#                     context.launch_configurations['config_name'] + '.yaml'
#                 )
#             )
#         ]),
        
#         Node(
#             package='fbot_world',
#             executable='pose.py',
#             name='pose',
#             output='screen'
#         )
#         ])



from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from scripts.marge_yaml_files import mergeYamlFiles
import os





def generate_launch_description():
    node_name = 'pose'

    plugin_config_file_path = os.path.join(
        get_package_share_directory('fbot_world'),
        'config',
        'plugin.yaml'
    )

    #Definir caminho vindo do behavior
    config_launch_file_path = os.path.join(
        get_package_share_directory('fbot_world'),
        'config',
        'pose.yaml'
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
