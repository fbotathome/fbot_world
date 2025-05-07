To run pose.launch.py you need the FBOT_BRINGUP package. The config file example is:

pose:
  package_name: fbot_world_bringup
  executable: launch/pose.launch.py
  enable: 'true'
  parameters:
    name: 'pose'
    namespace: 'pose'
    config_file_name : 'pose' #change name to config pose yaml

And the launch on fbot_bringup package is:

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('fbot_bringup')
    config_file = str(Path(package_share_dir) / 'config/test.yaml')

    # Return the launch description
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('fbot_bringup'), 'launch/full_bringup.launch.py']),
            launch_arguments=[('config_file_path', config_file)]
        )
    ])