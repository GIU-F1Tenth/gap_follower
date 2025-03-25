import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    package_share_directory = get_package_share_directory('gap_follower')
    parameters_file_path = os.path.join(package_share_directory, 'config', 'gap_follower_config.yaml')

    gap_follower = Node(
        package='gap_follower',
        executable='steering_speed_exe',
        name='gap_steering_node',
        parameters=[parameters_file_path]
    )

    ld.add_action(gap_follower)
    
    return ld