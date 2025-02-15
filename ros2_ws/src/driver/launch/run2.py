from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('driver')
    
    # Define the path to the config file
    config_file = os.path.join(package_share_dir, 'config', 'driver_node2.yaml')
    
    return LaunchDescription([
        Node(
            package='driver',
            executable='driver_node',
            name='driver_node2',
            parameters=[config_file],
            output='screen',
        )
    ])