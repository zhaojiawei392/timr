from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('kinematics')
    
    # Define the path to the config file
    config_file = os.path.join(package_share_dir, 'config', 'kinematics_node.yaml')
    
    # Add launch argument for log level
    log_level = LaunchConfiguration('log_level')
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value=['info'],
        description='Logging level'
    )
    
    return LaunchDescription([
        log_level_arg,
        Node(
            package='kinematics',
            executable='kinematics_node',
            name='kinematics_node',
            parameters=[config_file],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        )
    ])