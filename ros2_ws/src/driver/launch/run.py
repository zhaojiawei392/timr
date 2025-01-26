from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='your_node_name',
            name='custom_node_name',
            parameters=[{
                'param1': 42,
                'param2': 'hello'
            }],
            remappings=[
                ('original_topic', 'remapped_topic')
            ]
        )
    ])