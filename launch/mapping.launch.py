from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='falcon_mapping_node',
            executable='mapping_node',
            output='screen'
        )
    ])
