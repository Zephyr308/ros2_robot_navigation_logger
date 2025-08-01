from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stopper',
            executable='stopper_node',
            name='stopper_node',
            output='screen'
        )
    ])
