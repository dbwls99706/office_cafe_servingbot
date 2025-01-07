from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_serving',
            executable='server_monitor',
            name='server_monitor'
        ),
        Node(
            package='turtle_serving',
            executable='gazebo_sub',
            name='gazebo_sub'
        )
    ])
