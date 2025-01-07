from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_table',
            executable='tableorder',
            name='tableorder_final'
        ),
        Node(
            package='turtle_table',
            executable='send',
            name='send_order'
        )
    ])
