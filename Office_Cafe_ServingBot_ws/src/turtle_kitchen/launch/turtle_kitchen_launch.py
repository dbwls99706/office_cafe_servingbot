from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_kitchen',
            executable='main_gui',
            name='main_gui'
        ),
        Node(
            package='turtle_kitchen',
            executable='sub_data',
            name='sub_data'
        ),
        Node(
            package='turtle_kitchen',
            executable='check',
            name='check_order'
        ),
        Node(
            package='turtle_kitchen',
            executable='go_table',
            name='send_goal'
        )
    ])
