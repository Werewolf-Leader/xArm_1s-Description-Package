from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xarm_remote',
            executable='task_server_number',
            name='task_server_number',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
    ])
