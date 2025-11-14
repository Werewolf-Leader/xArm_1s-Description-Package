from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xarm_remote',
            executable='task_server',
            name='xarm_task_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
    ])
