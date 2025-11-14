from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Include MoveIt demo launch
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit'),
                'launch',
                'demo.launch.py'
            ])
        ])
    )

    # Task server node
    task_server = Node(
        package='xarm_remote',
        executable='task_server',
        name='xarm_task_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        moveit_demo,
        task_server,
    ])
