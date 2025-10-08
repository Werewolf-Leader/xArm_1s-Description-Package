import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("xarm_description"), "urdf", "xarm.xacro"]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    controller_config = PathJoinSubstitution(
        [FindPackageShare("xarm_description"), "launch", "controller.yaml"]
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Gazebo server
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={'verbose': 'false'}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "xarm"],
        output="screen",
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Robot controller
    xarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of controllers after spawn
    delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_xarm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[xarm_controller_spawner],
        )
    )

    nodes = [
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        delay_joint_state_broadcaster_after_spawn,
        delay_xarm_controller_after_joint_state_broadcaster,
    ]

    return LaunchDescription(nodes)
