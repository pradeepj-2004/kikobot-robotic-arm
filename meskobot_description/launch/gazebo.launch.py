from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler,ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
import os
from launch.actions import DeclareLaunchArgument

import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('meskobot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_world_dir=get_package_share_directory('eyantra_warehouse')
    xacro_file = os.path.join(share_dir, 'urdf', 'meskobot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )
    world = LaunchConfiguration('world')

    # Launch argument for world file
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_world_dir, 'worlds', 'eyantra_warehouse_task4.world'),
        description='Path to the world file to load in Gazebo'
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),launch_arguments={'world': world}.items()
    ) 



    # gazebo_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gzserver.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'pause': 'false'
    #     }.items()
    # )

    # gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gzclient.launch.py'
    #         ])
    #     ])
    # )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'meskobot',
            '-topic', 'robot_description','-x', '1.6', '-y', '-2.4', '-z', '0.58', '-Y', '3.14'
        ],
        output='screen'
    )
    
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    
    return LaunchDescription([
    	RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawn_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
           event_handler=OnProcessExit(
               target_action=load_joint_state_broadcaster,
               on_exit=[load_joint_trajectory_controller],
          )
        ),




        robot_state_publisher_node,
        declare_world_arg,
        gazebo,
        urdf_spawn_node,
        rviz_node
    ])
