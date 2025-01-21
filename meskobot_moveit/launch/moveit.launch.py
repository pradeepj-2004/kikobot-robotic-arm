import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml
import xacro

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None
    
def generate_launch_description():

    is_sim = LaunchConfiguration('is_sim')
    
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True'
    )

    moveit_config = (
        MoveItConfigsBuilder("meskobot", package_name="meskobot_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("mesko_hardware"),
            "urdf",
            "meskobot.xacro"
            )
        )
        .robot_description_semantic(file_path="config/meskobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")

        .to_moveit_configs()
    )
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    file_path_yaml=os.path.join(
            get_package_share_directory("meskobot_moveit"),
            "config",
            "ompl_planning.yaml"
            )
    ompl_planning_yaml = load_yaml(file_path_yaml)
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)



    share_dir = get_package_share_directory('meskobot_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'meskobot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    share_dir_s = get_package_share_directory('meskobot_moveit')
    xacro_file_s = os.path.join(share_dir_s, 'config', 'meskobot.srdf')
    robot_description_config_s = xacro.process_file(xacro_file_s)
    robot_description_semantic = robot_description_config_s.toxml()


    file_path_yaml_servo=os.path.join(
            get_package_share_directory("meskobot_moveit"),
            "config",
            "arm_servo.yaml"
            )
            
    servo_yaml = load_yaml(file_path_yaml_servo)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            {"moveit_servo": servo_yaml},
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
        ],
        output="screen",
    )


    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    {'use_sim_time': is_sim},
                    {'publish_robot_description_semantic': True},
                    ompl_planning_pipeline_config
                    ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # # RViz
    rviz_config = os.path.join(
        get_package_share_directory("meskobot_moveit"),
            "config",
            "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node,
            servo_node,
            rviz_node
        ]
    )
