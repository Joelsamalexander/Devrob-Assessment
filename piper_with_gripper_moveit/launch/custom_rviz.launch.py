import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
 
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    initial_positions_file_path = 'config/initial_positions.yaml'
    # Configure MoveIt with critical fixes
    robot_description_kinematic = {
    "robot_description_kinematic.arm.kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin", 
    "robot_description_kinematic.arm.kinematics_solver_search_resolution": 0.005,
    "robot_description_kinematic.arm.kinematics_solver_timeout": 0.05,
    "robot_description_kinematic.arm.position_only_ik": True,
    
    "robot_description_kinematic.gripper.kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "robot_description_kinematic.gripper.kinematics_solver_search_resolution": 0.005,
    "robot_description_kinematic.gripper.kinematics_solver_timeout": 0.05,
    "robot_description_kinematic.gripper.position_only_ik": True,
     }
    moveit_config = (
        MoveItConfigsBuilder("piper_description", package_name="piper_with_gripper_moveit")
        .robot_description_semantic(file_path="config/piper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .planning_scene_monitor(
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("piper_with_gripper_moveit"),  # Update package
                "config",
                "ros2_controllers.yaml",  # Update controller config
            ),
        ],
        output="screen",
    )
        
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            {'start_state':{'content': initial_positions_file_path}},
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )
    
    devrob_move_node = Node(
        package="devrob_move_program",
        executable="devrob_move_program",
        name="devrob_move_program",
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
)

    rviz_config_path = os.path.join(
        get_package_share_directory("piper_with_gripper_moveit"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            robot_description_kinematic,
            moveit_config.trajectory_execution,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
            {'use_rviz': use_rviz}
        ],
    )
    return LaunchDescription(
        [
          DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
          DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'),

          move_group_node,
          rviz_node,
        #   devrob_move_node,
          ros2_control_node
        ]
    )