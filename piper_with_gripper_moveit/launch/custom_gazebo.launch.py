from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to your Xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare("piper_description"),
        "urdf",
        "piper_description.xacro"
    ])

    # Convert Xacro to URDF
    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", xacro_file
        ]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description},{"use_sim_time": True}],
    )

    # Classic Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("gazebo_ros"),
            "launch",
            "gazebo.launch.py"
        ]),
    )
    
    # Ignition Gazebo Launch
    
    # ign_gazebo = IncludeLaunchDescription(
    #     PathJoinSubstitution([
    #         FindPackageShare("ros_gz_sim"),
    #         "launch",
    #         "gz_sim.launch.py"
    #     ]),
    #     launch_arguments={
    #         "gz_args": "-r empty.sdf"    # start empty world
    #     }.items(),
    # )
    
    # Classic Gazebo Spawn Entity Node
    
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "piper"],
        output="screen",
    )
    
    # Ignition Gazebo Spawn Entity Node
    
    # spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     arguments=[
    #         "-name", "assembly_dualarm",
    #         "-topic", "robot_description",
    #     ],
    #     output="screen"
    # )    
    joint_state_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broad"],
        output="screen",
    )    
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],  # Update controller name
        output="screen",    
    )

    end_effector_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],  # Update controller name
        output="screen",    
    )
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        # ign_gazebo,
        spawn_entity,
        joint_state_broad,
        arm_controller_spawner,
        end_effector_controller_spawner
    ])