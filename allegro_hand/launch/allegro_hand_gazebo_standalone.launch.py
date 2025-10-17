from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

def launch_setup(context):
    rviz_config_file = PathJoinSubstitution([FindPackageShare("allegro_hand_description"), "viz", "allegro_hand_config.rviz"])
    chirality = LaunchConfiguration("chirality").perform(context)

    ###### Robot description ######
    # Command to generate robot description from xacro file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # Find the xacro executable
            " ", 
            PathJoinSubstitution([FindPackageShare("allegro_hand"), "urdf", f"allegro_hand_on_stand.urdf.xacro"]),
            " ",
            "chirality:=",
            chirality,
            " ",
        ]
    )
    # Create dictionary to pass robot_description to the robot_state_publisher node parameters
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Define the robot_state_publisher node to publish the robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': True}],
    )

    ###### Rviz ######
    # Define the RViz node for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    ###### GAZEBO ######
    gazebo_launch = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py',
        ])
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf '}.items(),
    )

    # Spawn gripper in Gazebo
    spawn_gripper = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                    '-name', 'allegro_hand',
                    '-allow_renaming', 'true'],
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # Controller launch
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
            FindPackageShare('allegro_hand_controllers'),
            'launch',
            'allegro_hand_controller.launch.py',
            ])
        ]),
        launch_arguments={'prefix': "", 'controller_file': PathJoinSubstitution([FindPackageShare('allegro_hand_controllers'), 'config', 'default_pos_controller.yaml']), 'standalone': "true"}.items()
    )

    return [
        robot_state_publisher_node,
        rviz_node,
        gazebo_launch,
        spawn_gripper,
        gz_sim_bridge,
        controller_launch,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "chirality",
            default_value=["right"],
            description="Chirality of allegro hand (right/left)"
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
