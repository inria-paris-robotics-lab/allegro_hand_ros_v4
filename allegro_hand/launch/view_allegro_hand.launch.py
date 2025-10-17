from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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
    # Define the joint_state_publisher_gui node
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[robot_description],
    )
    # Define the robot_state_publisher node to publish the robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
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

    return [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
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
