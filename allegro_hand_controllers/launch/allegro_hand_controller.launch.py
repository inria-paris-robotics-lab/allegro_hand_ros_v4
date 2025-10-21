from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def launch_setup(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    controller_file = LaunchConfiguration('controller_file').perform(context)
    standalone = LaunchConfiguration('standalone').perform(context)

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster"],
        condition=IfCondition(standalone),
    )

    allegro_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[prefix + "allegro_hand_controller", '--param-file', controller_file],
    )

    return [joint_state_broadcaster, allegro_hand_controller_spawner]

def generate_launch_description():
    default_controller_file = PathJoinSubstitution([
        FindPackageShare('allegro_hand_controllers'),
        'config',
        'allegro_hand_controllers.yaml'
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Gripper prefix"
        ),
        DeclareLaunchArgument(
            "controller_file",
            default_value=default_controller_file,
            description="Path to the allegro hand controller yaml file"
        ),
        DeclareLaunchArgument(
            "standalone",
            default_value="false",
            description="If true, launch gripper standalone; else robot setup."
        )
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
