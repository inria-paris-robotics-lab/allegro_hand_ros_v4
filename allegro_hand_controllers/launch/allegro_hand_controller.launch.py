from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def launch_setup(context):
    # Arguments
    prefix = LaunchConfiguration('prefix').perform(context)
    controller_file=LaunchConfiguration('controller_file').perform(context)
    standalone=LaunchConfiguration('standalone').perform(context)
    # Controller spawner
    joint_state_broadcatser = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster"],
        condition=IfCondition(standalone),
    )
    width_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[prefix+"allegro_hand_controller", '--param-file', controller_file],
    )
    return [joint_state_broadcatser, width_controller_spawner]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Gripper prefix"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_file",
            default_value="",
            description="Path to the WSG50 controller file"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "standalone",
            default_value="false",
            description="If true, the gripper will be launched in standalone mode. If false, the gripper will be launched in a robot setup."
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


