


# obtenir le numero de serie de l'adaptateur : lsusb -v | grep -A 10 "PEAK-System"


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
    ExecuteProcess,
    LogInfo,

)
from launch.event_handlers import OnProcessExit
def launch_setup(context):
	# Récupérer les valeurs des arguments
    chirality = LaunchConfiguration("chirality").perform(context)
    can_device = LaunchConfiguration("can_device").perform(context)
    
    # --- Configuration du CAN (Méthode non-interactive) ---
    # PRÉREQUIS: Avoir configuré sudoers pour que `sudo ip` ne demande pas de mdp.
    # $ sudo visudo
    # VOTRE_USER ALL=(ALL) NOPASSWD: /sbin/ip
    
    can_setup_actions = [
        LogInfo(msg=["Configuring CAN interface: ", can_device]),
        ExecuteProcess(cmd=['sudo', 'ip', 'link', 'set', can_device,'up', 'type', 'can', 'bitrate', '1000000']),
    ]

    # --- Description du Robot (URDF) ---
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
        parameters=[robot_description, {'use_sim_time': False}],
    )

    # --- ros2_control ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([FindPackageShare("allegro_hand_controllers"), "config", "default_pos_controller.yaml"])
        ],
        output="screen",
    )
    
    rviz_config_file = PathJoinSubstitution([FindPackageShare("allegro_hand_description"), "viz", "allegro_hand_config.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
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
        launch_arguments=[
                ('prefix', ""),
                ("standalone","True"),
                ('controller_file', PathJoinSubstitution([FindPackageShare('allegro_hand_controllers'), 'config', 'default_pos_controller.yaml']))
            ],
    )
	
    nodes_to_launch = [
        ros2_control_node,
        robot_state_publisher_node,
        rviz_node,
        controller_launch,
    ]
    
    return can_setup_actions + nodes_to_launch


def generate_launch_description():

    # --- Arguments de Lancement ---
    declared_arguments = [
        DeclareLaunchArgument(
            "chirality",
            default_value="right",
            description="Chirality of allegro hand (right/left)"
        ),
        DeclareLaunchArgument(
            "can_device",
            default_value="can0",
            description="CAN interface name for the real robot (e.g., can0)"
        )
    ]
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
