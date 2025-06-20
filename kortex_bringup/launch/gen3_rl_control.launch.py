from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_name = LaunchConfiguration("robot_name")
    gripper = LaunchConfiguration("gripper")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    robot_traj_controller = LaunchConfiguration("robot_controller")
    robot_pos_controller = LaunchConfiguration("robot_pos_controller")
    robot_hand_controller = LaunchConfiguration("robot_hand_controller")
    fault_controller = LaunchConfiguration("fault_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm") # This also helps us to identify 
                                                                #whether we are dealing with real-life hardware or simulation
    gripper_joint_name = LaunchConfiguration("gripper_joint_name")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "robots", description_file]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=",
            robot_name,
            " ",
            "gripper:=",
            gripper,
            " ",
            "use_internal_bus_gripper_comm:=",
            use_internal_bus_gripper_comm,
            " ",
            "gripper_max_velocity:=",
            gripper_max_velocity,
            " ",
            "gripper_max_force:=",
            gripper_max_force,
            " ",
            "gripper_joint_name:=",
            gripper_joint_name,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "gen3_arm/config",
            controllers_file,
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_traj_controller, "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_pos_controller, "--inactive", "-c", "/controller_manager"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_hand_controller, "-c", "/controller_manager"],
    )

    # only start the fault controller if we are using hardware
    fault_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[fault_controller, "-c", "/controller_manager"],
        condition=IfCondition(use_internal_bus_gripper_comm),
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        fault_controller_spawner,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    # Robot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "username", description="Robot session username.", default_value="admin"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "password", description="Robot session password.", default_value="admin"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port", description="Robot port for tcp connection.", default_value="10000"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port_realtime",
            description="Robot port for udp realtime control.",
            default_value="10001",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "session_inactivity_timeout_ms",
            description="Robot session inactivity timeout in milliseconds.",
            default_value="60000",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "connection_inactivity_timeout_ms",
            description="Robot connection inactivity timeout in milliseconds.",
            default_value="2000",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="kortex_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="gen3.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="arm",
            description="Name of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="robotiq_2f_85",
            description="Name of the gripper attached to the arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_pos_controller",
            default_value="twist_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_hand_controller",
            default_value="robotiq_gripper_controller",
            description="Robot hand controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fault_controller",
            default_value="fault_controller",
            description="Name of the 'fault controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use internal bus for gripper communication?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Max velocity for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_joint_name",
            default_value="robotiq_85_left_knuckle_joint",
            description="Max force for gripper commands",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
