from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # robot_state_publisher node
    urdf_path = PathJoinSubstitution([FindPackageShare("cobot_description"), "urdf", "cobot.urdf.xacro"])

    xacro_script = Command(["xacro ", urdf_path])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(xacro_script, value_type=str)
        }]
    )

    # rviz2 node
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("cobot_bringup"), "config", "cobot.rviz"])

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config]
    )

    # controller_manager node
    controller_config = PathJoinSubstitution([
        FindPackageShare("cobot_bringup"), "config", "cobot_controller.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description")  # ~ = private namespace = controller_manager
        ],
    )

    # spawn joint_trajectory_controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # event handling
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[rviz2_node],
        )
    )

    nodes = [
        control_node,
        robot_state_publisher,
        joint_trajectory_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner
    ]

    return LaunchDescription(nodes)