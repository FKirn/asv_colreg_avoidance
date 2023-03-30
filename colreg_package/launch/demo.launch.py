from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()
    colreg_description = get_package_share_directory('colreg_package')

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        remappings=[
            ('/turtle1/pose', 'own_ship/pose'),
            ('/turtle1/cmd_vel', 'own_ship/cmd_vel'),
        ],
    )
    initial_positioning_node = Node(
        package="colreg_package",
        executable="position",
        name="turtle_initial_positioning",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("colreg_package"), 'parameters', 'crossing_port.yaml')]
    )

    target_ship_control_node = Node(
        package="colreg_package",
        executable="target_ship_control",
        name="target_ship_control"
    )

    own_ship_control_node = Node(
        package="colreg_package",
        executable="own_ship_control",
        name="own_ship_control"
    )

    collision_scenario_calculation_node = Node(
        package="colreg_package",
        executable="collision_scenario_calculation",
        name="collision_scenario_calculation",
        output="screen"
    )

    ld.add_action(turtlesim_node)
    ld.add_action(initial_positioning_node)
    ld.add_action(target_ship_control_node)
    ld.add_action(own_ship_control_node)
    ld.add_action(collision_scenario_calculation_node)
    return ld
