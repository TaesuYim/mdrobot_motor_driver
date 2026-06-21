"""Launch the dual-channel driver node.

All options live in config/dual.yaml — edit that file instead of passing them on
the command line:
  ros2 launch mdrobot_ros2_driver dual.launch.py
Use a different parameter file with:
  ros2 launch mdrobot_ros2_driver dual.launch.py config:=/path/to/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("mdrobot_ros2_driver"), "config", "dual.yaml"
    )
    args = [
        DeclareLaunchArgument(
            "config", default_value=default_config,
            description="Parameter YAML file. Edit it (or pass your own) instead of CLI options.",
        ),
        DeclareLaunchArgument("namespace", default_value=""),
    ]
    node = Node(
        package="mdrobot_ros2_driver",
        executable="motor_driver_node",
        name="mdrobot_motor_driver",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription(args + [node])
