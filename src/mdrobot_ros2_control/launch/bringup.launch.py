# Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
"""Bring up a MDROBOT controller through ros2_control.

  ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=single port:=/dev/ttyUSB0
  ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=dual   port:=/dev/ttyUSB0

Starts robot_state_publisher, the controller_manager (ros2_control_node), and
spawns joint_state_broadcaster (+ diff_cont for dual).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    device_type = LaunchConfiguration("device_type").perform(context)
    if device_type not in ("single", "dual"):
        raise RuntimeError(f"device_type must be 'single' or 'dual', got {device_type!r}")
    port = LaunchConfiguration("port")
    cpr = LaunchConfiguration("counts_per_rev").perform(context)

    pkg = FindPackageShare("mdrobot_ros2_control")
    urdf = PathJoinSubstitution([pkg, "urdf", f"mdrobot_{device_type}.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", f"{device_type}_controllers.yaml"])

    # Only override counts_per_rev when set (non-zero); otherwise keep the URDF default.
    xacro_cmd = [FindExecutable(name="xacro"), " ", urdf, " port:=", port]
    if cpr not in ("", "0.0", "0"):
        xacro_cmd += [" counts_per_rev:=", cpr]
    robot_description = {"robot_description": Command(xacro_cmd)}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers],
        output="screen",
    )
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    extra = "diff_cont" if device_type == "dual" else "velocity_cont"
    nodes = [control_node, rsp_node, jsb_spawner]
    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[extra, "--controller-manager", "/controller_manager"],
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_type", default_value="single",
                description="single (MD400) or dual (PNT50/MD400T)",
            ),
            DeclareLaunchArgument(
                "port", default_value="/dev/ttyUSB0",
                description="serial port (default /dev/ttyUSB0)",
            ),
            DeclareLaunchArgument(
                "counts_per_rev", default_value="0.0",
                description="counts per rev for SI joint_states; 0 keeps the URDF default",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
