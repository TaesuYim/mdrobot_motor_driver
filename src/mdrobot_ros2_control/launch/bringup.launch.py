# Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
"""Bring up a MDROBOT controller through ros2_control.

  ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=single port:=/dev/ttyUSB0
  ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=dual   port:=/dev/ttyUSB0
  ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=twin   port:=/dev/ttyUSB0 \
      motor_id_1:=1 motor_id_2:=2 reverse_2:=true

Starts robot_state_publisher, the controller_manager (ros2_control_node), and
spawns joint_state_broadcaster (+ diff_cont for dual/twin, velocity_cont for single).

twin = two single-channel controllers on one bus at distinct slave IDs (skid-steer);
re-ID one unit (PID_ID) so the two motor_id values differ before bringup.
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
    if device_type not in ("single", "dual", "twin"):
        raise RuntimeError(
            f"device_type must be 'single', 'dual', or 'twin', got {device_type!r}")
    port = LaunchConfiguration("port")

    pkg = FindPackageShare("mdrobot_ros2_control")
    urdf = PathJoinSubstitution([pkg, "urdf", f"mdrobot_{device_type}.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", f"{device_type}_controllers.yaml"])

    xacro_cmd = [FindExecutable(name="xacro"), " ", urdf, " port:=", port]
    if device_type == "twin":
        # Per-wheel twin args. Only forward those the user actually set so the
        # xacro defaults (motor_id 1/2, reverse false, counts_per_rev 24) apply
        # otherwise. xacro ignores undeclared/extra args, so this is safe.
        for name in ("motor_id_1", "motor_id_2", "reverse_1", "reverse_2",
                     "counts_per_rev_1", "counts_per_rev_2"):
            if LaunchConfiguration(name).perform(context) != "":
                xacro_cmd += [f" {name}:=", LaunchConfiguration(name)]
    else:
        # Only override counts_per_rev when set (non-zero); else keep URDF default.
        cpr = LaunchConfiguration("counts_per_rev").perform(context)
        if cpr not in ("", "0.0", "0"):
            xacro_cmd += [" counts_per_rev:=", LaunchConfiguration("counts_per_rev")]
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

    extra = "diff_cont" if device_type in ("dual", "twin") else "velocity_cont"
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
                description="single (MD400), dual (PNT50/MD400T), or twin (two single controllers on one bus)",
            ),
            DeclareLaunchArgument(
                "port", default_value="/dev/ttyUSB0",
                description="serial port (default /dev/ttyUSB0)",
            ),
            DeclareLaunchArgument(
                "counts_per_rev", default_value="0.0",
                description="single/dual: counts per rev for SI joint_states; 0 keeps the URDF default",
            ),
            # --- twin-only (ignored unless device_type:=twin); empty keeps URDF default ---
            DeclareLaunchArgument(
                "motor_id_1", default_value="",
                description="twin: left controller Modbus slave id (URDF default 1)",
            ),
            DeclareLaunchArgument(
                "motor_id_2", default_value="",
                description="twin: right controller Modbus slave id (URDF default 2; must differ from motor_id_1)",
            ),
            DeclareLaunchArgument(
                "reverse_1", default_value="",
                description="twin: invert left wheel direction (true/false; URDF default false)",
            ),
            DeclareLaunchArgument(
                "reverse_2", default_value="",
                description="twin: invert right wheel direction (true/false; URDF default false)",
            ),
            DeclareLaunchArgument(
                "counts_per_rev_1", default_value="",
                description="twin: left wheel counts per rev (URDF default 24)",
            ),
            DeclareLaunchArgument(
                "counts_per_rev_2", default_value="",
                description="twin: right wheel counts per rev (URDF default 24)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
