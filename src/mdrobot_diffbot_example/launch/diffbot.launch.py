# Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
"""Bring up the diffbot example (ros2_control diff-drive on a MDROBOT dual controller).

  # mock hardware (no device) — visualize + drive in RViz:
  ros2 launch mdrobot_diffbot_example diffbot.launch.py

  # real PNT50 / MD400T:
  ros2 launch mdrobot_diffbot_example diffbot.launch.py use_mock_hardware:=false port:=/dev/ttyUSB0

Drive it with TwistStamped on /diff_cont/cmd_vel, or:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
      --ros-args -r /cmd_vel:=/diff_cont/cmd_vel -p stamped:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_mock = LaunchConfiguration("use_mock_hardware").perform(context)
    port = LaunchConfiguration("port")
    cpr = LaunchConfiguration("counts_per_rev")
    rev_l = LaunchConfiguration("reverse_left")
    rev_r = LaunchConfiguration("reverse_right")
    rate_arg = LaunchConfiguration("update_rate").perform(context)

    mock = use_mock.lower() in ("true", "1", "yes")
    update_rate = int(rate_arg) if rate_arg not in ("", "0") else (30 if mock else 15)

    pkg = FindPackageShare("mdrobot_diffbot_example")
    xacro_file = PathJoinSubstitution([pkg, "description", "diffbot.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "diffbot_controllers.yaml"])
    rviz_cfg = PathJoinSubstitution([pkg, "rviz", "diffbot.rviz"])

    # ParameterValue(value_type=str) forces the description to a string; without it
    # launch_ros YAML-auto-detects the type and aborts if the URDF XML is not a valid
    # YAML scalar (see bringup.launch.py).
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", xacro_file,
                " use_mock_hardware:=", use_mock,
                " port:=", port,
                " counts_per_rev:=", cpr,
                " reverse_left:=", rev_l,
                " reverse_right:=", rev_r,
            ]),
            value_type=str,
        )
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers, {"update_rate": update_rate}],
        output="screen",
    )
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    diff = Node(package="controller_manager", executable="spawner",
                arguments=["diff_cont", "-c", "/controller_manager"])
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_cfg], output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return [control_node, rsp_node, jsb, diff, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="true",
                              description="true: mock_components (no device); false: real MDROBOT dual"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0",
                              description="serial port (real hardware)"),
        DeclareLaunchArgument("counts_per_rev", default_value="12.0",
                              description="counts/rev per wheel motor (PNT50 measured: 12)"),
        DeclareLaunchArgument("reverse_left", default_value="false",
                              description="reverse left wheel (mirrored mount)"),
        DeclareLaunchArgument("reverse_right", default_value="false",
                              description="reverse right wheel (mirrored mount)"),
        DeclareLaunchArgument("update_rate", default_value="0",
                              description="controller_manager Hz; 0 = auto (mock 30 / real 15)"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="launch RViz"),
        OpaqueFunction(function=launch_setup),
    ])
