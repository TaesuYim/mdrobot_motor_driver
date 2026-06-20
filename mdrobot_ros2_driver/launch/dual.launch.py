"""PNT50 듀얼 채널 드라이버 노드 launch.

예:
  ros2 launch mdrobot_ros2_driver dual.launch.py port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="19200"),
        DeclareLaunchArgument("motor_id", default_value="1"),
        DeclareLaunchArgument("command_timeout", default_value="0.5"),
        DeclareLaunchArgument("use_limit_sw", default_value="-1",
                              description="-1 유지 / 0 비활성 / 1 사용 (motor1·motor2 동일 적용)"),
        DeclareLaunchArgument("namespace", default_value="pnt50"),
    ]
    node = Node(
        package="mdrobot_ros2_driver",
        executable="motor_driver_node",
        name="mdrobot_motor_driver",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "device_type": "dual",
            "port": LaunchConfiguration("port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "motor_id": LaunchConfiguration("motor_id"),
            "command_timeout": LaunchConfiguration("command_timeout"),
            "use_limit_sw": LaunchConfiguration("use_limit_sw"),
        }],
    )
    return LaunchDescription(args + [node])
