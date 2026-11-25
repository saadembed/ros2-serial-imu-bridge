from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments are strings. ParameterValue(value_type=...) pins the type the
    # node declared, instead of relying on YAML-style guessing of the string
    # (e.g. a port name that happens to look like a number).
    port = ParameterValue(LaunchConfiguration("port"), value_type=str)
    baud = ParameterValue(LaunchConfiguration("baud"), value_type=int)
    alpha = ParameterValue(LaunchConfiguration("alpha"), value_type=float)
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="", description="Serial port, empty = simulated source"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument(
                "with_filter", default_value="false", description="Also start the complementary filter node"
            ),
            DeclareLaunchArgument("alpha", default_value="0.98", description="Complementary filter gyro weight"),
            Node(
                package="serial_imu_bridge",
                executable="bridge_node",
                name="imu_bridge",
                parameters=[{"port": port, "baud": baud}],
                output="screen",
            ),
            Node(
                package="serial_imu_bridge",
                executable="imu_filter_node",
                name="imu_filter",
                parameters=[{"alpha": alpha}],
                output="screen",
                condition=IfCondition(LaunchConfiguration("with_filter")),
            ),
        ]
    )
