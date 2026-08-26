from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("bw_teleop"))
    default_params = package_share / "config" / "bw_teleop.yaml"
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=str(default_params),
            description="bw_teleop 参数文件",
        ),
        Node(
            package="bw_teleop",
            executable="bw_teleop",
            name="bw_teleop",
            output="screen",
            parameters=[params_file],
        ),
    ])
