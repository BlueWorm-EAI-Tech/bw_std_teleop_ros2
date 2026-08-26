"""以 ros2_control mock hardware 启动 Standard。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    main_launch = (
        Path(get_package_share_directory("bw_std_bringup"))
        / "launch"
        / "standard.launch.py"
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(main_launch)),
                launch_arguments={
                    "use_mock_hardware": "true",
                    "activate_motion_controllers": "true",
                    "power_on_on_activate": "false",
                    "start_teleop": "true",
                }.items(),
            )
        ]
    )
