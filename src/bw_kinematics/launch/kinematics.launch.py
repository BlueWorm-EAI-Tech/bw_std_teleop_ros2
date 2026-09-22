from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("bw_kinematics"))
    default_parameters = str(package_share / "config" / "kinematics.yaml")

    ik_model_path = LaunchConfiguration("ik_model_path")
    parameters_file = LaunchConfiguration("parameters_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ik_model_path",
                default_value=str(
                    package_share / "assets" / "standard_ik" / "standard_ik.urdf"
                ),
                description="Standard 14 轴 Pinocchio/CasADi IK URDF",
            ),
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
                description="运动学节点参数文件",
            ),
            ComposableNodeContainer(
                name="kinematics_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="bw_kinematics",
                        plugin="bw_kinematics::KinematicsNode",
                        name="kinematics_node",
                        parameters=[
                            parameters_file,
                            {"ik_model_path": ik_model_path},
                        ],
                    )
                ],
            ),
        ]
    )
