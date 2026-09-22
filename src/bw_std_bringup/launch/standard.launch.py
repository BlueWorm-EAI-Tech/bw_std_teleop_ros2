"""启动 Standard 真机或 mock ros2_control 遥操作链路。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


MOTION_CONTROLLERS = (
    "dual_arm_controller",
    "gripper_controller",
    "lift_controller",
    "base_controller",
    "head_controller",
)


def _motion_spawners(context):
    activate = LaunchConfiguration("activate_motion_controllers").perform(context).lower()
    inactive_args = [] if activate in {"1", "true", "yes", "on"} else ["--inactive"]
    common = ["--controller-manager", "/controller_manager", *inactive_args]
    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name, *common],
            output="screen",
        )
        for controller_name in MOTION_CONTROLLERS
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("bw_std_bringup"))
    teleop_share = Path(get_package_share_directory("bw_teleop"))
    kinematics_share = Path(get_package_share_directory("bw_kinematics"))

    model_path = bringup_share / "urdf" / "standard.xacro"
    controllers_path = bringup_share / "config" / "standard_controllers.yaml"

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            str(model_path),
            " use_mock_hardware:=",
            use_mock_hardware,
            " serial_port:=",
            LaunchConfiguration("serial_port"),
            " baud_rate:=",
            LaunchConfiguration("baud_rate"),
            " feedback_timeout_ms:=",
            LaunchConfiguration("feedback_timeout_ms"),
            " power_on_on_activate:=",
            LaunchConfiguration("power_on_on_activate"),
            " arm_mapping_calibrated:=",
            LaunchConfiguration("arm_mapping_calibrated"),
            " left_arm_motor_indices:=",
            LaunchConfiguration("left_arm_motor_indices"),
            " right_arm_motor_indices:=",
            LaunchConfiguration("right_arm_motor_indices"),
            " left_arm_direction:=",
            LaunchConfiguration("left_arm_direction"),
            " right_arm_direction:=",
            LaunchConfiguration("right_arm_direction"),
            " left_arm_raw_zero_rad:=",
            LaunchConfiguration("left_arm_raw_zero_rad"),
            " right_arm_raw_zero_rad:=",
            LaunchConfiguration("right_arm_raw_zero_rad"),
            " arm_max_velocity_rad_s:=",
            LaunchConfiguration("arm_max_velocity_rad_s"),
            " pelvis_max_velocity_mm_s:=",
            LaunchConfiguration("pelvis_max_velocity_mm_s"),
            " gripper_max_velocity_normalized_s:=",
            LaunchConfiguration("gripper_max_velocity_normalized_s"),
            " arm_startup_limit_tolerance_rad:=",
            LaunchConfiguration("arm_startup_limit_tolerance_rad"),
            " gripper_startup_limit_tolerance_m:=",
            LaunchConfiguration("gripper_startup_limit_tolerance_m"),
            " head_max_velocity_rad_s:=",
            LaunchConfiguration("head_max_velocity_rad_s"),
            " command_rate_hz:=",
            LaunchConfiguration("command_rate_hz"),
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(teleop_share / "launch" / "bw_teleop.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_teleop")),
    )
    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(kinematics_share / "launch" / "kinematics.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_teleop")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("activate_motion_controllers", default_value="false"),
            DeclareLaunchArgument("start_teleop", default_value="false"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud_rate", default_value="2000000"),
            DeclareLaunchArgument("feedback_timeout_ms", default_value="100"),
            DeclareLaunchArgument("power_on_on_activate", default_value="false"),
            DeclareLaunchArgument("arm_mapping_calibrated", default_value="false"),
            DeclareLaunchArgument(
                "left_arm_motor_indices", default_value="0,1,2,3,4,5,6"
            ),
            DeclareLaunchArgument(
                "right_arm_motor_indices", default_value="0,1,2,3,4,5,6"
            ),
            DeclareLaunchArgument(
                "left_arm_direction", default_value="1,1,1,1,1,1,1"
            ),
            DeclareLaunchArgument(
                "right_arm_direction", default_value="1,1,1,1,1,1,1"
            ),
            DeclareLaunchArgument(
                "left_arm_raw_zero_rad", default_value="0,0,0,0,0,0,0"
            ),
            DeclareLaunchArgument(
                "right_arm_raw_zero_rad", default_value="0,0,0,0,0,0,0"
            ),
            DeclareLaunchArgument("arm_max_velocity_rad_s", default_value="12.0"),
            DeclareLaunchArgument("pelvis_max_velocity_mm_s", default_value="200.0"),
            DeclareLaunchArgument(
                "gripper_max_velocity_normalized_s", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "arm_startup_limit_tolerance_rad", default_value="0"
            ),
            DeclareLaunchArgument(
                "gripper_startup_limit_tolerance_m", default_value="0"
            ),
            DeclareLaunchArgument("head_max_velocity_rad_s", default_value="1.0"),
            DeclareLaunchArgument("command_rate_hz", default_value="200.0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[str(controllers_path)],
                remappings=[("~/robot_description", "/robot_description")],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            OpaqueFunction(function=_motion_spawners),
            teleop_launch,
            kinematics_launch,
        ]
    )
