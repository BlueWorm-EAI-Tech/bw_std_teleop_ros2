#!/usr/bin/env python3

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class BringupContractTest(unittest.TestCase):

    def read_required(self, relative_path: str) -> str:
        path = PACKAGE_ROOT / relative_path
        self.assertTrue(path.is_file(), f"缺少发布文件: {relative_path}")
        return path.read_text(encoding="utf-8")

    def test_manifest_contains_only_standard_runtime_packages(self):
        root = ET.fromstring(self.read_required("package.xml"))
        self.assertEqual("bw_std_bringup", root.findtext("name"))

        dependencies = {
            element.text
            for element in root
            if element.tag in {"depend", "exec_depend"}
        }
        self.assertTrue(
            {
                "bw_kinematics",
                "bw_std_control",
                "bw_std_description",
                "bw_teleop",
            }.issubset(dependencies)
        )

    def test_controller_configuration_matches_standard_contract(self):
        config = yaml.safe_load(self.read_required("config/standard_controllers.yaml"))
        manager = config["controller_manager"]["ros__parameters"]

        self.assertNotIn("head_controller", manager)
        self.assertEqual(
            "bw_std_control/StandardBaseController",
            manager["base_controller"]["type"],
        )
        expected_arms = [
            *(f"A_left_Degree{index}_joint" for index in range(1, 8)),
            *(f"A_right_Degree{index}_joint" for index in range(1, 8)),
        ]
        self.assertEqual(
            expected_arms,
            config["dual_arm_controller"]["ros__parameters"]["joints"],
        )
        self.assertEqual(
            ["A_left_Degree8_joint", "A_right_Degree8_joint"],
            config["gripper_controller"]["ros__parameters"]["joints"],
        )
        self.assertEqual(
            ["C_joint"], config["lift_controller"]["ros__parameters"]["joints"]
        )

    def test_launch_uses_standard_packages_and_safe_defaults(self):
        launch_path = PACKAGE_ROOT / "launch" / "standard.launch.py"
        launch_source = self.read_required("launch/standard.launch.py")
        ast.parse(launch_source, filename=str(launch_path))

        self.assertIn('get_package_share_directory("bw_std_description")', launch_source)
        self.assertIn('get_package_share_directory("bw_std_bringup")', launch_source)
        self.assertIn(
            'DeclareLaunchArgument("start_teleop", default_value="false")',
            launch_source,
        )
        self.assertIn(
            'DeclareLaunchArgument("activate_motion_controllers", default_value="false")',
            launch_source,
        )
        self.assertIn(
            'DeclareLaunchArgument("power_on_on_activate", default_value="false")',
            launch_source,
        )
        self.assertIn(
            'DeclareLaunchArgument("arm_mapping_calibrated", default_value="false")',
            launch_source,
        )
        self.assertNotIn("pelvis_zero_mm", launch_source)
        self.assertNotIn("pelvis_reference_raw_mm", launch_source)
        self.assertNotIn("pelvis_reference_position_m", launch_source)
        self.assertIn('LaunchConfiguration("arm_mapping_calibrated")', launch_source)
        self.assertIn('LaunchConfiguration("left_arm_motor_indices")', launch_source)
        self.assertIn('LaunchConfiguration("right_arm_motor_indices")', launch_source)
        self.assertIn('LaunchConfiguration("left_arm_direction")', launch_source)
        self.assertIn('LaunchConfiguration("right_arm_direction")', launch_source)
        self.assertIn('LaunchConfiguration("left_arm_raw_zero_rad")', launch_source)
        self.assertIn('LaunchConfiguration("right_arm_raw_zero_rad")', launch_source)
        self.assertNotIn("left_elbow_raw_zero_rad", launch_source)
        self.assertNotIn("right_elbow_raw_zero_rad", launch_source)
        self.assertNotIn("static_transform_publisher", launch_source)
        self.assertNotIn("head_controller", launch_source)

        mock_source = self.read_required("launch/standard_mock.launch.py")
        self.assertIn('"start_teleop": "true"', mock_source)


if __name__ == "__main__":
    unittest.main()
