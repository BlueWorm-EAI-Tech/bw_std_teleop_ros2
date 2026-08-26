#!/usr/bin/env python3
"""验证 Standard 机器人描述与 ros2_control 的发布契约。"""

from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_FILE = PACKAGE_ROOT / "urdf" / "standard.xacro"

ARM_JOINTS = {
    f"A_{side}_Degree{degree}_joint"
    for side in ("left", "right")
    for degree in range(1, 8)
}
ACTIVE_GRIPPERS = {
    "A_left_Degree8_joint",
    "A_right_Degree8_joint",
}
CONTROLLED_JOINTS = ARM_JOINTS | ACTIVE_GRIPPERS | {"C_joint"}
PASSIVE_JOINTS = {
    "A_left_Degree9_joint",
    "A_right_Degree9_joint",
    "D_left_joint",
    "D_right_joint",
    "D_behind_joint",
    "H_Degree1_joint",
    "H_Degree2_joint",
    "H_Degree3_joint",
}


def expand_model(
    use_mock_hardware: bool, overrides: dict[str, str] | None = None
) -> ET.Element:
    """展开模型并返回 XML 根节点。"""
    arguments = [
        "xacro",
        str(XACRO_FILE),
        f"use_mock_hardware:={'true' if use_mock_hardware else 'false'}",
    ]
    arguments.extend(
        f"{name}:={value}" for name, value in (overrides or {}).items()
    )
    result = subprocess.run(
        arguments,
        check=True,
        capture_output=True,
        text=True,
        timeout=20,
    )
    return ET.fromstring(result.stdout)


class DescriptionContractTest(unittest.TestCase):
    """检查发布包身份和运行时描述契约。"""

    def test_package_identity_is_consistent(self) -> None:
        manifest = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
        self.assertEqual(manifest.findtext("name"), "bw_std_description")

        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("project(bw_std_description)", cmake)

        for launch_file in (PACKAGE_ROOT / "launch").glob("*.launch.py"):
            content = launch_file.read_text(encoding="utf-8")
            self.assertIn('get_package_share_directory("bw_std_description")', content)

    def test_package_does_not_install_legacy_controller_joint_list(self) -> None:
        legacy_config = PACKAGE_ROOT / "config" / "joint_names_standard.yaml"
        self.assertFalse(
            legacy_config.exists(),
            "未使用的 controller_joint_names 会误导工具绕过 ros2_control 契约",
        )

        self.assertFalse((PACKAGE_ROOT / "urdf" / "standard.csv").exists())
        self.assertFalse((PACKAGE_ROOT / "export.log").exists())

    def test_mock_model_exports_only_approved_interfaces(self) -> None:
        root = expand_model(use_mock_hardware=True)
        control = root.find("ros2_control")
        self.assertIsNotNone(control)
        self.assertEqual(
            control.findtext("hardware/plugin"),
            "mock_components/GenericSystem",
        )

        control_joints = {joint.get("name"): joint for joint in control.findall("joint")}
        self.assertEqual(set(control_joints), CONTROLLED_JOINTS)
        self.assertTrue(PASSIVE_JOINTS.isdisjoint(control_joints))

        for name, joint in control_joints.items():
            self.assertEqual(
                [item.get("name") for item in joint.findall("command_interface")],
                ["position"],
                name,
            )
            self.assertEqual(
                [item.get("name") for item in joint.findall("state_interface")],
                ["position", "velocity", "effort"],
                name,
            )

        base = control.find("gpio")
        self.assertIsNotNone(base)
        self.assertEqual(base.get("name"), "base")
        self.assertEqual(
            [item.get("name") for item in base.findall("command_interface")],
            ["vx", "vy", "wz"],
        )
        self.assertEqual(base.findall("state_interface"), [])

    def test_mock_control_joints_are_movable_urdf_joints(self) -> None:
        root = expand_model(use_mock_hardware=True)
        model_joints = {joint.get("name"): joint for joint in root.findall("joint")}

        for control_joint in root.findall("ros2_control/joint"):
            name = control_joint.get("name")
            self.assertIn(name, model_joints, name)
            self.assertNotEqual("fixed", model_joints[name].get("type"), name)

    def test_real_model_uses_standard_hardware_and_safe_defaults(self) -> None:
        root = expand_model(use_mock_hardware=False)
        hardware = root.find("ros2_control/hardware")
        self.assertIsNotNone(hardware)
        self.assertEqual(
            hardware.findtext("plugin"),
            "bw_std_control/StandardSystemHardware",
        )
        self.assertEqual(
            [item.get("name") for item in root.findall("ros2_control/gpio/command_interface")],
            ["vx", "vy", "wz"],
        )
        control_joint_names = {
            joint.get("name") for joint in root.findall("ros2_control/joint")
        }
        self.assertNotIn("base", control_joint_names)
        parameters = {
            parameter.get("name"): parameter.text
            for parameter in hardware.findall("param")
        }
        self.assertEqual(parameters["power_on_on_activate"], "false")
        self.assertEqual(parameters["arm_mapping_calibrated"], "false")
        self.assertNotIn("pelvis_zero_mm", parameters)
        self.assertNotIn("pelvis_reference_raw_mm", parameters)
        self.assertNotIn("pelvis_reference_position_m", parameters)
        self.assertEqual(parameters["left_arm_motor_indices"], "0,1,2,3,4,5,6")
        self.assertEqual(parameters["right_arm_motor_indices"], "0,1,2,3,4,5,6")
        self.assertEqual(parameters["left_arm_direction"], "1,1,1,1,1,1,1")
        self.assertEqual(parameters["right_arm_direction"], "1,1,1,1,1,1,1")
        self.assertEqual(parameters["left_arm_raw_zero_rad"], "0,0,0,0,0,0,0")
        self.assertEqual(parameters["right_arm_raw_zero_rad"], "0,0,0,0,0,0,0")

        model_source = XACRO_FILE.read_text(encoding="utf-8")
        self.assertIn('<xacro:include filename="standard.urdf"/>', model_source)

    def test_real_model_forwards_commissioning_overrides(self) -> None:
        overrides = {
            "arm_mapping_calibrated": "true",
            "left_arm_motor_indices": "6,5,4,3,2,1,0",
            "right_arm_motor_indices": "1,0,3,2,5,4,6",
            "left_arm_direction": "-1,1,-1,1,-1,1,-1",
            "right_arm_direction": "1,-1,1,-1,1,-1,1",
            "left_arm_raw_zero_rad": "0.1,0.2,0.3,0.4,0.5,0.6,0.7",
            "right_arm_raw_zero_rad": "-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7",
            "arm_max_velocity_rad_s": "0.07",
            "pelvis_max_velocity_mm_s": "15.0",
            "gripper_max_velocity_normalized_s": "0.1",
        }
        root = expand_model(use_mock_hardware=False, overrides=overrides)
        hardware = root.find("ros2_control/hardware")
        parameters = {
            parameter.get("name"): parameter.text
            for parameter in hardware.findall("param")
        }
        for name, value in overrides.items():
            self.assertEqual(parameters[name], value)

    def test_model_preserves_passive_mechanics_and_valid_limits(self) -> None:
        root = expand_model(use_mock_hardware=True)
        joints = {joint.get("name"): joint for joint in root.findall("joint")}

        for wheel in ("D_left_joint", "D_right_joint", "D_behind_joint"):
            self.assertEqual(joints[wheel].get("type"), "fixed")

        for side in ("left", "right"):
            follower = joints[f"A_{side}_Degree9_joint"].find("mimic")
            self.assertIsNotNone(follower)
            self.assertEqual(follower.get("joint"), f"A_{side}_Degree8_joint")
            self.assertEqual(follower.get("multiplier"), "1.0")

        lift_limit = joints["C_joint"].find("limit")
        self.assertEqual(lift_limit.get("lower"), "-0.231")
        self.assertEqual(lift_limit.get("upper"), "0.231")

        for name, joint in joints.items():
            if joint.get("type") == "fixed":
                continue
            limit = joint.find("limit")
            self.assertIsNotNone(limit, name)
            self.assertGreater(float(limit.get("effort", "0")), 0.0, name)
            self.assertGreater(float(limit.get("velocity", "0")), 0.0, name)

        for mesh in root.findall(".//mesh"):
            self.assertTrue(
                mesh.get("filename").startswith("package://bw_std_description/"),
                mesh.get("filename"),
            )


if __name__ == "__main__":
    unittest.main()
