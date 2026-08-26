#!/usr/bin/env python3

from pathlib import Path
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class StandardDefaultsTest(unittest.TestCase):

    def test_kinematics_defaults_use_standard_arm_chains(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "kinematics.yaml").read_text(encoding="utf-8")
        )["kinematics_node"]["ros__parameters"]

        self.assertEqual("C_Link", config["root_link"])
        self.assertEqual("A_left_Degree7_link", config["left_tip_link"])
        self.assertEqual("A_right_Degree7_link", config["right_tip_link"])
        self.assertEqual("/current_left_pose", config["left_current_pose_topic"])
        self.assertEqual("/current_right_pose", config["right_current_pose_topic"])
        self.assertEqual(200, config["joint_state_timeout_ms"])

    def test_package_metadata_describes_standard_robot(self):
        manifest = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
        launch = (PACKAGE_ROOT / "launch" / "kinematics.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("Standard 双臂 KDL 运动学", manifest)
        self.assertIn("Standard URDF XML", launch)


if __name__ == "__main__":
    unittest.main()
