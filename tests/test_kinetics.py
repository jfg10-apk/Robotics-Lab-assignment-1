import sys
from pathlib import Path

import numpy as np

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

from env import HumanoidEnv
from kinematics import dir_kinematics, inv_kinematics


def test_forward_kinematics_rest_position_uses_model_link_lengths():
    env = HumanoidEnv()
    l1, l2 = env.link_lengths

    np.testing.assert_allclose(
        dir_kinematics(0.0, 0.0, l1, l2),
        (0.0, -(l1 + l2)),
    )


def test_inverse_kinematics_reaches_a_reachable_planar_target():
    env = HumanoidEnv()
    l1, l2 = env.link_lengths
    target_x, target_z = 0.15, -0.2

    shoulder_angle, elbow_angle = inv_kinematics(
        target_x, target_z, l1, l2
    )
    actual_x, actual_z = dir_kinematics(
        shoulder_angle, elbow_angle, l1, l2
    )

    np.testing.assert_allclose(
        (actual_x, actual_z),
        (target_x, target_z),
        atol=1e-5,
    )


def test_inverse_kinematics_projects_targets_inside_the_unreachable_inner_radius():
    l1, l2 = 2.0, 1.0

    shoulder_angle, elbow_angle = inv_kinematics(0.0, 0.0, l1, l2)
    actual_x, actual_z = dir_kinematics(shoulder_angle, elbow_angle, l1, l2)

    np.testing.assert_allclose(np.hypot(actual_x, actual_z), abs(l1 - l2))
    assert actual_z < 0.0


def test_inverse_kinematics_projects_targets_beyond_maximum_reach():
    l1, l2 = 2.0, 1.0

    shoulder_angle, elbow_angle = inv_kinematics(0.0, -10.0, l1, l2)
    actual_x, actual_z = dir_kinematics(shoulder_angle, elbow_angle, l1, l2)

    np.testing.assert_allclose(np.hypot(actual_x, actual_z), l1 + l2)
    np.testing.assert_allclose((actual_x, actual_z), (0.0, -(l1 + l2)), atol=1e-8)
