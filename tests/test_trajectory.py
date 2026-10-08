import sys
from pathlib import Path

import numpy as np

# Make the project's flat src modules importable when pytest runs from the root.
SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

from controllers import SwingController
from env import HumanoidEnv
from main import _make_key_callback
from trajectory import (
    ANGULAR_SPEED_DEGREES_PER_SECOND,
    DURATION_SECONDS,
    END_ANGLE_DEGREES,
    JOINT_TRAJECTORIES,
    SHOULDER_JOINT,
    START_ANGLE_DEGREES,
    get_joint_targets,
    trajectory_1,
)


def test_viewer_backspace_requests_simulation_reset():
    import threading

    import glfw

    reset_requested = threading.Event()
    key_callback = _make_key_callback(reset_requested)

    key_callback(glfw.KEY_BACKSPACE)

    assert reset_requested.is_set()


def test_algebraic_shoulder_trajectory_spans_180_degrees_in_six_seconds():
    assert ANGULAR_SPEED_DEGREES_PER_SECOND == 30.0
    assert DURATION_SECONDS == 6.0
    assert trajectory_1(0.0) == START_ANGLE_DEGREES == -90.0
    assert trajectory_1(1.0) == -60.0
    assert trajectory_1(3.0) == 0.0
    assert trajectory_1(DURATION_SECONDS) == END_ANGLE_DEGREES == 90.0
    assert trajectory_1(DURATION_SECONDS + 2.0) == END_ANGLE_DEGREES


def test_trajectory_returns_configured_joint_targets_in_radians():
    targets = get_joint_targets(0.0)

    assert set(targets) == {SHOULDER_JOINT}
    np.testing.assert_allclose(targets[SHOULDER_JOINT], np.deg2rad(-90.0))


def test_adding_joint_equation_preserves_existing_shoulder_target(monkeypatch):
    shoulder_target_before = get_joint_targets(2.0)[SHOULDER_JOINT]
    monkeypatch.setitem(
        JOINT_TRAJECTORIES,
        "elbow_right",
        lambda time_seconds: 45.0,
    )

    targets = get_joint_targets(2.0)

    assert set(targets) == {SHOULDER_JOINT, "elbow_right"}
    np.testing.assert_allclose(targets[SHOULDER_JOINT], shoulder_target_before)
    np.testing.assert_allclose(targets["elbow_right"], np.deg2rad(45.0))


def test_controller_maps_trajectory_joint_to_its_actuator():
    controller = SwingController(HumanoidEnv())

    action = controller.get_action(0.0)

    assert controller.joint_names == (SHOULDER_JOINT,)
    assert set(action) == {controller.actuator_ids[SHOULDER_JOINT]}
    np.testing.assert_allclose(
        action[controller.actuator_ids[SHOULDER_JOINT]],
        np.deg2rad(-90.0),
    )


def test_world_rotation_profile_does_not_actuate_other_shoulder_axes():
    env = HumanoidEnv()
    controller = SwingController(env)

    assert "shoulder2_right" not in controller.joint_names
    assert "shoulder3_right" not in controller.joint_names
    assert "elbow_right" not in controller.joint_names


def test_target_site_prediction_matches_kinematic_pose_without_mutating_state():
    env = HumanoidEnv()
    controller = SwingController(env)
    action = controller.get_action(0.0)
    expected_target = env.get_target_site_position(action)

    env.apply_kinematic_action(action)

    np.testing.assert_allclose(env.data.site("ponta_taco").xpos, expected_target)


def test_trajectory_rejects_non_finite_time():
    with np.testing.assert_raises(ValueError):
        trajectory_1(float("nan"))
