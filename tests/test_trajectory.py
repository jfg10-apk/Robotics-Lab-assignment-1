import sys
from pathlib import Path

import numpy as np

# Make the project's flat src modules importable when pytest runs from the root.
SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

from controllers import SwingController
from configuration import load_trial, select_case
from env import HumanoidEnv
from main import _make_key_callback
from trajectory import get_joint_targets, get_joint_velocity_targets

TRIAL_PATH = Path(__file__).resolve().parents[1] / "trials" / "trial_1.toml"


def test_viewer_backspace_requests_simulation_reset():
    import threading

    import glfw

    reset_requested = threading.Event()
    key_callback = _make_key_callback(reset_requested)

    key_callback(glfw.KEY_BACKSPACE)

    assert reset_requested.is_set()


def test_toml_trajectory_returns_keyframes_in_radians():
    trial = load_trial(TRIAL_PATH)
    trajectory = trial["trajectory"]
    targets = get_joint_targets(4.0, trajectory)

    assert set(targets) == set(trajectory["joints"])
    np.testing.assert_allclose(targets["shoulder1_right"], np.deg2rad(-90.0))
    np.testing.assert_allclose(targets["abdomen_z"], np.deg2rad(55.0))


def test_trajectory_velocity_matches_linear_keyframe_segment():
    trajectory = load_trial(TRIAL_PATH)["trajectory"]
    velocities = get_joint_velocity_targets(2.0, trajectory)
    times = trajectory["phase_times_s"]
    values = trajectory["joints"]["shoulder1_right"]["keyframes_deg"]

    np.testing.assert_allclose(
        velocities["shoulder1_right"],
        np.deg2rad((values[1] - values[0]) / (times[1] - times[0])),
    )


def test_controller_separates_position_targets_from_torque_commands():
    trial = load_trial(TRIAL_PATH)
    case = select_case(trial, "baseline")
    env = HumanoidEnv()
    controller = SwingController(env, trial, case)

    shoulder_id = controller.actuator_ids["shoulder1_right"]
    assert len(controller.joint_names) == 6
    np.testing.assert_allclose(
        controller.get_position_targets(4.0)[shoulder_id], np.deg2rad(-90.0)
    )
    assert abs(controller.get_torque_action(4.0)[shoulder_id]) <= 50.0


def test_target_site_prediction_matches_kinematic_pose_without_mutating_state():
    env = HumanoidEnv()
    controller = SwingController(env)
    position_targets = controller.get_position_targets(0.0)
    expected_target = env.get_target_site_position(position_targets)

    env.apply_kinematic_action(position_targets)

    np.testing.assert_allclose(env.data.site("ponta_taco").xpos, expected_target)


def test_trajectory_rejects_non_finite_time():
    controller = SwingController(HumanoidEnv())
    with np.testing.assert_raises(ValueError):
        controller.get_position_targets(float("nan"))
