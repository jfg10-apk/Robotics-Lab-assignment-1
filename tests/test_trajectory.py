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
    JOINT_TRAJECTORIES,
    SHOULDER_JOINT,
    TRAJECTORY_DURATION,
    get_target_angle,
)


def test_viewer_backspace_requests_simulation_reset():
    import threading

    import glfw

    reset_requested = threading.Event()
    key_callback = _make_key_callback(reset_requested)

    key_callback(glfw.KEY_BACKSPACE)

    assert reset_requested.is_set()


def test_swing_targets_only_the_right_shoulder():
    env = HumanoidEnv()
    controller = SwingController(env)

    right_swing_ids = {
        controller.actuator_ids["shoulder1_right"],
    }
    times = [0.0, 0.5, 1.0, 1.5, 2.0]
    actions = [controller.get_action(t) for t in times]

    assert all(set(action) == right_swing_ids for action in actions)
    shoulder_id = controller.actuator_ids["shoulder1_right"]
    shoulder_targets = [action[shoulder_id] for action in actions]
    assert all(
        first < second
        for first, second in zip(shoulder_targets, shoulder_targets[1:])
    )


def test_environment_imports_xml_parameters_for_controller():
    env = HumanoidEnv()
    expected_actuator_ids = {
        env.model.actuator(actuator_id).name: actuator_id
        for actuator_id in range(env.model.nu)
    }
    expected_link_lengths = tuple(
        2.0 * env.model.geom_size[env.model.geom(name).id, 1]
        for name in ("upper_arm_right", "lower_arm_right")
    )

    assert env.actuator_ids == expected_actuator_ids
    np.testing.assert_allclose(env.link_lengths, expected_link_lengths)
    assert env.timestep == env.model.opt.timestep

    controller = SwingController(env)
    assert controller.actuator_ids is env.actuator_ids
    assert controller.joint_names == ("shoulder1_right",)


def test_swing_trajectory_clamps_and_does_not_repeat():
    env = HumanoidEnv()
    controller = SwingController(env)
    keyframes = JOINT_TRAJECTORIES[SHOULDER_JOINT]
    initial_angle = get_target_angle(-1.0, keyframes)
    final_angle = get_target_angle(TRAJECTORY_DURATION + 1.0, keyframes)

    assert initial_angle == get_target_angle(0.0, keyframes)
    np.testing.assert_allclose(
        final_angle,
        get_target_angle(TRAJECTORY_DURATION, keyframes),
    )
    assert controller.get_action(TRAJECTORY_DURATION + 10.0) == controller.get_action(
        TRAJECTORY_DURATION
    )


def test_trajectory_interpolates_between_keyframes():
    start, end = JOINT_TRAJECTORIES[SHOULDER_JOINT]
    midpoint = (start[0] + end[0]) / 2
    angle = np.deg2rad((start[1] + end[1]) / 2)

    assert get_target_angle(midpoint, JOINT_TRAJECTORIES[SHOULDER_JOINT]) == angle


def test_trajectory_rejects_invalid_keyframe_order_and_nonfinite_time():
    import pytest

    with pytest.raises(ValueError, match="strictly increasing"):
        get_target_angle(0.5, ((0.0, -55.0), (0.0, 0.0)))

    with pytest.raises(ValueError, match="finite"):
        get_target_angle(float("nan"), JOINT_TRAJECTORIES[SHOULDER_JOINT])


def test_circular_target_follows_actual_shoulder_axis_and_tip():
    env = HumanoidEnv()
    controller = SwingController(env)
    elbow_joint_id = env.model.joint("elbow_right").id
    initial_elbow_angle = env.data.qpos[env.model.jnt_qposadr[elbow_joint_id]]
    radius = np.linalg.norm(env.initial_club_tip - env.shoulder_pivot)
    initial_target = controller.get_target_point(0.0)
    final_target = controller.get_target_point(TRAJECTORY_DURATION)
    assert final_target[2] < initial_target[2]

    for time in np.linspace(0.0, TRAJECTORY_DURATION, 101):
        action = controller.get_action(time)
        shoulder = action[controller.actuator_ids["shoulder1_right"]]
        lower, upper = env.model.actuator_ctrlrange[
            controller.actuator_ids["shoulder1_right"]
        ]
        assert lower <= shoulder <= upper

        env.apply_kinematic_action(action)
        target_point = controller.get_target_point(time)
        np.testing.assert_allclose(
            env.data.site("ponta_taco").xpos,
            target_point,
            atol=1e-8,
        )
        np.testing.assert_allclose(
            np.linalg.norm(target_point - env.shoulder_pivot),
            radius,
            atol=1e-8,
        )
        assert (
            env.data.qpos[env.model.jnt_qposadr[elbow_joint_id]]
            == initial_elbow_angle
        )
        env.set_target_marker(target_point)
        np.testing.assert_allclose(
            env.data.mocap_pos[env.target_marker_mocap_id],
            target_point,
            atol=1e-8,
        )


def test_club_angle_and_grip_stay_fixed_to_forearm_during_swing():
    env = HumanoidEnv()

    controller = SwingController(env)
    measured_angles = []
    for time in (0.0, TRAJECTORY_DURATION / 2, TRAJECTORY_DURATION):
        env.apply_kinematic_action(controller.get_action(time))
        forearm_grip = env.data.site("forearm_grip")
        club_grip = env.data.site("grip_right")
        forearm_axis = forearm_grip.xmat.reshape(3, 3)[:, 2]
        club_axis = club_grip.xmat.reshape(3, 3)[:, 2]

        np.testing.assert_allclose(forearm_grip.xpos, club_grip.xpos, atol=1e-8)
        included_angle = np.rad2deg(
            np.arccos(np.clip(np.dot(-forearm_axis, club_axis), -1.0, 1.0))
        )
        measured_angles.append(included_angle)
    np.testing.assert_allclose(measured_angles, measured_angles[0], atol=0.1)


def test_kinematic_swing_changes_arm_without_moving_torso_or_legs():
    env = HumanoidEnv()
    controller = SwingController(env)
    fixed_body_names = ("torso", "pelvis", "thigh_right", "thigh_left")
    initial_positions = {
        name: env.data.xpos[env.model.body(name).id].copy()
        for name in fixed_body_names
    }
    initial_tip = env.data.site("ponta_taco").xpos.copy()

    env.apply_kinematic_action(controller.get_action(1.3))

    assert not np.allclose(env.data.site("ponta_taco").xpos, initial_tip)
    for name, position in initial_positions.items():
        np.testing.assert_allclose(env.data.xpos[env.model.body(name).id], position)