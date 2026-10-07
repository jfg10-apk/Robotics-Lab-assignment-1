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
    ELBOW_JOINT,
    SHOULDER_JOINT,
    SHOULDER_Y_JOINT,
    SHOULDER_Z_JOINT,
    THETA1_A_END,
    THETA1_A_START,
    THETA1_B_END,
    THETA1_C_END,
    THETA2_A,
    THETA2_C,
    get_path1_angles,
    get_t1_pA_angles,
    get_t1_pB_angles,
    get_t1_pC_angles,
)


def test_viewer_backspace_requests_simulation_reset():
    import threading

    import glfw

    reset_requested = threading.Event()
    key_callback = _make_key_callback(reset_requested)

    key_callback(glfw.KEY_BACKSPACE)

    assert reset_requested.is_set()


def test_controller_commands_shoulder_and_elbow_in_radians():
    env = HumanoidEnv()
    controller = SwingController(env)

    action = controller.get_action(0.0)

    assert set(action) == {
        controller.actuator_ids[SHOULDER_JOINT],
        controller.actuator_ids[SHOULDER_Y_JOINT],
        controller.actuator_ids[SHOULDER_Z_JOINT],
        controller.actuator_ids[ELBOW_JOINT],
    }
    assert all(np.isfinite(target) for target in action.values())


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
    np.testing.assert_allclose(env.omega1, np.pi / 2)
    np.testing.assert_allclose(env.omega2, np.pi / 2)

    for joint_name, expected_range in (
        (SHOULDER_JOINT, (-360.0, 360.0)),
        (SHOULDER_Y_JOINT, (-360.0, 360.0)),
        (SHOULDER_Z_JOINT, (-360.0, 360.0)),
        (ELBOW_JOINT, (0.0, 180.0)),
    ):
        joint_id = env.model.joint(joint_name).id
        np.testing.assert_allclose(
            np.rad2deg(env.model.jnt_range[joint_id]),
            expected_range,
        )

    controller = SwingController(env)
    assert controller.actuator_ids is env.actuator_ids
    assert controller.joint_names == (
        SHOULDER_JOINT,
        SHOULDER_Y_JOINT,
        SHOULDER_Z_JOINT,
        ELBOW_JOINT,
    )


def test_path1_phases_define_fixed_poses_and_connect_continuously():
    assert get_t1_pA_angles(0.0) == (THETA1_A_START, THETA2_A)
    assert get_t1_pA_angles(1.0) == (THETA1_A_END, THETA2_A)
    assert get_t1_pB_angles(0.0) == (THETA1_A_END, THETA2_A)
    assert get_t1_pB_angles(1.0) == (THETA1_B_END, THETA2_C)
    assert get_t1_pC_angles(0.0) == (THETA1_B_END, THETA2_C)
    assert get_t1_pC_angles(1.0) == (THETA1_C_END, THETA2_C)

    assert get_path1_angles(0.0) == get_t1_pA_angles(0.0)
    assert get_path1_angles(1 / 3) == get_t1_pB_angles(0.0)
    assert get_path1_angles(2 / 3) == get_t1_pC_angles(0.0)
    assert get_path1_angles(1.0) == get_t1_pC_angles(1.0)
    assert get_path1_angles(2.0) == get_path1_angles(1.0)


def test_path_geometry_is_independent_of_execution_speed():
    env = HumanoidEnv()
    controller = SwingController(env)
    original_geometry = [get_t1_pB_angles(progress) for progress in np.linspace(0, 1, 5)]
    original_duration = controller.phase_durations[1]

    env.omega1 *= 2
    env.omega2 *= 2
    faster_controller = SwingController(env)

    assert faster_controller.phase_durations[1] < original_duration
    assert [get_t1_pB_angles(progress) for progress in np.linspace(0, 1, 5)] == original_geometry


def test_controller_traverses_all_three_phases_and_holds_final_pose():
    env = HumanoidEnv()
    controller = SwingController(env)
    a_duration, b_duration, _ = controller.phase_durations

    phase_a_boundary = controller.get_action(a_duration)
    phase_b_boundary = controller.get_action(a_duration + b_duration)
    final_action = controller.get_action(controller.duration)

    # Apply each target and verify the resulting shoulder orientation is the
    # requested rotation about world X, not a torso-local hinge angle.
    for action, shoulder_deg in (
        (phase_a_boundary, THETA1_A_END),
        (phase_b_boundary, THETA1_B_END),
        (final_action, THETA1_C_END),
    ):
        env.apply_kinematic_action(action)
        actual_rotation = np.array(
            env.data.xmat[env.model.body("shoulder1_pivot").id]
        ).reshape(3, 3)
        expected_rotation = controller._rotation_xyz(
            np.deg2rad(shoulder_deg), 0.0, 0.0
        ) @ controller._reference_torso_rotation
        np.testing.assert_allclose(actual_rotation, expected_rotation, atol=1e-8)

    np.testing.assert_allclose(
        phase_b_boundary[controller.actuator_ids[ELBOW_JOINT]],
        np.deg2rad(THETA2_C),
    )
    assert controller.get_action(controller.duration + 1.0) == final_action


def test_target_site_prediction_matches_kinematic_pose_without_mutating_state():
    env = HumanoidEnv()
    controller = SwingController(env)
    action = controller.get_action(0.0)
    expected_target = env.get_target_site_position(action)

    env.apply_kinematic_action(action)

    np.testing.assert_allclose(env.data.site("ponta_taco").xpos, expected_target)


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