import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
sys.path.insert(0, str(SRC_DIR))

from configuration import load_trial, select_case
from controllers import SwingController
from env import HumanoidEnv
from trajectory import get_joint_velocity_targets


TRIAL_PATH = PROJECT_ROOT / "trials" / "trial_1.toml"
PENDULUM_TRIAL_PATH = PROJECT_ROOT / "trials" / "trial_2.toml"
REGIONAL_TRIAL_PATH = PROJECT_ROOT / "trials" / "trial_3.toml"


def make_controller(case_name="baseline"):
    trial = load_trial(TRIAL_PATH)
    case = (
        {"name": case_name, "torque_scale": 0.7, "aim_yaw_deg": -10.0}
        if case_name == "reduced_torque_left"
        else select_case(trial, case_name)
    )
    env = HumanoidEnv()
    return env, SwingController(env, trial, case)


def test_toml_keyframes_remain_position_targets_in_radians():
    env, controller = make_controller()
    shoulder_id = env.actuator_ids["shoulder1_right"]

    assert controller.get_position_targets(0.0)[shoulder_id] == 0.0
    np.testing.assert_allclose(
        controller.get_position_targets(4.0)[shoulder_id],
        np.deg2rad(-90.0),
    )


def test_pd_controller_saturates_torque_at_trial_limit():
    env, controller = make_controller()
    shoulder_id = env.actuator_ids["shoulder1_right"]
    shoulder_joint = env.actuator_joint_ids[shoulder_id]
    qpos_address = env.model.jnt_qposadr[shoulder_joint]
    env.data.qpos[qpos_address] = -1.0

    torque = controller.get_torque_action(0.0)[shoulder_id]

    assert torque == 50.0
    np.testing.assert_allclose(env.model.actuator_forcerange[shoulder_id], [-50.0, 50.0])


def test_selected_case_scales_torque_limit_and_applies_local_aim_offset():
    env, controller = make_controller("reduced_torque_left")
    shoulder3_id = env.actuator_ids["shoulder3_right"]
    shoulder1_id = env.actuator_ids["shoulder1_right"]

    np.testing.assert_allclose(
        env.model.actuator_forcerange[shoulder1_id], [-35.0, 35.0]
    )
    np.testing.assert_allclose(
        controller.get_position_targets(4.0)[shoulder3_id],
        np.deg2rad(-10.0),
    )


def test_pendulum_trial_commands_a_single_shoulder_torque_pulse():
    trial = load_trial(PENDULUM_TRIAL_PATH)
    env = HumanoidEnv()
    controller = SwingController(env, trial, select_case(trial, "baseline"))
    shoulder_id = env.actuator_ids["shoulder1_right"]

    assert controller.joint_names == ("shoulder1_right",)
    np.testing.assert_allclose(
        [controller.get_torque_action(time)[shoulder_id]
         for time in (0.0, 0.05, 0.125, 0.2, 0.25, 0.5, 2.0)],
        [0.0, 8.0, 8.0, 8.0, 0.0, 0.0, 0.0],
        atol=1e-12,
    )


def test_pendulum_torque_moves_only_the_right_shoulder():
    trial = load_trial(PENDULUM_TRIAL_PATH)
    env = HumanoidEnv()
    controller = SwingController(env, trial, select_case(trial, "baseline"))
    env.set_initial_pose(controller.get_position_targets(0.0))
    locked = env.get_locked_humanoid_joints(controller.joint_names)
    shoulder_id = env.actuator_ids["shoulder1_right"]
    shoulder_joint = env.actuator_joint_ids[shoulder_id]
    shoulder_qpos = env.model.jnt_qposadr[shoulder_joint]
    elbow_qpos = env.model.jnt_qposadr[env.model.joint("elbow_right").id]
    initial_shoulder = env.data.qpos[shoulder_qpos]
    initial_elbow = env.data.qpos[elbow_qpos]

    assert "shoulder1_right" not in locked
    assert "elbow_right" in locked
    assert "shoulder2_right" in locked
    assert "shoulder3_right" in locked
    for _ in range(round(0.5 / env.timestep)):
        for actuator_id, torque in controller.get_torque_action(env.get_time()).items():
            env.data.ctrl[actuator_id] = torque
        env.step(locked_joint_names=locked)

    assert env.data.qpos[shoulder_qpos] != initial_shoulder
    np.testing.assert_allclose(env.data.qpos[elbow_qpos], initial_elbow)


def test_trial_three_reuses_trial_one_angle_trajectory():
    baseline = load_trial(TRIAL_PATH)["trajectory"]
    regional = load_trial(REGIONAL_TRIAL_PATH)["trajectory"]

    assert baseline["phase_times_s"] == regional["phase_times_s"]
    assert baseline["joints"] == regional["joints"]


def test_angle_speed_torque_uses_velocity_error_and_region_gains():
    trial = load_trial(REGIONAL_TRIAL_PATH)
    env = HumanoidEnv()
    controller = SwingController(env, trial, select_case(trial, "baseline"))
    shoulder_id = env.actuator_ids["shoulder1_right"]
    shoulder_joint = env.actuator_joint_ids[shoulder_id]
    qpos = env.model.jnt_qposadr[shoulder_joint]
    dof = env.model.jnt_dofadr[shoulder_joint]
    angle_error = np.deg2rad(1.0)

    observed = []
    for time, expected_gain in ((2.0, 80.0), (4.35, 200.0)):
        target = controller.get_position_targets(time)[shoulder_id]
        env.data.qpos[qpos] = target - angle_error
        env.data.qvel[dof] = 0.0
        torque_at_reference_speed = controller.get_torque_action(time)[shoulder_id]
        env.data.qvel[dof] = 0.1
        torque_with_speed_error = controller.get_torque_action(time)[shoulder_id]
        speed_gain = controller.region_at(time)["speed_control_nm_s_per_rad"]
        np.testing.assert_allclose(
            torque_at_reference_speed,
            expected_gain * angle_error + speed_gain * (
                    get_joint_velocity_targets(
                        time, controller.trajectory
                    )["shoulder1_right"]
            ),
        )
        np.testing.assert_allclose(
            torque_with_speed_error,
            torque_at_reference_speed - speed_gain * 0.1,
        )
        observed.append(torque_with_speed_error)

        controller.apply_region(time)
        expected_damping = 0.2 if time < 4.0 else 1.0
        np.testing.assert_allclose(env.model.dof_damping[dof], expected_damping)

    assert controller.feedback_mode == "angle_speed"
    assert observed[1] > observed[0]