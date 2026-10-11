import sys
from pathlib import Path

import mujoco
import numpy as np

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

from controllers import SwingController
from env import HumanoidEnv
from main import _advance_frame
from configuration import load_trial

TRIAL_DURATION_SECONDS = load_trial()["trajectory"]["phase_times_s"][-1]


def test_kinematic_frame_updates_target_and_advances_clock():
    env = HumanoidEnv()
    controller = SwingController(env)
    expected_target = env.get_target_site_position(
        controller.get_position_targets(0.0)
    )

    _advance_frame(env, controller, "kinematics", ())

    assert env.get_time() == env.timestep
    np.testing.assert_allclose(
        env.data.mocap_pos[env.target_marker_mocap_id],
        expected_target,
    )


def test_physics_frame_advances_model_time():
    env = HumanoidEnv()
    controller = SwingController(env)

    _advance_frame(env, controller, "physics", ())

    assert env.get_time() == env.timestep


def test_environment_reset_restores_pose_and_simulation_clock():
    env = HumanoidEnv()
    controller = SwingController(env)
    initial_tip = env.data.site("ponta_taco").xpos.copy()

    for _ in range(round(1.0 / env.timestep)):
        env.apply_kinematic_action(
            controller.get_position_targets(env.get_time())
        )
        env.advance_kinematic_time()
    assert env.get_time() >= 1.0
    assert not np.allclose(env.data.site("ponta_taco").xpos, initial_tip)

    env.reset()

    assert env.get_time() == 0.0
    np.testing.assert_allclose(env.data.qpos, env.initial_qpos)
    np.testing.assert_allclose(env.data.site("ponta_taco").xpos, initial_tip)


def test_simulation_step_advances_by_model_timestep():
    env = HumanoidEnv()

    env.step()

    assert env.get_time() == env.timestep
    assert np.isfinite(env.data.qpos).all()
    assert np.isfinite(env.data.qvel).all()


def test_gravity_is_enabled_and_generalized_acceleration_is_finite():
    env = HumanoidEnv()
    root_joint_id = env.model.joint("root").id
    root_dof_address = env.model.jnt_dofadr[root_joint_id]
    vertical_translation_dof = root_dof_address + 2

    assert env.model.jnt_type[root_joint_id] == mujoco.mjtJoint.mjJNT_FREE
    assert env.model.opt.gravity[2] < 0.0

    mujoco.mj_forward(env.model, env.data)

    assert env.data.qacc.shape == (env.model.nv,)
    assert np.isfinite(env.data.qacc).all()
    assert np.isfinite(env.data.qacc[vertical_translation_dof])


def test_kinematic_action_updates_joint_positions_without_stepping_time():
    env = HumanoidEnv()
    controller = SwingController(env)
    driven_joint_id = env.model.actuator_trnid[
        controller.actuator_ids["shoulder1_right"], 0
    ]
    qpos_address = env.model.jnt_qposadr[driven_joint_id]

    env.apply_kinematic_action(controller.get_position_targets(0.0))
    initial_angle = env.data.qpos[qpos_address]
    initial_time = env.get_time()

    env.apply_kinematic_action(controller.get_position_targets(1.0))

    assert env.data.qpos[qpos_address] != initial_angle
    assert env.get_time() == initial_time == 0.0


def test_physics_swing_tracks_joint_targets_with_bounded_torque():
    env = HumanoidEnv()
    controller = SwingController(env)
    env.set_initial_pose(controller.get_position_targets(0.0))
    locked_joint_names = tuple(
        env.model.joint(joint_id).name
        for joint_id in range(env.model.njnt)
        if env.model.joint(joint_id).name not in controller.joint_names
    )
    fixed_body_names = ("torso",)
    initial_positions = {
        name: env.data.xpos[env.model.body(name).id].copy()
        for name in fixed_body_names
    }
    shoulder_joint_id = env.model.actuator_trnid[
        controller.actuator_ids["shoulder1_right"], 0
    ]
    elbow_joint_id = env.model.joint("elbow_right").id
    shoulder_qpos = env.model.jnt_qposadr[shoulder_joint_id]
    elbow_qpos = env.model.jnt_qposadr[elbow_joint_id]
    initial_shoulder_angle = env.data.qpos[shoulder_qpos]
    initial_elbow_angle = env.data.qpos[elbow_qpos]
    steps = int(TRIAL_DURATION_SECONDS / env.timestep) + 1
    for _ in range(steps):
        action = controller.get_torque_action(env.get_time())
        for actuator_id, torque in action.items():
            env.data.ctrl[actuator_id] = torque
            limit = env.model.actuator_forcerange[actuator_id, 1]
            assert abs(torque) <= limit
        env.step(locked_joint_names=locked_joint_names)

    assert env.get_time() >= TRIAL_DURATION_SECONDS
    assert env.data.qpos[shoulder_qpos] != initial_shoulder_angle
    assert env.data.qpos[elbow_qpos] != initial_elbow_angle
    for name, position in initial_positions.items():
        np.testing.assert_allclose(env.data.xpos[env.model.body(name).id], position)

    for _ in range(20):
        for actuator_id, torque in controller.get_torque_action(env.get_time()).items():
            env.data.ctrl[actuator_id] = torque
        env.step(locked_joint_names=locked_joint_names)
    assert np.isfinite(env.data.qfrc_actuator).all()
