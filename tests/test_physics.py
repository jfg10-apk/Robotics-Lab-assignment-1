import sys
from pathlib import Path

import mujoco
import numpy as np

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

from controllers import SwingController
from env import HumanoidEnv
from main import _advance_frame
def test_kinematic_frame_updates_target_and_advances_clock():
    env = HumanoidEnv()
    controller = SwingController(env)
    expected_target = env.get_target_site_position(controller.get_action(0.0))

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
        env.apply_kinematic_action(controller.get_action(env.get_time()))
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


def test_gravity_accelerates_root_downward():
    env = HumanoidEnv()
    root_joint_id = env.model.joint("root").id
    root_dof_address = env.model.jnt_dofadr[root_joint_id]
    vertical_translation_dof = root_dof_address + 2

    assert env.model.jnt_type[root_joint_id] == mujoco.mjtJoint.mjJNT_FREE
    assert env.model.opt.gravity[2] < 0.0

    mujoco.mj_forward(env.model, env.data)

    assert env.data.qacc.shape == (env.model.nv,)
    assert np.isfinite(env.data.qacc).all()
    assert env.data.qacc[vertical_translation_dof] < 0.0


def test_kinematic_action_updates_joint_positions_without_stepping_time():
    env = HumanoidEnv()
    controller = SwingController(env)
    driven_joint_id = env.model.actuator_trnid[
        controller.actuator_ids["shoulder1_right"], 0
    ]
    qpos_address = env.model.jnt_qposadr[driven_joint_id]

    env.apply_kinematic_action(controller.get_action(0.0))
    initial_angle = env.data.qpos[qpos_address]
    initial_time = env.get_time()

    env.apply_kinematic_action(controller.get_action(1.0))

    assert env.data.qpos[qpos_address] != initial_angle
    assert env.get_time() == initial_time == 0.0


def test_physics_swing_moves_only_the_arm_once():
    env = HumanoidEnv()
    controller = SwingController(env)
    locked_joint_names = tuple(
        env.model.joint(joint_id).name
        for joint_id in range(env.model.njnt)
        if env.model.joint(joint_id).name not in controller.joint_names
    )
    fixed_body_names = ("torso", "pelvis", "thigh_right", "thigh_left")
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
    maximum_elbow_angle = initial_elbow_angle

    steps = int(controller.duration / env.timestep) + 1
    for _ in range(steps):
        action = controller.get_action(env.get_time())
        for actuator_id, target in action.items():
            env.data.ctrl[actuator_id] = target
        env.step(locked_joint_names=locked_joint_names)
        maximum_elbow_angle = max(maximum_elbow_angle, env.data.qpos[elbow_qpos])

    assert env.get_time() >= controller.duration
    assert env.data.qpos[shoulder_qpos] != initial_shoulder_angle
    assert maximum_elbow_angle > initial_elbow_angle
    for name, position in initial_positions.items():
        np.testing.assert_allclose(env.data.xpos[env.model.body(name).id], position)

    final_action = controller.get_action(env.get_time())
    final_target = final_action[controller.actuator_ids["shoulder1_right"]]
    for _ in range(20):
        for actuator_id, target in final_action.items():
            env.data.ctrl[actuator_id] = target
        env.step(locked_joint_names=locked_joint_names)
    assert controller.get_action(env.get_time())[
        controller.actuator_ids["shoulder1_right"]
    ] == final_target
