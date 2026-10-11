import argparse
import threading
import time
import numpy as np

import glfw
import mujoco.viewer

from controllers import SwingController
from env import HumanoidEnv

SIMULATION_MODES = ("physics", "kinematics", "ph", "ki")


def parse_args():
    """
        Reads arguments from CMD line
    """
    parser = argparse.ArgumentParser(description="Runs mujoco golf player swing project.")
    parser.add_argument(
        "--mode",
        choices=SIMULATION_MODES,
        default="physics",
        help=(
            "physics advances MuJoCo with actuators; kinematics sets joint "
            "positions directly without stepping physics"
        ),
    )
    return parser.parse_args()


def _make_key_callback(reset_requested: threading.Event):
    """Queue a reset request; model state stays on the simulation thread."""

    def key_callback(key):
        if key == glfw.KEY_BACKSPACE:
            reset_requested.set()

    return key_callback


def _advance_frame(env, controller, mode, locked_joint_names):
    """
        Apply one trajectory frame using the selected simulation mode.
    """

    sim_time = env.get_time()                   # Simulation moment
    action = controller.get_action(sim_time)    # Simulation actuation/action by the actuators

   

    env.set_target_marker(env.get_target_site_position(action)) # GREEN MARKER!!!  # Predict the commanded pose on scratch data without changing the live state.
    
    # Physics mode:
    if mode in ("ph", "physics"):
        for actuator_id, target in action.items():
            env.data.ctrl[actuator_id] = target
        env.step(locked_joint_names=locked_joint_names)

    # Kinematics only mode:
    elif mode in ("ki", "kinematics"):
        env.apply_kinematic_action(action)
        env.advance_kinematic_time()
    else:
        raise ValueError(f"Unsupported simulation mode: {mode!r}")


def _viewer_telemetry(env, controller):
    ball = env.get_ball_state()
    ball_geom = env.model.geom("bola").id
    club_geom = env.model.geom("cabeca_taco").id
    contact_force = 0.0
    for contact_id in range(env.data.ncon):
        contact = env.data.contact[contact_id]
        if {contact.geom1, contact.geom2} != {ball_geom, club_geom}:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(env.model, env.data, contact_id, force)
        contact_force = max(contact_force, float(np.linalg.norm(force[:3])))

    peak_actuator_effort = float(np.max(np.abs(env.data.actuator_force)))
    position = ball["position_m"]
    velocity = ball["velocity_m_s"]
    left_text = (
        f"Case: {controller.case['name']}\n"
        f"Time: {env.get_time():.3f} s\n"
        f"Ball position (m): {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}\n"
        f"Ball velocity (m/s): {velocity[0]:.3f}, {velocity[1]:.3f}, {velocity[2]:.3f}\n"
        f"Ball speed: {ball['speed_m_s']:.3f} m/s\n"
        f"Club-ball contact force: {contact_force:.1f} N\n"
        f"Current max joint actuator effort: {peak_actuator_effort:.1f} N m"
    )
    right_text = (
        # f"Ball speed: {ball['speed_m_s']:.3f} m/s\n"
        # f"Club-ball contact force: {contact_force:.1f} N\n"
        f"\n"
    )
    return (
        mujoco.mjtFontScale.mjFONTSCALE_150,
        mujoco.mjtGridPos.mjGRID_TOPLEFT,
        left_text,
        right_text,
    )


def _init_main():
    args = parse_args()
    env = HumanoidEnv()
    ctrl = SwingController(env)

    locked_jnt = env.get_locked_humanoid_joints(ctrl.joint_names)
    rst_event = threading.Event()
    return args, env, ctrl, locked_jnt, rst_event

def _check_rst(rst_event, env):
    if rst_event.is_set():
        rst_event.clear()
        env.reset()
        return True
    return False


def main():
    """
        Initializes classes: env, ctrl, locked_jnt, rst_event (to reset with GUI)
        Launches the MuJoCo viewer
        Runs the humanoid motion loop.
    """

    args, env, ctrl, locked_jnt, rst_event = _init_main()

    print(f"Running one-shot swing in {args.mode} mode.")
    print("Press Backspace in the viewer to reset the swing.")
    

    with mujoco.viewer.launch_passive(
        env.model,
        env.data,
        key_callback=_make_key_callback(rst_event),
    ) as viewer:
        while viewer.is_running():
            step_start = time.perf_counter() # Registers start time

            # Resetting the simulation clock also resets the stateless controller.
            _check_rst(rst_event, env)
            _advance_frame(env, ctrl, args.mode, locked_jnt) # Builds next frame
            viewer.set_texts(_viewer_telemetry(env, ctrl)) # Updates the GUI with telemetry
            viewer.sync() # Syncs mujoco GUI

            time_until_next_step = env.timestep - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step) # Sleeps until next step


if __name__ == "__main__":
    main()
