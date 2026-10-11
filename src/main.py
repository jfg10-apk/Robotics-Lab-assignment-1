import argparse
import threading
import time
from pathlib import Path

import glfw
import mujoco.viewer

from controllers import SwingController
from configuration import load_trial, select_case
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
    parser.add_argument(
        "--trial",
        default=str(Path(__file__).resolve().parents[1] / "trials" / "trial_1.toml"),
        help="TOML file containing trajectory and torque settings",
    )
    parser.add_argument(
        "--case",
        default="baseline",
        help="Named evaluation case in the trial TOML",
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
    position_targets = controller.get_position_targets(sim_time)
    if controller.direct_torque_profiles:
        env.set_target_marker(env.data.site("ponta_taco").xpos.copy())
    else:
        env.set_target_marker(env.get_target_site_position(position_targets))
    
    # Physics mode:
    if mode in ("ph", "physics"):
        controller.apply_region(sim_time)
        for actuator_id, torque in controller.get_torque_action(sim_time).items():
            env.data.ctrl[actuator_id] = torque
        env.step(locked_joint_names=locked_joint_names)

    # Kinematics only mode:
    elif mode in ("ki", "kinematics"):
        if controller.requires_physics:
            raise ValueError("This trial's torque feedback requires physics mode.")
        env.apply_kinematic_action(position_targets)
        env.advance_kinematic_time()
    else:
        raise ValueError(f"Unsupported simulation mode: {mode!r}")



def _init_main():
    args = parse_args()
    trial = load_trial(args.trial)
    case = select_case(trial, args.case)
    env = HumanoidEnv()
    ctrl = SwingController(env, trial, case)
    env.set_initial_pose(ctrl.get_position_targets(0.0))

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
            viewer.sync() # Syncs mujoco GUI

            time_until_next_step = env.timestep - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step) # Sleeps until next step


if __name__ == "__main__":
    main()
