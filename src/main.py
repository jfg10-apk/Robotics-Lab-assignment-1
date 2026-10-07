import argparse
import threading
import time

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



def _init_main():
    args = parse_args()
    env = HumanoidEnv()
    ctrl = SwingController(env)

    locked_jnt = tuple(
        env.model.joint(joint_id).name
        for joint_id in range(env.model.njnt)
        if env.model.joint(joint_id).name not in ctrl.joint_names
    )
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
