import argparse
import threading
import time

import glfw
import mujoco.viewer

from controllers import SwingController
from env import HumanoidEnv

SIMULATION_MODES = ("physics", "kinematics")


def parse_args():
    """Read the simulation mode from the command line."""
    parser = argparse.ArgumentParser(description="Run the humanoid golf swing simulation.")
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
    """Apply one trajectory frame using the selected simulation mode."""
    sim_time = env.get_time()
    action = controller.get_action(sim_time)

    # The marker is a kinematic reference; physics-mode tip motion can lag it.
    env.set_target_marker(controller.get_target_point(sim_time))
    if mode == "physics":
        for actuator_id, target in action.items():
            env.data.ctrl[actuator_id] = target
        env.step(locked_joint_names=locked_joint_names)
    else:
        env.apply_kinematic_action(action)
        env.advance_kinematic_time()



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


def main():
    """Launch the MuJoCo viewer and run the humanoid motion loop."""
    # args = parse_args()
    # env = HumanoidEnv()
    # controller = SwingController(env)

    # locked_joint_names = tuple(
    #     env.model.joint(joint_id).name
    #     for joint_id in range(env.model.njnt)
    #     if env.model.joint(joint_id).name not in controller.joint_names
    # )
    # reset_requested = threading.Event()

    args, env, ctrl, locked_jnt, rst_event = _init_main()

    print(f"Running one-shot swing in {args.mode} mode.")
    print("Press Backspace in the viewer to reset the swing.")
    

    with mujoco.viewer.launch_passive(
        env.model,
        env.data,
        key_callback=_make_key_callback(rst_event),
    ) as viewer:
        while viewer.is_running():
            step_start = time.perf_counter()
            # if rst_event.is_set():
            #     rst_event.clear()
            #     env.reset()

            _check_rst(rst_event, env)
            _advance_frame(env, ctrl, args.mode, locked_jnt)
            viewer.sync()

            time_until_next_step = env.timestep - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
