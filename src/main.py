import argparse
import threading
import time
import numpy as np

import glfw
import mujoco.viewer

from controllers import SwingController
from env import HumanoidEnv
from tools.csv import CSVData
from tools.parser import (
    parse_args,
    SIMULATION_MODES
)




def _make_key_callback(reset_requested: threading.Event):
    """
       Reset request
    """

    def key_callback(key):
        if key == glfw.KEY_BACKSPACE:
            reset_requested.set()

    return key_callback


def _advance_frame(env, controller, mode, locked_joint_names):

    sim_time = env.get_time()                   # Simulation moment
    action = controller.get_action(sim_time)    # Simulation actuation/action by the actuators

    #env.set_target_marker(env.get_target_site_position(action)) # GREEN MARKER!!!  # Predict the commanded pose on scratch data without changing the live state.
    # Physics mode:
    if mode in SIMULATION_MODES:
        for actuator_id, target in action.items():
            env.data.ctrl[actuator_id] = target
        env.step(locked_joint_names=locked_joint_names)

    else:
        raise ValueError(f"Unsupported simulation mode: {mode!r}")


def _viewer_telemetry(env):
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
        f"Time: {env.get_time():.3f} s\n"
        f"Ball position (m): {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}\n"
        #f"Ball velocity (m/s): {velocity[0]:.3f}, {velocity[1]:.3f}, {velocity[2]:.3f}\n"
        f"Ball speed: {ball['speed_m_s']:.3f} m/s\n"
        f"Club-ball contact force: {contact_force:.1f} N\n"
        f"Current max joint actuator effort: {peak_actuator_effort:.1f} N m\0"
    )
    right_text = (
        # f"Ball speed: {ball['speed_m_s']:.3f} m/s\n"
        # f"Club-ball contact force: {contact_force:.1f} N\n"
        f"\0"
    )
    return (
        mujoco.mjtFontScale.mjFONTSCALE_150,
        mujoco.mjtGridPos.mjGRID_TOPLEFT,
        left_text,
        right_text,
    )

def _check_rst(rst_event, env):
    if rst_event.is_set():
        rst_event.clear()
        env.reset()
        return True
    return False


def _init_main():
    args = parse_args()
    env = HumanoidEnv()
    ctrl = SwingController(env)    
    csv_data = CSVData()
    locked_jnt = env.get_locked_humanoid_joints(ctrl.joint_names)
    rst_event = threading.Event()
    return (
                args, 
                env, 
                ctrl, 
                csv_data, 
                locked_jnt, 
                rst_event
            )


def main():
    """
        Launches the MuJoCo viewer
        Runs the humanoid motion loop.
    """

    args, env, ctrl, csv_data, locked_jnt, rst_event = _init_main()

    print(f"Running swing in {args.mode} mode.")

    if csv_data is not None:
        CSVData._create_csv_file() # Creates the CSV file if it doesn't exist

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
            if(env is not None):
                try:
                    viewer.set_texts(_viewer_telemetry(env)) # Updates the GUI with telemetry
                except:
                    print(f"MuJoCo viewer has problems with Graphic drivers\n")
                

                csv_data.add_data_point(env) # Writes the data points to a CSV file
            viewer.sync() # Syncs mujoco GUI

            time_until_next_step = env.timestep - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step) # Sleeps until next step
    
    csv_data.save_to_csv()



if __name__ == "__main__":
    main()
