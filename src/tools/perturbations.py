"""Run a normal swing and three independent external-torque tests."""

from pathlib import Path
import sys

# Direct execution must not import tools/csv.py as Python's standard csv module.
if __package__ in (None, ""):
    sys.path[0] = str(Path(__file__).resolve().parents[1])

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from controllers import SwingController
from env import HumanoidEnv
from tools.csv import CSVData

TORQUE_NM = -3.0
START_S = 4.35
END_S = 4.50
DURATION_S = 6.0
# Actuator names: resolve their actual joint names from the model.
CASES = {"referencia": None, "ombro": "shoulder1_right",
         "cotovelo": "elbow_right", "pulso": "pulso_right"}
RESULTS_DIR = Path(__file__).resolve().parent / "results"


def _simulate_case(case, actuator_name, output, torque_nm, show=True):
    env = HumanoidEnv()
    ctrl = SwingController(env)
    locked = env.get_locked_humanoid_joints(ctrl.joint_names)
    joint_name = None
    dof = None
    if actuator_name is not None:
        actuator_id = env.actuator_ids[actuator_name]
        joint_id = env.actuator_joint_ids[actuator_id]
        joint_name = env.model.joint(joint_id).name
        dof = int(env.model.jnt_dofadr[joint_id])
    path = output / f"{case}.csv"
    logger = CSVData(file_path=path)
    CSVData._create_csv_file(env, path)
    steps = round(DURATION_S / env.timestep)
    viewer = None
    completed = True
    print(f"Ensaio: {case}; junta: {joint_name or 'nenhuma'}; torque externo: {torque_nm if dof is not None else 0:+g} N m.")
    try:
        if show:
            viewer = mujoco.viewer.launch_passive(env.model, env.data)
        for step in range(steps + 1):
            if viewer is not None and not viewer.is_running():
                completed = False
                break
            step_start = time.perf_counter()
            for actuator_id, target in ctrl.get_action(env.get_time()).items():
                env.data.ctrl[actuator_id] = target
            env.data.qfrc_applied[:] = 0.0
            t = round(env.get_time(), 9)
            if dof is not None and START_S <= t < END_S:
                env.data.qfrc_applied[dof] = torque_nm
            # Refresh sensors and forces to record the same simulation instant.
            mujoco.mj_forward(env.model, env.data)
            if (not np.isfinite(env.data.qpos).all()
                    or not np.isfinite(env.data.qvel).all()
                    or any(w.number for w in env.data.warning)):
                raise RuntimeError(f"Simulacao instavel em {case}; CSV contem apenas as amostras validas.")
            logger.add_data_point(env)
            if viewer is not None:
                viewer.sync()
            if step < steps:
                previous_time = env.get_time()
                env.step(locked_joint_names=locked)
                if env.get_time() <= previous_time:
                    raise RuntimeError(f"O tempo da simulacao deixou de avancar em {case}.")
                if viewer is not None:
                    time.sleep(max(0.0, env.timestep - (time.perf_counter() - step_start)))
    finally:
        env.data.qfrc_applied[:] = 0.0
        try:
            logger.save_to_csv()
        finally:
            if viewer is not None:
                viewer.close()
                # close() signals the GUI thread; let it release the window
                # before opening the next viewer or exiting Python.
                time.sleep(1.0)
    print(f"{'Concluido' if completed else 'Interrompido'}: {case}; CSV: {path}")
    return completed


def run_perturbation_tests(torque_nm=TORQUE_NM, case=None, show=True):
    """Show each test in real time and save its CSV; closing early stops the tests."""
    if not np.isfinite(torque_nm) or not 0 <= START_S < END_S < DURATION_S:
        raise ValueError("Usar torque finito e 0 <= START_S < END_S < DURATION_S.")
    output = RESULTS_DIR / f"torque_{torque_nm:g}Nm"
    output.mkdir(parents=True, exist_ok=True)
    print(f"Resultados: {output}")
    cases = CASES if case is None else {case: CASES[case]}
    for name, actuator_name in cases.items():
        if not _simulate_case(name, actuator_name, output, torque_nm, show):
            print("Ensaios interrompidos ao fechar a janela; amostras recolhidas guardadas.")
            break
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Swing normal e perturbacoes no ombro, cotovelo e pulso.")
    parser.add_argument("--torque", type=float, default=TORQUE_NM,
                        help="Torque externo em N m; pode ser positivo ou negativo.")
    parser.add_argument("--case", choices=CASES,
                        help="Mostrar apenas este ensaio; por defeito, mostrar os quatro.")
    args = parser.parse_args()
    run_perturbation_tests(args.torque, case=args.case)
