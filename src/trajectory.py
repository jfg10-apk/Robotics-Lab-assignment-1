"""
    Description of the trajectories + actuation

    The current test motion turns the first right-shoulder hinge 180 degrees over
six seconds. Targets are returned in radians, as required by MuJoCo.
"""

from collections.abc import Callable

import numpy as np

SHOULDER_RX = "shoulder1_right"
ELBOW_RY = "elbow_right"


START_ANGLE_DEGREES = -90.0
END_ANGLE_DEGREES = 90.0
OMEGA_RX_A = 60.0 # [degrees/s]
OMEGA_RY_B = 300.0 # [degrees/s]




START_JNTRX_PART_A = -100.0
START_JNTRX_PART_B = -60.0
START_JNTRX_PART_C = 0.0

START_JNTRY_PART_B = 35.0
START_JNTRY_PART_C = 180.0


START_TIME = 0.0
DURATION_A = (
    START_JNTRX_PART_C - START_JNTRX_PART_A
) / OMEGA_RX_A

DURATION_B = (
    START_JNTRY_PART_C - START_JNTRY_PART_B
) / OMEGA_RY_B



# Add another joint here, mapping its MuJoCo joint name to an angle function
# that accepts time in seconds and returns its angle in degrees.


def _clip_time(sim_time: float, duration: float) -> float:
    return float(np.clip(sim_time, START_TIME, duration))

def _duration(start_phi: float, end_phi: float, omega: float) -> float:
    return (end_phi - start_phi) / omega


def shoulder_rx(delta_t: float) -> float:
    """
        Evaluates theta1(t), the angle of the right shoulder.

        Hold the one-shot motion at its endpoint before t=0 and after its
    six-second duration.

        To be enhanced with inverse kinematics plan.
    """

    return (OMEGA_RX_A * float(_clip_time(delta_t, DURATION_A)) +
             + START_JNTRX_PART_A)


def elbow_ry(delta_t: float) -> float:
    """
        Evaluates theta2(t), the angle of the right elbow.
    """
    return (START_JNTRY_PART_C - OMEGA_RY_B * float(_clip_time(delta_t, DURATION_B)))


def static_joint(delta_t: float) -> float:
    return -90.0
"""
    Multi Joint mapping dictionary:
    str: Joint name given to XML model body part
    float array: time
    float: angle (degrees)
"""

JOINT_TRAJECTORIES: dict[str, Callable[[float], float]] = {
    SHOULDER_RX: static_joint, # Right shoulder, x axis
    ELBOW_RY: elbow_ry # Right elbow, y axis
}



def get_joint_targets(sim_t: float) -> dict[str, float]:
    """
        Return targets for all configured joints, converted from degrees to radians.
    """
    return {
        joint_name: float(np.deg2rad(joint_trajectory(sim_t)))
        for joint_name, joint_trajectory in JOINT_TRAJECTORIES.items()
    }


