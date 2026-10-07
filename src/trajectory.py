"""Fixed joint-space path definitions, independent from timing and actuator response.

Path 1 is the 2D swing path, split into three geometric phases. A phase progress
of 0 means its starting pose and 1 means its ending pose. The controller maps
simulation time to phase progress using a separate speed profile.
"""

import numpy as np

SHOULDER_JOINT = "shoulder1_right"
SHOULDER_Y_JOINT = "shoulder2_right"
SHOULDER_Z_JOINT = "shoulder3_right"
ELBOW_JOINT = "elbow_right"
ABDOMEN_Z_JOINT = "abdomen_z"

# Path 1's fixed shoulder angles. Edit these to change its geometry.
THETA1_A_START  = -90.0
THETA1_A_END    = -30.0
THETA1_B_END    = 0.0
THETA1_C_END    = 90.0

# Elbow angles set effective radius: 180° folds the links (minimum reach);
# 0° aligns them (maximum reach). Phase B transitions between these poses.
THETA2_A    = 150.0
THETA2_C    = 0.0

THETA_ABD_NEUTRAL = 0.0   # Fica neutro (0°) durante o Backswing e Impacto
THETA_ABD_END     = 45.0  # Roda 45° apenas no Follow-through (após o impacto)


def _progress(value: float) -> float:
    """Validate and clamp normalized phase progress to [0, 1]."""
    if not np.isfinite(value):
        raise ValueError("Phase progress must be finite.")
    return float(np.clip(value, 0.0, 1.0))


def get_t1_pA_angles(progress: float) -> tuple[float, float, float]:
    """Path 1A: shoulder 90° -> 30° with a constant minimum radius."""
    p = _progress(progress)
    shoulder = THETA1_A_START + p * (THETA1_A_END - THETA1_A_START)
    return shoulder, THETA2_A, THETA_ABD_NEUTRAL


def get_t1_pB_angles(progress: float) -> tuple[float, float, float]:
    """Path 1B: shoulder 30° -> 0° while elbow extension grows the radius."""
    p = _progress(progress)
    shoulder = THETA1_A_END + p * (THETA1_B_END - THETA1_A_END)
    elbow = THETA2_A + p * (THETA2_C - THETA2_A)
    return shoulder, elbow, THETA_ABD_NEUTRAL


def get_t1_pC_angles(progress: float) -> tuple[float, float, float]:
    """Path 1C: shoulder 0° -> -90° with a constant maximum radius."""
    p = _progress(progress)
    shoulder = THETA1_B_END + p * (THETA1_C_END - THETA1_B_END)
    abdomen_z = THETA_ABD_NEUTRAL + p * (THETA_ABD_END - THETA_ABD_NEUTRAL)
    return shoulder, THETA2_C, abdomen_z


def get_path1_angles(progress: float) -> tuple[float, float, float]:
    """Sample fixed Path 1 with normalized whole-path progress in [0, 1]."""
    phase_position = _progress(progress) * 3.0
    phase_index = min(int(phase_position), 2)
    phase_progress = phase_position - phase_index

    # At the final endpoint return phase C's ending pose, not its start.
    if phase_position == 3.0:
        phase_progress = 1.0

    phase_functions = (
        get_t1_pA_angles,
        get_t1_pB_angles,
        get_t1_pC_angles,
    )
    return phase_functions[phase_index](phase_progress)
