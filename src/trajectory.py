"""Editable, piecewise-linear joint trajectories for the swing controller."""

from collections.abc import Sequence

import numpy as np

SHOULDER_JOINT = "shoulder1_right"

# Add or edit keyframes here: (time in seconds, joint angle in degrees).
# Add another joint-name entry when that joint should join the trajectory.
JOINT_TRAJECTORIES = {
    SHOULDER_JOINT: (
        (0.0, -55.0),
        (2.0, 0.0),
    ),
}
TRAJECTORY_DURATION = max(
    keyframe[0]
    for keyframes in JOINT_TRAJECTORIES.values()
    for keyframe in keyframes
)


def _validated_keyframes(
    keyframes: Sequence[tuple[float, float]],
) -> tuple[np.ndarray, np.ndarray]:
    """Convert keyframes to arrays and reject malformed trajectory data."""
    if len(keyframes) == 0:
        raise ValueError("A joint trajectory must contain at least one keyframe.")

    values = np.asarray(keyframes, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2:
        raise ValueError("Each keyframe must be a (time, angle) pair.")
    if not np.isfinite(values).all():
        raise ValueError("Trajectory keyframes must contain only finite values.")

    times = values[:, 0]
    if np.any(np.diff(times) <= 0.0):
        raise ValueError("Trajectory keyframe times must be strictly increasing.")

    return times, values[:, 1]


def get_target_angle(
    time: float,
    keyframes: Sequence[tuple[float, float]],
) -> float:
    """Interpolate a degree-based joint trajectory and return radians."""
    if not np.isfinite(time):
        raise ValueError("Trajectory time must be finite.")

    times, angles_degrees = _validated_keyframes(keyframes)
    # np.interp holds the first and last pose outside the keyframe interval.
    angle_degrees = np.interp(time, times, angles_degrees)
    return float(np.deg2rad(angle_degrees))


def get_target_point(
    time: float,
    pivot: Sequence[float],
    axis: Sequence[float],
    initial_tip: Sequence[float],
    initial_angle: float,
    keyframes: Sequence[tuple[float, float]],
) -> np.ndarray:
    """Rotate the initial club tip around the shoulder's world-space hinge."""
    pivot = np.asarray(pivot, dtype=float)
    axis = np.asarray(axis, dtype=float)
    initial_tip = np.asarray(initial_tip, dtype=float)
    vectors = (pivot, axis, initial_tip)
    if any(vector.shape != (3,) for vector in vectors):
        raise ValueError("Pivot, axis, and initial tip must be 3D vectors.")
    if not all(np.isfinite(vector).all() for vector in vectors):
        raise ValueError("Pivot, axis, and initial tip must be finite.")

    axis_length = np.linalg.norm(axis)
    if axis_length == 0.0:
        raise ValueError("The shoulder rotation axis must be non-zero.")
    axis /= axis_length

    if not np.isfinite(initial_angle):
        raise ValueError("The initial shoulder angle must be finite.")

    # Rodrigues' rotation formula gives the exact circular locus of the tip.
    angle = get_target_angle(time, keyframes) - initial_angle
    radius = initial_tip - pivot
    rotated_radius = (
        radius * np.cos(angle)
        + np.cross(axis, radius) * np.sin(angle)
        + axis * np.dot(axis, radius) * (1.0 - np.cos(angle))
    )
    return pivot + rotated_radius
