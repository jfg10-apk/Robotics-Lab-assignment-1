import numpy as np

from trajectory import (
    ELBOW_JOINT,
    SHOULDER_JOINT,
    THETA1_A_END,
    THETA1_A_START,
    THETA1_B_END,
    THETA1_C_END,
    THETA2_A,
    THETA2_C,
    get_path1_angles,
)


class SwingController:
    """Map fixed Path 1 poses onto actuators using a separate timing profile."""

    def __init__(self, env):
        self.actuator_ids = env.actuator_ids
        self.env = env
        self.joint_names = (SHOULDER_JOINT, ELBOW_JOINT)
        # Speeds determine traversal time only; they do not alter Path 1 poses.
        if env.omega1 <= 0.0 or env.omega2 <= 0.0:
            raise ValueError("Shoulder and elbow angular speeds must be positive.")
        self.phase_durations = (
            np.deg2rad(THETA1_A_START - THETA1_A_END) / env.omega1,
            max(
                np.deg2rad(THETA1_A_END - THETA1_B_END) / env.omega1,
                np.deg2rad(THETA2_A - THETA2_C) / env.omega2,
            ),
            np.deg2rad(THETA1_B_END - THETA1_C_END) / env.omega1,
        )
        self.duration = sum(self.phase_durations)

    def get_action(self, sim_time):
        """Return current joint targets in radians for the selected path phase."""
        path_progress = self._path_progress_at_time(sim_time)
        shoulder_deg, elbow_deg = get_path1_angles(path_progress)

        return {
            self.actuator_ids[SHOULDER_JOINT]: float(np.deg2rad(shoulder_deg)),
            self.actuator_ids[ELBOW_JOINT]: float(np.deg2rad(elbow_deg)),
        }

    def _path_progress_at_time(self, sim_time):
        """Map elapsed simulation time to normalized whole-path progress."""
        if not np.isfinite(sim_time):
            raise ValueError("Simulation time must be finite.")
        elapsed = max(0.0, float(sim_time))
        for phase_index, duration in enumerate(self.phase_durations):
            if elapsed < duration:
                phase_progress = elapsed / duration
                return (phase_index + phase_progress) / len(self.phase_durations)
            elapsed -= duration

        # Hold the final Path 1 pose after the one-shot swing completes.
        return 1.0
