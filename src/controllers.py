import numpy as np

from trajectory import (
    ABDOMEN_Z_JOINT,
    ELBOW_JOINT,
    SHOULDER_JOINT,
    SHOULDER_Y_JOINT,
    SHOULDER_Z_JOINT,
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
        self.joint_names = (
            SHOULDER_JOINT,
            SHOULDER_Y_JOINT,
            SHOULDER_Z_JOINT,
            ELBOW_JOINT,
            ABDOMEN_Z_JOINT,
        )
        self._reference_torso_rotation = np.array(
            env.data.xmat[env.model.body("torso").id], dtype=float
        ).reshape(3, 3)
        # Speeds determine traversal time only; they do not alter Path 1 poses.
        if env.omega1 <= 0.0 or env.omega2 <= 0.0:
            raise ValueError("Shoulder and elbow angular speeds must be positive.")
        self.phase_durations = (
            abs(np.deg2rad(THETA1_A_START - THETA1_A_END)) / env.omega1,
            max(
                abs(np.deg2rad(THETA1_A_END - THETA1_B_END)) / env.omega1,
                abs(np.deg2rad(THETA2_A - THETA2_C)) / env.omega2,
            ),
            abs(np.deg2rad(THETA1_B_END - THETA1_C_END)) / env.omega1,
        )
        self.duration = sum(self.phase_durations)

    def get_action(self, sim_time):
        """Return targets in radians, converting world XYZ rotation to gimbal hinges."""
        path_progress = self._path_progress_at_time(sim_time)
        shoulder_x_deg, elbow_deg, abdomen_z_deg = get_path1_angles(path_progress)
        shoulder_y_deg = -20.0
        world_rotation = self._rotation_xyz(
            np.deg2rad(shoulder_x_deg), np.deg2rad(shoulder_y_deg), 0.0
        ) @ self._reference_torso_rotation
        torso_rotation = np.array(
            self.env.data.xmat[self.env.model.body("torso").id], dtype=float
        ).reshape(3, 3)
        local_rotation = torso_rotation.T @ world_rotation
        shoulder_x, shoulder_y, shoulder_z = self._euler_xyz(local_rotation)

        return {
            self.actuator_ids[SHOULDER_JOINT]: shoulder_x,
            self.actuator_ids[SHOULDER_Y_JOINT]: shoulder_y,
            self.actuator_ids[SHOULDER_Z_JOINT]: shoulder_z,
            self.actuator_ids[ELBOW_JOINT]: float(np.deg2rad(elbow_deg)),
            self.actuator_ids[ABDOMEN_Z_JOINT]: float(np.deg2rad(abdomen_z_deg)),
        }

    @staticmethod
    def _rotation_xyz(x, y, z):
        """Build extrinsic world XYZ rotation as Rz @ Ry @ Rx."""
        cx, sx = np.cos(x), np.sin(x)
        cy, sy = np.cos(y), np.sin(y)
        cz, sz = np.cos(z), np.sin(z)
        return np.array(
            [
                [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                [-sy, cy * sx, cy * cx],
            ]
        )

    @staticmethod
    def _euler_xyz(rotation):
        """Extract X, Y, Z angles from Rz @ Ry @ Rx, in radians."""
        y = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
        if abs(np.cos(y)) > 1e-8:
            x = np.arctan2(rotation[2, 1], rotation[2, 2])
            z = np.arctan2(rotation[1, 0], rotation[0, 0])
        else:
            # At Euler gimbal lock, keep Z zero and use X for the remaining rotation.
            x = np.arctan2(np.sin(y) * rotation[1, 2], rotation[1, 1])
            z = 0.0
        return float(x), float(y), float(z)

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
