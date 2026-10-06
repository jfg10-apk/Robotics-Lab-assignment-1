from trajectory import (
    JOINT_TRAJECTORIES,
    SHOULDER_JOINT,
    get_target_angle,
    get_target_point,
)


class SwingController:
    """Map the single-joint swing trajectory onto the right-arm actuator."""

    def __init__(self, env):
        self.actuator_ids = env.actuator_ids
        self.env = env
        self.joint_names = tuple(JOINT_TRAJECTORIES)

    def get_action(self, sim_time):
        """Map configured joint trajectories to MuJoCo actuator targets."""
        return {
            self.actuator_ids[joint_name]: get_target_angle(sim_time, keyframes)
            for joint_name, keyframes in JOINT_TRAJECTORIES.items()
        }

    def get_target_point(self, sim_time):
        """Return the shoulder-only circular target shown by the marker."""
        # The marker's circle currently assumes every other joint stays locked.
        return get_target_point(
            sim_time,
            self.env.shoulder_pivot,
            self.env.shoulder_axis,
            self.env.initial_club_tip,
            self.env.shoulder_initial_angle,
            JOINT_TRAJECTORIES[SHOULDER_JOINT],
        )
