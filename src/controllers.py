"""Map joint-name targets onto the MuJoCo actuator controls."""

from trajectory import get_joint_targets
import numpy as np

class SwingController:
    """Translate trajectory joint targets into MuJoCo actuator-index targets."""

    def __init__(self, env):
        self.actuator_ids = env.actuator_ids
        self.joint_names = tuple(get_joint_targets(0.0))

        missing_actuators = set(self.joint_names) - self.actuator_ids.keys()
        if missing_actuators:
            raise ValueError(
                "No actuator is configured for trajectory joint(s): "
                + ", ".join(sorted(missing_actuators))
            )

    def get_action(self, sim_t: float) -> dict[int, float]:
        """Return actuator-index targets in radians for the current time."""

        if not np.isfinite(sim_t) or np.isnan(sim_t):            
            raise ValueError("Sim_time must not be infinite or NaN.")

        jnt_actuation = get_joint_targets(sim_t)
        return {
            self.actuator_ids[jnt_str]: jnt_id
            for jnt_str, jnt_id in jnt_actuation.items()
        }
