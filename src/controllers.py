"""Track joint position references using explicitly commanded motor torque."""

import mujoco
import numpy as np
from configuration import load_trial, select_case
from trajectory import get_joint_targets, get_joint_velocity_targets

class SwingController:
    """Use PD feedback to track positions with torque-limited motor actuators."""

    def __init__(self, env, trial=None, case=None):
        trial = trial or load_trial()
        self.env = env
        self.trajectory = trial["trajectory"]
        self.actuation = trial["actuation"]
        self.direct_torque_profiles = self.actuation.get("torque_profiles_nm", {})
        self.feedback_mode = self.actuation.get("feedback_mode", "pd")
        self.regions = self.actuation.get("regions", [])
        self.requires_physics = bool(
            self.direct_torque_profiles
            or self.feedback_mode in ("angle_only", "angle_speed")
        )
        self.case = case or select_case(trial, "baseline")
        self.aim_offset = float(self.trajectory.get("aim_yaw_deg", 0.0))
        self.aim_offset += float(self.case.get("aim_yaw_deg", 0.0))
        self.joint_offsets = {"shoulder3_right": self.aim_offset}
        self.actuator_ids = env.actuator_ids
        position_joint_names = set(
            get_joint_targets(0.0, self.trajectory, self.joint_offsets)
        )
        self.joint_names = tuple(sorted(
            position_joint_names | set(self.direct_torque_profiles)
        ))

        missing_actuators = set(self.joint_names) - self.actuator_ids.keys()
        if missing_actuators:
            raise ValueError(
                "No actuator is configured for trajectory joint(s): "
                + ", ".join(sorted(missing_actuators))
            )

        self.actuator_joints = {}
        self.initial_targets = {}
        for actuator_id in range(env.model.nu):
            joint_id = env.actuator_joint_ids[actuator_id]
            joint_name = env.model.joint(joint_id).name
            joint_type = int(env.model.jnt_type[joint_id])
            if joint_type not in (
                mujoco.mjtJoint.mjJNT_HINGE,
                mujoco.mjtJoint.mjJNT_SLIDE,
            ):
                raise ValueError(
                    f"Torque position tracking requires hinge or slide joints; "
                    f"actuator {env.model.actuator(actuator_id).name!r} is "
                    f"attached to {joint_name!r}."
                )
            qpos_address = int(env.model.jnt_qposadr[joint_id])
            dof_address = int(env.model.jnt_dofadr[joint_id])
            self.actuator_joints[actuator_id] = (
                joint_name, qpos_address, dof_address
            )
            self.initial_targets[joint_name] = float(env.initial_qpos[qpos_address])

        env.set_actuator_torque_limits(
            self.actuation.get("torque_limit_nm", {}),
            self.actuation.get("default_torque_limit_nm", 50.0),
            float(self.case.get("torque_scale", 1.0)),
        )

    def region_at(self, sim_t: float) -> dict:
        for region in self.regions:
            if sim_t < region["until_s"]:
                return region
        return self.regions[-1] if self.regions else {}

    def apply_region(self, sim_t: float) -> None:
        """Apply regional passive damping before the physics step."""
        if self.feedback_mode in ("angle_only", "angle_speed") and self.regions:
            region = self.region_at(sim_t)
            self.env.set_joint_speed_resistance(
                self.joint_names,
                float(region["speed_resistance_nm_s_per_rad"]),
            )

    def get_position_targets(self, sim_t: float) -> dict[int, float]:
        """Return actuator-index position references in radians."""
        if not np.isfinite(sim_t):
            raise ValueError("Sim_time must not be infinite or NaN.")
        trajectory_targets = get_joint_targets(
            sim_t, self.trajectory, self.joint_offsets
        )
        return {
            actuator_id: trajectory_targets.get(
                joint_name, self.initial_targets[joint_name]
            )
            for actuator_id, (joint_name, _, _) in self.actuator_joints.items()
        }

    def get_torque_action(self, sim_t: float) -> dict[int, float]:
        """Return saturated PD torque commands in N m for each motor."""
        positions = self.get_position_targets(sim_t)
        trajectory_velocities = (
            get_joint_velocity_targets(sim_t, self.trajectory)
            if self.feedback_mode in ("pd", "angle_speed")
            else {}
        )
        region = self.region_at(sim_t)
        kp = float(self.actuation.get("kp", 150.0))
        kv = float(self.actuation.get("kv", 10.0))
        torque_limits = self.actuation.get("torque_limit_nm", {})
        default_limit = float(
            self.actuation.get("default_torque_limit_nm", 50.0)
        )
        scale = float(self.case.get("torque_scale", 1.0))

        torques = {}
        for actuator_id, target in positions.items():
            joint_name, qpos_address, dof_address = self.actuator_joints[actuator_id]
            profile = self.direct_torque_profiles.get(joint_name)
            if profile is not None:
                if profile["type"] == "sinusoidal":
                    angular_frequency = 2.0 * np.pi / profile["period_s"]
                    phase = np.deg2rad(profile.get("phase_deg", 0.0))
                    torque = profile["amplitude_nm"] * np.sin(
                        angular_frequency * sim_t + phase
                    )
                else:
                    times = profile["times_s"]
                    values = profile["torque_nm"]
                    torque = (
                        float(np.interp(sim_t, times, values))
                        if sim_t < times[-1]
                        else 0.0
                    )
                limit = (
                    float(torque_limits.get(joint_name, default_limit))
                    * scale
                    * float(region.get("torque_scale", 1.0))
                )
                torques[actuator_id] = float(np.clip(torque, -limit, limit))
                continue

            position_error = target - self.env.data.qpos[qpos_address]
            limit = (
                float(torque_limits.get(joint_name, default_limit))
                * scale
                * float(region.get("torque_scale", 1.0))
            )
            if self.feedback_mode in ("angle_only", "angle_speed"):
                position_resistance = float(
                    region.get("position_resistance_nm_per_rad", kp)
                )
                torque = position_resistance * position_error
                if self.feedback_mode == "angle_speed":
                    velocity_target = trajectory_velocities.get(joint_name, 0.0)
                    velocity_error = (
                        velocity_target - self.env.data.qvel[dof_address]
                    )
                    speed_control = float(
                        region.get("speed_control_nm_s_per_rad", 0.0)
                    )
                    torque += speed_control * velocity_error
                torques[actuator_id] = float(np.clip(torque, -limit, limit))
                continue

            velocity_target = trajectory_velocities.get(joint_name, 0.0)
            velocity_error = velocity_target - self.env.data.qvel[dof_address]
            torques[actuator_id] = float(np.clip(
                kp * position_error + kv * velocity_error,
                -limit,
                limit,
            ))
        return torques

    def get_action(self, sim_t: float) -> dict[int, float]:
        """Compatibility alias for the physics torque command."""
        return self.get_torque_action(sim_t)
