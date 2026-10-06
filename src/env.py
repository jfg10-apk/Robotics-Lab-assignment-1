from math import pi
from pathlib import Path

import mujoco


class HumanoidEnv:
    """Wrapper around the MuJoCo model and state used by the simulation."""

    def __init__(self, xml_path=None):
        project_root = Path(__file__).resolve().parents[1]
        model_path = (
            Path(xml_path)
            if xml_path is not None
            else project_root / "assets" / "main.xml"
        )
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self._target_data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        # Locked physics joints are restored to this initial pose each step.
        self.initial_qpos = self.data.qpos.copy()
        self._import_parameters()

    def _import_parameters(self):
        """Read model-derived values used by the controller and simulation loop."""
        self.timestep = float(self.model.opt.timestep)
        self.actuator_ids = {
            self.model.actuator(actuator_id).name: actuator_id
            for actuator_id in range(self.model.nu)
        }
        self.actuator_joint_ids = {
            actuator_id: int(self.model.actuator_trnid[actuator_id, 0])
            for actuator_id in range(self.model.nu)
        }
        self.link_lengths = tuple(
            2.0 * self.model.geom_size[self.model.geom(name).id, 1]
            for name in ("upper_arm_right", "lower_arm_right")
        )
        # MuJoCo hinge positions are radians; phase equations can use the same units.
        elbow_id = self.model.joint("elbow_right").id
        elbow_qpos_address = int(self.model.jnt_qposadr[elbow_id])
        self.elbow_initial_angle = float(self.data.qpos[elbow_qpos_address])

        # These are trajectory speeds (rad/s), not actuator position targets.
        self.omega1 = pi / 2
        self.omega2 = pi / 2

        self.target_marker_mocap_id = int(
            self.model.body("target_marker").mocapid[0]
        )

    def set_target_marker(self, position):
        """Place the visual target marker at a world-space point."""
        self.data.mocap_pos[self.target_marker_mocap_id] = position

    def get_target_site_position(self, action, site_name="ponta_taco"):
        """Compute a site's target position for joint targets without altering live state."""
        # Use scratch data so the live simulation state remains untouched.
        self._target_data.qpos[:] = self.data.qpos
        for actuator_id, target in action.items():
            if not 0 <= actuator_id < self.model.nu:
                raise ValueError(f"Invalid actuator ID: {actuator_id}")

            joint_id = self.actuator_joint_ids[actuator_id]
            joint_type = int(self.model.jnt_type[joint_id])
            if joint_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                raise ValueError(
                    f"Target-pose calculation requires hinge or slide joints; "
                    f"actuator {self.model.actuator(actuator_id).name!r} is "
                    f"attached to {self.model.joint(joint_id).name!r}."
                )
            self._target_data.qpos[self.model.jnt_qposadr[joint_id]] = target

        mujoco.mj_forward(self.model, self._target_data)
        return self._target_data.site(site_name).xpos.copy()

    def step(self, locked_joint_names=()):
        """Step physics, restoring locked joint poses on both sides of the step."""
        locked_joints = []
        for name in locked_joint_names:
            joint_id = self.model.joint(name).id
            joint_type = int(self.model.jnt_type[joint_id])
            qpos_size, dof_size = self._joint_state_sizes(joint_type)
            qpos_address = int(self.model.jnt_qposadr[joint_id])
            dof_address = int(self.model.jnt_dofadr[joint_id])
            locked_joints.append(
                (qpos_address, qpos_size, dof_address, dof_size)
            )

        # Freeze named joints before and after integration to keep the test rig fixed.
        self._restore_locked_joints(locked_joints)
        mujoco.mj_step(self.model, self.data)
        self._restore_locked_joints(locked_joints)
        if locked_joints:
            mujoco.mj_forward(self.model, self.data)

    @staticmethod
    def _joint_state_sizes(joint_type):
        if joint_type == mujoco.mjtJoint.mjJNT_FREE:
            return 7, 6
        if joint_type == mujoco.mjtJoint.mjJNT_BALL:
            return 4, 3
        return 1, 1

    def _restore_locked_joints(self, joints):
        for qpos_address, qpos_size, dof_address, dof_size in joints:
            self.data.qpos[qpos_address : qpos_address + qpos_size] = self.initial_qpos[
                qpos_address : qpos_address + qpos_size
            ]
            self.data.qvel[dof_address : dof_address + dof_size] = 0.0

    def apply_kinematic_action(self, action):
        """Set target hinge/slide positions directly and recompute the pose."""
        for actuator_id, target in action.items():
            if not 0 <= actuator_id < self.model.nu:
                raise ValueError(f"Invalid actuator ID: {actuator_id}")

            joint_id = self.actuator_joint_ids[actuator_id]
            joint_type = int(self.model.jnt_type[joint_id])
            if joint_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                raise ValueError(
                    f"Kinematic mode requires a hinge or slide joint; actuator "
                    f"{self.model.actuator(actuator_id).name!r} is attached to "
                    f"joint {self.model.joint(joint_id).name!r}."
                )

            qpos_address = self.model.jnt_qposadr[joint_id]
            self.data.qpos[qpos_address] = target

        mujoco.mj_forward(self.model, self.data)

    def reset(self):
        """Reset the model to its initial state."""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def advance_kinematic_time(self):
        """Advance the shared trajectory clock without integrating physics."""
        self.data.time += self.timestep

    def get_time(self):
        """Return the simulation time used for trajectory interpolation."""
        return float(self.data.time)