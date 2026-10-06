# Robotics Laboratory 1

This project implements a MuJoCo humanoid simulation with a modular structure to keep the environment, controller logic, and motion trajectory separated for easier maintenance and extension.

## Authors
- Tiago Duamnu
- Francisco Gonçalves
- Gonçalo

## Project structure

```
.
├── assets/
│   ├── main.xml            # top-level MuJoCo file that composes the scene
│   ├── scene.xml           # visual, option, stats and shared assets
│   ├── humanoid.xml        # robot body, joints, contacts and actuators
│   ├── club.xml            # rigid club geometry and orientation
│   ├── human.xml           # legacy model snapshot kept for reference
│   └── ...
├── src/
│   ├── main.py             # entry point for the simulation loop
│   ├── env.py              # MuJoCo environment wrapper
│   ├── controllers.py      # controller mapping motion targets to actuators
│   ├── trajectory.py       # editable per-joint keyframe trajectories
│   ├── kinematics.py       # standalone planar two-link helpers
│   ├── Lab1.py             # backward compatible wrapper
│   └── ...
├── tests/
│   └── ...                 # trajectory, model, and physics tests
├── README.md
└── requirements.txt
```

## Why this structure is easier to develop

- XML resources are separated by responsibility: the `scene` and the `humanoid` model are now in dedicated files.
- The simulation environment is isolated in `env.py` so it can be reused or extended without mixing physics logic with motion planning.
- The environment reads actuator IDs, right-arm link lengths, hinge geometry, and the simulation timestep from the loaded MuJoCo model.
- The controller maps the per-joint keyframes in `src/trajectory.py` to actuator targets.
- `main.py` acts as the entry point and keeps the execution flow readable.

## Run the project

Run from the project root:

```bash
python src/main.py
```

The simulation can run in either of two modes:

```bash
# Run the standard trajectory by directly setting joint positions (no physics stepping).
python src/main.py --mode kinematics

# Run the standard trajectory with MuJoCo physics and position actuators.
python src/main.py --mode physics
```

Physics mode uses the simulation clock, position actuators, and MuJoCo physics
stepping. Kinematics mode writes the selected joint positions directly, updates
the displayed pose with `mj_forward`, and advances the simulation clock without
stepping physics. Both modes run the same two-second shoulder trajectory. To
change the path, edit `JOINT_TRAJECTORIES` in `src/trajectory.py`: each joint
maps to ordered `(time_seconds, angle_degrees)` keyframes. Add intermediate
entries to shape the motion; the controller linearly interpolates between them
and holds the endpoint poses outside the keyframe range. For example:

```python
JOINT_TRAJECTORIES = {
    "shoulder1_right": ((0.0, -55.0), (1.0, -20.0), (2.0, 0.0)),
}
```

The shoulder arc is planned from the actual right-shoulder hinge in
`src/trajectory.py`. The target marker rotates around the shoulder pivot with
the same axis and angle as `shoulder1_right`, so it describes the club-tip path.
That marker matches the club exactly for the current shoulder-only trajectory;
adding trajectories for other joints also requires extending marker kinematics.
The elbow is held at zero and the arm/forearm geometry is aligned as a straight
chain. The club is defined separately in `assets/club.xml` and included beneath
the forearm. It has no joint, so it is rigidly fixed to the arm rather than
rotating under gravity. Its local rotation in `assets/club.xml` controls its
fixed orientation relative to the forearm; `ponta_taco` marks its end.
Only the right shoulder is actuated; the viewer
keeps its normal interactive camera controls. The torso, legs, left arm, and
other joints are not driven. In physics mode, non-swing joints are held at
their initial positions, so contact forces can make the actual club tip lag
the target marker. Playback stops at the final pose and does not restart
automatically. Press Backspace in the viewer to reset and replay the swing
without closing it.

The `dir_kinematics` and `inv_kinematics` functions in `src/kinematics.py`
provide standalone planar two-link calculations. The current swing controller
does not use them; it follows the actual shoulder hinge geometry in MuJoCo.

If a legacy script is still being used, `src/Lab1.py` remains a compatible entry point.

## Notes

The MuJoCo model is composed through `assets/main.xml` using `<include>` blocks.
This keeps the scene, humanoid, and club definitions separate while loading a
single model at runtime.
