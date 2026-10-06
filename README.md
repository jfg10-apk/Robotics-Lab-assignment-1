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
│   ├── trajectory.py       # fixed geometric paths, parameterized by progress
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
- The controller maps fixed path poses in `src/trajectory.py` to actuator targets.
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
stepping physics.

Path 1 is defined in `src/trajectory.py` as three geometric phases, sampled by
normalized progress rather than simulation time:

- A: shoulder 90°→30°, elbow held at 180° (minimum reach).
- B: shoulder 30°→0°, elbow 180°→0° (the reach increases).
- C: shoulder 0°→−90°, elbow held at 0° (maximum reach).

These pose functions do not read actuator settings, angular velocities, or
simulation time. Edit their angle boundaries to change Path 1 itself. The
controller separately maps elapsed simulation time to phase progress. Its
`omega1` and `omega2` speed settings are initialized to `pi / 2` radians per
second in `HumanoidEnv._import_parameters()` in `src/env.py`; they set the
phase durations, not the geometry. In phase B the longer of the shoulder and
elbow travel times sets the duration, and both joints interpolate over that
shared phase. As a result, changing a speed changes when the pose is reached,
not which poses define the path.

Actuator gains and MuJoCo physics affect how closely the physical arm tracks
these targets; they do not modify the fixed Path 1 definition. Angles in
`trajectory.py` are expressed in degrees and converted to radians by the
controller before sending targets to MuJoCo. The XML position-actuator `kv`
setting is velocity feedback/damping, not a commanded angular speed.

The controller is stateless: each target is calculated from the current
simulation clock, so Backspace resets both the MuJoCo state and trajectory
progress by resetting the clock. The green marker shows the predicted tip
location for the current joint targets.
The club is defined separately in `assets/club.xml` and included beneath the
forearm. It has no joint, so it remains rigidly attached to the arm.

Only the right shoulder and elbow are actuated; the viewer
keeps its normal interactive camera controls. The torso, legs, left arm, and
other joints are not driven. In physics mode, non-swing joints are held at
their initial positions, so contact forces can make the actual club tip lag
the target marker. Playback stops at the final pose and does not restart
automatically. Press Backspace in the viewer to reset and replay the swing
without closing it.

The shoulder joint range in `assets/humanoid.xml` is −90° to 90°, and the elbow
range is 0° to 180°, matching the Path 1 limits.

The `dir_kinematics` and `inv_kinematics` functions in `src/kinematics.py`
provide standalone planar two-link calculations. The current swing controller
does not use them; it follows the actual shoulder hinge geometry in MuJoCo.

If a legacy script is still being used, `src/Lab1.py` remains a compatible entry point.

## Notes

The MuJoCo model is composed through `assets/main.xml` using `<include>` blocks.
This keeps the scene, humanoid, and club definitions separate while loading a
single model at runtime.
