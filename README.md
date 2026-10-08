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
│   ├── trajectory.py       # time-based joint target equations
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

## Setup and run

Run the following commands from the project root (the directory containing
`requirements.txt`). Use Python 3.11 or newer.

### 1. Create a virtual environment

Linux/macOS:

```bash
python3 -m venv .venv
```

Windows PowerShell or Command Prompt:

```powershell
py -m venv .venv
```

The `.venv` directory is ignored by Git and contains this project's isolated
Python environment.

### 2. Activate the environment

Linux/macOS:

```bash
source .venv/bin/activate
```

Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

Windows Command Prompt:

```bat
.venv\Scripts\activate.bat
```

When activation succeeds, the environment name (usually `.venv`) appears in
your terminal prompt. Activate it again whenever you open a new terminal for
this project.

### 3. Install the dependencies

With `.venv` activated, install the packages listed in `requirements.txt`:

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### 4. Run the simulation

Still from the project root, start the default physics simulation:

```bash
python src/main.py
```

To run without physics stepping, with the selected joint positions set directly:

```bash
python src/main.py --mode kinematics
```

To explicitly select physics mode:

```bash
python src/main.py --mode physics
```

Close the virtual environment when you are finished:

```bash
deactivate
```

Physics mode uses the simulation clock, position actuators, and MuJoCo physics
stepping. Kinematics mode writes the selected joint positions directly, updates
the displayed pose with `mj_forward`, and advances the simulation clock without
stepping physics.

Path 1 is currently inactive. The active example trajectory in
`src/trajectory.py` commands only `shoulder1_right` using the algebraic
function:

```text
theta1(t) = 30 degrees/second * t - 90 degrees
```

It starts at −90° at `t=0`, reaches +90° at `t=6` seconds (a 180° turn), then
holds that endpoint. The angle is about the local X axis of the
`shoulder1_right` hinge; it is not converted to a world-X orientation. Change
`START_ANGLE_DEGREES`, `END_ANGLE_DEGREES`, or
`ANGULAR_SPEED_DEGREES_PER_SECOND` in `src/trajectory.py` to adjust this
example. `get_joint_targets(time_seconds)` returns joint-name targets in
radians for every trajectory registered in `JOINT_TRAJECTORIES`. To add a
second joint, define another function of time that returns degrees, then add
the function under that joint's MuJoCo name:

```python
def elbow_trajectory(time_seconds):
    return 45.0  # Hold the elbow at 45 degrees.


JOINT_TRAJECTORIES = {
    SHOULDER_JOINT: trajectory_1,
    "elbow_right": elbow_trajectory,
}
```

The existing shoulder equation is evaluated independently, so registering an
elbow trajectory does not change its shoulder targets. The controller maps
every registered joint name to its actuator; each name must therefore have a
matching actuator in `assets/humanoid.xml`.

Trajectory and actuator responsibilities are separated: `trajectory.py`
calculates the desired joint angle from time, while `controllers.py` maps that
joint name to its MuJoCo actuator ID. The main loop passes the same targets to
either physics mode or kinematics mode. Backspace resets simulation time, so
the trajectory starts over. The green marker shows the predicted tip position
for the commanded targets.

The XML position actuators use proportional position feedback (`kp`) and
velocity damping (`kv`); this is PD-style actuation, not a full PID controller
with integral action. In physics mode, these settings control how closely a
joint tracks its target. Kinematics mode sets the target angle directly and
does not test physical tracking.
The club is defined separately in `assets/club.xml` and included beneath the
wrist body. It has no joint, so it remains rigidly attached to the wrist.

Only `shoulder1_right` is driven by this example; the viewer keeps its normal
interactive camera controls. The other shoulder axes, elbow, torso, legs, and
left arm remain at their initial positions. In physics mode, contact forces
can make the actual club tip lag the target marker. Playback stops at the
final pose and does not restart automatically. Press Backspace in the viewer
to reset and replay the swing without closing it.

The right-shoulder hinge ranges in `assets/humanoid.xml` are −360° to 360°; the
elbow range is 0° to 180°.

The `dir_kinematics` and `inv_kinematics` functions in `src/kinematics.py`
provide standalone planar two-link calculations. The current swing controller
does not use them; it follows the actual shoulder hinge geometry in MuJoCo.

If a legacy script is still being used, `src/Lab1.py` remains a compatible entry point.

## Notes

The MuJoCo model is composed through `assets/main.xml` using `<include>` blocks.
This keeps the scene, humanoid, and club definitions separate while loading a
single model at runtime.
