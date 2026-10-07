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

Path 1 is defined in `src/trajectory.py` as three geometric phases, sampled by
normalized progress rather than simulation time:

- A: shoulder world-X rotation −90°→−30°, elbow held at 180° (minimum reach).
- B: shoulder world-X rotation −30°→0°, elbow 180°→0° (the reach increases).
- C: shoulder world-X rotation 0°→90°, elbow held at 0° (maximum reach).

These pose functions do not read actuator settings, angular velocities, or
simulation time. Edit their angle boundaries to change Path 1 itself. The
controller separately maps elapsed simulation time to phase progress. The
right shoulder is a three-hinge gimbal nested in Z-Y-X order around one pivot.
The controller treats the trajectory's shoulder angle as a world-referenced X
rotation and converts the requested XYZ orientation to the corresponding local
hinge targets. The XML shoulder range is ±360° to permit full turns. The
gimbal's physical hinge axes still move with the rotating links; only the
rotation inputs use the fixed world XYZ reference. World XYZ rotations use a
defined order, so combining rotations about multiple axes is order-dependent.
As with any Euler-angle representation, the conversion has a singular
configuration at certain combined rotations. The current Path 1 varies only
the world-X input. The `omega1` and `omega2` speed settings are initialized to
`pi / 2` radians per
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

Trajectory progress is calculated from the current simulation clock, so
Backspace resets both the MuJoCo state and trajectory progress. The green
marker shows the predicted tip location for the current joint targets.
The club is defined separately in `assets/club.xml` and included beneath the
wrist body. It has no joint, so it remains rigidly attached to the wrist.

Only the three right-shoulder hinges and elbow are actuated by the swing
controller; the viewer keeps its normal interactive camera controls. The
torso, legs, left arm, and other joints are not driven. In physics mode,
non-swing joints are held at
their initial positions, so contact forces can make the actual club tip lag
the target marker. Playback stops at the final pose and does not restart
automatically. Press Backspace in the viewer to reset and replay the swing
without closing it.

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
