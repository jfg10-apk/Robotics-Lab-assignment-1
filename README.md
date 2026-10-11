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
│   ├── configuration.py    # TOML trial and evaluation-case loading
│   ├── kinematics.py       # standalone planar two-link helpers
│   ├── Lab1.py             # backward compatible wrapper
│   └── ...
├── trials/
│   └── trial_1.toml        # trajectory, PD gains, torque limits and cases
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

Select the trial TOML and one of its named cases with `--trial` and `--case`:

```bash
python src/main.py --mode physics --trial trials/trial_1.toml --case baseline
```

Run the region-dependent angle-and-speed torque experiment with:

```bash
python src/main.py --mode physics --trial trials/trial_3.toml --case baseline
```

Close the virtual environment when you are finished:

```bash
deactivate
```

Physics mode advances MuJoCo with torque motors. Trial 1 uses PD torque
tracking to follow position references; trial 2 applies a direct torque pulse
to the right shoulder; trial 3 uses angle and speed feedback with regional parameters.
Kinematics mode writes selected joint positions directly, updates the displayed
pose with `mj_forward`, and advances the clock without stepping physics.
Torque-controlled trials require physics mode.

`trials/trial_1.toml` defines phase times and angle keyframes in degrees for
the six driven joints. The trajectory code interpolates those values and
converts them to radians. `aim_yaw_deg` is applied as an offset about the
local `shoulder3_right` hinge axis; it is not a guaranteed world-space launch
angle. `trials/trial_2.toml` contains only a direct torque profile for
`shoulder1_right`; all other humanoid joints are held at their initial pose.

`trials/trial_3.toml` repeats trial 1's angle keyframes. Its position
resistance is the proportional torque gain on angle error. Its speed-control
gain tracks a target speed derived from the slope of the angle keyframes; no
separate speed trajectory is configured. Speed resistance is passive MuJoCo
joint damping, and regional torque scales change the per-joint torque ceiling.
These are distinct feedback and physical-resistance parameters.

In physics mode, the XML defines torque motors with unit gear. The controller
uses a PD tracking law to turn each position reference into a torque command:

```text
torque = kp * (target_position - measured_position)
       + kv * (target_velocity - measured_velocity)
```

`kp` and `kv` are read from the trial TOML, and `torque_limit_nm` bounds each
configured motor. `default_torque_limit_nm` bounds motors without an explicit
entry. A named evaluation case can scale these limits and add a local shoulder
offset. This is torque-limited PD trajectory tracking, not a full inverse-
dynamics or human muscle model. The position reference is retained; the
physics mode no longer sends it directly as the actuator command.

Trial 2 instead defines torque keyframes in N m. The controller interpolates
between them and commands zero after the last profile time, so the arm receives
one torque pulse and then swings under gravity, inertia, and passive joint
damping rather than being driven by a repeating sinusoid.

Kinematics mode applies the position references directly and does not test
torque tracking or physical impact. Backspace resets simulation time. The
green marker shows the predicted club-tip position for the reference pose.
The club is defined separately in `assets/club.xml` and remains rigidly
attached to the wrist.

The `dir_kinematics` and `inv_kinematics` functions in `src/kinematics.py`
provide standalone planar two-link calculations. The swing controller does
not use them; it follows the joint geometry in MuJoCo.

If a legacy script is still being used, `src/Lab1.py` remains a compatible entry point.

## Notes

The MuJoCo model is composed through `assets/main.xml` using `<include>` blocks.
This keeps the scene, humanoid, and club definitions separate while loading a
single model at runtime.
