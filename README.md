# Robotics Laboratory 1

This project simulates a humanoid golf swing in MuJoCo. It includes a ball supported by a tee, position-controlled joints, live visualization, CSV recording, and independent external-torque perturbation experiments.

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
│   ├── ball.xml            # ball, tee, contacts and ball sensors
│   ├── human.xml           # legacy model snapshot kept for reference
│   └── ...
├── src/
│   ├── main.py             # entry point for the simulation loop
│   ├── env.py              # MuJoCo environment wrapper
│   ├── controllers.py      # controller mapping motion targets to actuators
│   ├── trajectory.py       # joint angle targets for the swing phases
│   ├── Lab1.py             # backward compatible wrapper
│   └── tools/
│       ├── parser.py       # command-line arguments for main.py
│       ├── csv.py          # shared CSV recorder
│       ├── perturbations.py # standalone perturbation experiments
│       ├── grafics.py      # plots all CSV recordings under results/
│       ├── data_points.csv # normal simulation output (generated)
│       └── results/        # perturbation outputs grouped by torque (generated)
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

To explicitly select physics mode:

```bash
python src/main.py --mode physics
```

The MuJoCo window shows the swing and the ball motion. Press **Backspace** to
reset the simulation and replay the swing. Close the window to save the recorded
samples to **`src/tools/data_points.csv`**. Each new execution replaces this file.
Backspace keeps the samples already collected, so a recording with replays can
contain several sequences starting at time zero.

The CSV path is based on the location of `src/tools/csv.py`, so it stays the same
even if the program is launched from a different working directory.

The parser accepts `physics`, `ph`, `kinematics`, and `ki`. Currently all four
values use the physics stepping loop in `main.py`.

Close the virtual environment when you are finished:

```bash
deactivate
```

## Swing trajectory and physics

`src/trajectory.py` defines five target poses. `joint_angle()` linearly
interpolates their angles using `np.interp`.

| Target pose | Simulation time |
| --- | --- |
| Address | 0.00 s |
| Backswing | 4.00 s |
| Before impact | 4.30 s |
| After impact | 4.45 s |
| Follow-through | 4.50 s |

These are trajectory keyframes; the actual club-ball contact time depends on
the simulated motion and ball position.

The trajectory commands the three right-shoulder axes, the right elbow, and
`abdomen_x` and `abdomen_z`. Edit the phase times and the `ADDRESS_*`,
`BACKSWING_*`, `BEFORE_IMPACT_*`, `AFTER_IMPACT_*`, and `FOLLOW_THROUGH_*`
angles to change the swing. Angles are defined in degrees and converted to
radians by `get_joint_targets()`. After the last keyframe, the targets remain
at their final values while physics continues.

`controllers.py` maps those targets to the position actuators. MuJoCo computes
their effort from position feedback (`kp`) and velocity damping (`kv`), so
actual joint angles can deviate from their targets under contact or external
loads. The left arm has no active actuators and follows the motion through the
hand connection and physical constraints.

`assets/main.xml` includes the scene, humanoid, and ball definitions. The club
is included beneath the wrist and remains rigidly attached to it. The ball is
an independent free body supported by a tee; gravity and club contact determine
its flight. Its trajectory is not prescribed directly.

## Perturbation experiments

Run the experiments separately from the normal simulation:

```bash
python src/tools/perturbations.py
```

This opens a MuJoCo viewer for each of four independent experiments:

| Experiment | External torque applied to |
| --- | --- |
| `referencia` | No joint: normal swing |
| `ombro` | `shoulder1_right` |
| `cotovelo` | `elbow_right` |
| `pulso` | `pulso_right1`, controlled by the actuator named `pulso_right` |

Each experiment starts from the initial state and runs for **6 seconds of
simulation time**. The window closes automatically at the end and the next
experiment starts. Closing a window early saves the samples collected and
stops the remaining experiments. `main.py` runs the normal swing only;
it does not launch these experiments.

Choose the external torque in N·m with `--torque`:

```bash
python src/tools/perturbations.py --torque 2
python src/tools/perturbations.py --torque -2
```

To run only one experiment:

```bash
python src/tools/perturbations.py --case pulso --torque 2
```

`--case` accepts `referencia`, `ombro`, `cotovelo`, or `pulso`. Without
`--torque`, the script uses `TORQUE_NM` in `src/tools/perturbations.py`.
The other experiment settings are also at the top of that file:

| Setting | Meaning | Current value |
| --- | --- | --- |
| `START_S` | Start of the external torque | 4.35 s |
| `END_S` | End of the external torque, excluded | 4.50 s |
| `DURATION_S` | Total simulation duration per experiment | 6.00 s |

The torque is applied through `qfrc_applied` on the selected joint, in addition
to the normal position actuators. Positive and negative values apply opposite
directions about that joint's axis. A negative torque can assist or oppose the
motion depending on the direction of rotation at that instant.

### Result files

Outputs are stored in `src/tools/results/`, in a folder named after the torque.
For example, `--torque 2` produces:

```text
src/tools/results/torque_2Nm/
├── referencia.csv
├── ombro.csv
├── cotovelo.csv
└── pulso.csv
```

A negative value produces a separate folder, such as `torque_-2Nm/`.
Running the same torque again replaces the CSV files for the experiments
executed. Selecting one `--case` replaces only that experiment's file; any
other existing files in the folder remain from their previous executions.
The terminal prints the output path and identifies each experiment.

## CSV contents and comparison

Both `main.py` and `perturbations.py` use `src/tools/csv.py`. The recorder
collects samples in memory during the simulation and writes them when the
recording finishes. The files use commas as separators and can be opened in
Excel or another spreadsheet program. If the columns are not separated
automatically, import the file using the comma delimiter.

| Columns | Recorded quantity |
| --- | --- |
| `Time (s)` | Simulation time |
| `Ball Pos X/Y/Z (m)` | Ball position in world coordinates, one column per axis |
| `Ball Vel X/Y/Z (m/s)` | Ball velocity in world coordinates, one column per axis |
| `Ball Speed (m/s)` | Norm of the ball velocity |
| `Current actuator max effort (N*m)` | Maximum absolute actuator effort at that instant |
| Joint names | Individual actuator efforts, one column per actuated joint |

Individual efforts come from `env.data.actuator_force`; the joint labels follow
the actuator order. The current model has 10 actuators, giving **19 columns**
in total. These columns contain actuator effort, rather than joint angles or
the external perturbation torque. The experiment name and folder identify
the perturbed joint and selected torque.

Time is rounded to three decimal places and other values to two. With the
current 0.005 s timestep, a complete perturbation experiment records **1201
samples**, including times 0 and 6 s. The normal simulation records until its
window is closed.

Compare each perturbed CSV with `referencia.csv` from the same torque folder
at matching times. Plot ball position, ball speed, or actuator effort against
time to see the effect of the disturbance. The ball position columns describe
its flight path; differences from the reference show how that path changed.
The maximum-effort column is an instantaneous value; take its maximum over
the recording to obtain the peak effort of the whole experiment.

## Generate graphs

After recording the experiments, run:

```bash
python src/tools/grafics.py
```

The script reads all CSV files recursively under `src/tools/results/` and
groups them by experiment. It saves one PNG image per recorded parameter
directly in `src/tools/`. Each image contains panels for the shoulder, elbow,
and wrist experiments, plotting that parameter against simulation time.

Each panel includes the torque runs and the normal swing, shown as a **black
dashed reference curve**. With results for -3, -1, +1, and +3 N·m, this gives
**five curves per panel**. Identical `referencia.csv` recordings are shown as
one reference; if reference recordings differ, each distinct reference is
retained and identified by its folder. The legend also identifies the torque
folders, such as `torque_-3Nm` and `torque_3Nm`.

The current 19-column CSV format produces **18 PNG images** (time is the
horizontal axis):

- `ball_pos_x.png`, `ball_pos_y.png`, and `ball_pos_z.png`.
- `ball_vel_x.png`, `ball_vel_y.png`, and `ball_vel_z.png`.
- `ball_speed.png` and `max_actuator_effort.png`.
- `actuator_<joint_name>.png` for each of the 10 actuators.

All graph titles are in English. Each execution replaces the PNG files with
graphs of the currently available results. Empty or invalid CSV files are
reported in the terminal and skipped. A reference recording is required for
comparison. The script reads the recordings and generates images without
running another simulation.
