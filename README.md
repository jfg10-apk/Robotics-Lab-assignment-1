# Robotics Laboratory 1

MuJoCo golf swing simulation with CSV recording and perturbation experiments.

## Authors

- Tiago Duamnu
- Francisco Gonçalves
- Gonçalo

## Installation

Use Python 3.11 or newer. Run all commands from the project root, where
`requirements.txt` is located.

### 1. Create a virtual environment

Windows:

```powershell
py -m venv .venv
```

Linux/macOS:

```bash
python3 -m venv .venv
```

### 2. Activate the environment

Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

Windows Command Prompt:

```bat
.venv\Scripts\activate.bat
```

Linux/macOS:

```bash
source .venv/bin/activate
```

### 3. Install the dependencies

```bash
python -m pip install -r requirements.txt
```

Activate the environment again whenever you open a new terminal.
On macOS, use `mjpython` instead of `python` for the simulation and
perturbation commands that open the MuJoCo viewer.

## Run the normal simulation

```bash
python src/main.py
```

The MuJoCo window opens and shows the swing. Press **Backspace** to reset and
replay it. Close the window to save the recording to:

```text
src/tools/data_points.csv
```

A new execution replaces this CSV. Replays within the same execution are
recorded in the same file.

## Run the perturbation experiments

Specify the torque in N·m:

```bash
python src/tools/perturbations.py --torque 5
```

The script shows four experiments in sequence: reference, shoulder, elbow,
and wrist. Each runs for 6 seconds of simulation time. Closing the window
early saves the collected samples and stops the remaining experiments.

For a negative torque:

```bash
python src/tools/perturbations.py --torque -5
```

To run only one experiment:

```bash
python src/tools/perturbations.py --case pulso --torque 5
```

`--case` accepts `referencia`, `ombro`, `cotovelo`, or `pulso`.
Without `--torque`, the script uses `TORQUE_NM` at the top of
`src/tools/perturbations.py`.

The CSV files are saved in a folder named after the torque, for example:

```text
src/tools/results/torque_5Nm/
├── referencia.csv
├── ombro.csv
├── cotovelo.csv
└── pulso.csv
```

Negative values use folders such as `torque_-5Nm/`. Repeating the same torque
replaces the CSV files for the experiments executed. With `--case`, only that
experiment's CSV is replaced.

To record four torque values for comparison:

```bash
python src/tools/perturbations.py --torque -5
python src/tools/perturbations.py --torque -1
python src/tools/perturbations.py --torque 1
python src/tools/perturbations.py --torque 5
```

## Generate the graphs

After recording the perturbation experiments, run:

```bash
python src/tools/grafics.py
```

The script reads the CSV files under `src/tools/results/`. Include a
`referencia.csv` recording so it can compare the experiments with the normal
swing. The PNG images are saved in:

```text
src/tools/grafics/
```

For example, `ball_speed.png` and `actuator_elbow_right.png` are generated in
this folder. Run the command again after creating new results to update the
images. Existing images with the same names are replaced.

The output folders are created automatically. The `results/` folders and PNG
images are ignored by Git and are generated locally using the commands above.

## Finish

To deactivate the virtual environment:

```bash
deactivate
```
