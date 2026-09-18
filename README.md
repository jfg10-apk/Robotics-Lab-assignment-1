Robotics Laboratory 1:

Authors:
- Tiago Duamnu
- Francisco Gonçalves
- Gonçalo

MuJoCo Golf Swing Imitation

A Python project using MuJoCo to simulate a humanoid golf player and reproduce a target golf swing while holding a golf club.

The project focuses on motion imitation and physics simulation. A target golf swing is converted into target humanoid joint movements, and a controller drives the MuJoCo humanoid to reproduce the motion.

The project uses uv for Python environment and dependency management.

Project Overview

The simulation consists of:

A humanoid golf player

A golf club attached to the player's hands

Target golf-swing motion data

Motion retargeting

A controller for following the target motion

MuJoCo for physics simulation and visualization

The general pipeline is:

Target Golf Swing
       │
       ▼
 Motion Retargeting
       │
       ▼
Target Joint Positions
       │
       ▼
 Motion Controller
       │
       ▼
     MuJoCo
       │
       ▼
Humanoid + Golf Club
       │
       ▼
   Golf Swing


This project does not require reinforcement learning. The humanoid directly follows a predefined target motion.

Requirements

Python 3.10+

uv

Git (optional)

MuJoCo is installed as a Python dependency and does not need to be installed separately for the normal Python workflow.

Project Structure

A recommended project structure is:

golf-swing-mujoco/
│
├── models/
│   ├── humanoid.xml
│   ├── golf_club.xml
│   └── ...
│
├── data/
│   └── target_swing/
│       └── swing.csv
│
├── src/
│   ├── main.py
│   ├── controller.py
│   ├── motion.py
│   └── ...
│
├── pyproject.toml
├── uv.lock
├── README.md
└── .gitignore


The exact structure can be changed depending on the implementation.

Installing uv
Linux

The recommended installation method is:

curl -LsSf https://astral.sh/uv/install.sh | sh


Restart your terminal or reload your shell configuration.

Verify the installation:

uv --version


If uv is not found, make sure the directory containing the uv executable is in your PATH.

Windows

Open PowerShell and run:

irm https://astral.sh/uv/install.ps1 | iex


Restart PowerShell after installation.

Verify:

uv --version

Getting the Project

If the project is hosted on Git:

git clone <REPOSITORY_URL>
cd <PROJECT_DIRECTORY>


If you already have the project, simply navigate to its directory:

Linux
cd <PROJECT_DIRECTORY>

Windows
cd <PROJECT_DIRECTORY>

Setting Up the Environment

uv automatically manages the project's virtual environment.

You do not need to manually run:

python -m venv .venv


and you do not need to manually activate the environment for normal uv run commands.

Linux

From the project directory:

uv sync

Windows

From the project directory:

uv sync


The command:

uv sync


will:

Create .venv/ if it does not exist.

Install the Python version required by the project.

Install the dependencies from pyproject.toml.

Use uv.lock to reproduce the locked dependency versions.

Running the Program

The recommended way to run the project is with uv run.

Linux
uv run python src/main.py

Windows
uv run python src\main.py


You do not need to manually activate .venv when using uv run.

uv automatically runs the command inside the project's managed environment.

Activating the Virtual Environment

You can activate .venv manually if you want to work interactively inside the environment.

Linux
source .venv/bin/activate


You should see something similar to:

(.venv) user@computer:~/golf-swing-mujoco$


Then you can run:

python src/main.py


When finished:

deactivate

Windows PowerShell
.\.venv\Scripts\Activate.ps1


You should see:

(.venv) PS C:\golf-swing-mujoco>


Then:

python src\main.py


When finished:

deactivate

Recommended approach

For most development, you can simply use:

uv run python src/main.py


instead of manually activating the environment.

Installing Dependencies

Project dependencies should be defined in pyproject.toml.

For example:

[project]
name = "golf-swing-mujoco"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = [
    "mujoco>=3.3,<4",
    "numpy>=1.26,<3",
    "scipy>=1.13,<2",
    "matplotlib>=3.9,<4",
    "imageio>=2.34,<3",
    "pyyaml>=6.0,<7",
    "tqdm>=4.66,<5",
]


After changing pyproject.toml, run:

uv sync


This updates the environment and uv.lock.

Adding a New Dependency

Instead of manually editing pyproject.toml, you can use uv.

For example, to add pandas:

uv add pandas


This will:

Add the dependency to pyproject.toml

Resolve compatible versions

Update uv.lock

Install the package

For a development dependency:

uv add --dev pytest

Updating Dependencies

To update dependencies according to the project's dependency requirements:

uv lock --upgrade


Then synchronize the environment:

uv sync


Alternatively:

uv sync --upgrade


Use this when you want uv to update the project's dependencies and synchronize the environment.

Removing a Dependency

For example:

uv remove matplotlib


uv will remove the dependency from the project and update the lock file.

Checking the Environment

You can check the Python version used by the project:

uv run python --version


Check the installed MuJoCo version:

uv run python -c "import mujoco; print('MuJoCo:', mujoco.__version__)"


Check the main dependencies:

uv run python -c "import numpy, scipy, matplotlib; print('Dependencies OK')"

Python Version

The Python version can be specified in pyproject.toml.

For example:

[project]
requires-python = ">=3.10"


If the project requires a specific Python version, uv can manage it for you.

For example:

uv python install 3.12


Then create/use the environment with:

uv venv --python 3.12


Normally, however, uv sync is sufficient when the project configuration already specifies the required Python version.

Target Golf Swing

The target swing can be represented as a time sequence of humanoid joint positions.

For example:

Time        Joint 1    Joint 2    Joint 3    ...
------------------------------------------------
0.00 s      ...        ...        ...        ...
0.01 s      ...        ...        ...        ...
0.02 s      ...        ...        ...        ...
...


The target motion can be stored in:

CSV

JSON

NumPy arrays

Motion-capture formats

For example:

data/
└── target_swing/
    └── swing.csv


The motion-processing code converts the target motion into joint targets that can be used by the MuJoCo controller.

Golf Swing Control

The humanoid follows the target golf swing using a motion controller.

A simplified control loop is:

Target Pose
     │
     ▼
Current Humanoid Pose
     │
     ▼
Calculate Error
     │
     ▼
Controller
     │
     ▼
Joint Torques
     │
     ▼
MuJoCo


A PD controller can, for example, calculate torque as:

torque =
    Kp * (target_position - current_position)
    - Kd * current_velocity


where:

Kp controls how strongly the humanoid follows the target position.

Kd provides damping.

target_position is the desired joint position.

current_position is the current joint position.

current_velocity is the current joint velocity.

The controller ultimately produces the forces/torques required to reproduce the golf swing.

Golf Club

The golf club is part of the MuJoCo model and should be connected to the humanoid's hands.

A typical model hierarchy is:

Humanoid
   │
   ├── Left Arm
   │      └── Left Hand
   │
   └── Right Arm
          └── Right Hand
                 │
                 ▼
             Golf Club
                 │
                 ▼
             Club Head


The golf club's position, orientation, and velocity are important when evaluating the resulting swing.

Simulation Loop

The main simulation follows this general process:

1. Load MuJoCo model
2. Load target golf swing
3. Initialize humanoid
4. Read current humanoid state
5. Determine target pose
6. Calculate control input
7. Apply joint torques
8. Advance MuJoCo simulation
9. Render humanoid and golf club
10. Repeat until the swing is complete

Running Tests

If tests are included in the project, run them with:

uv run pytest


If pytest has not yet been added:

uv add --dev pytest

Formatting and Linting

If the project uses Ruff, install it as a development dependency:

uv add --dev ruff


Run the linter:

uv run ruff check .


Format the project:

uv run ruff format .

Git and uv

The following files should normally be committed:

pyproject.toml
uv.lock
README.md
.gitignore
src/
models/
data/


The .venv/ directory should not be committed.

uv.lock should normally be committed because it ensures that other developers can reproduce the same dependency versions.

Updating the Project From Git

After pulling changes from the repository:

git pull
uv sync


Then run:

uv run python src/main.py


If pyproject.toml or uv.lock changed, uv sync will update the local environment accordingly.

Clean Environment

If the virtual environment becomes corrupted, you can remove it and recreate it.

Linux
rm -rf .venv
uv sync

Windows PowerShell
Remove-Item -Recurse -Force .venv
uv sync


Then run the project normally:

Linux
uv run python src/main.py

Windows
uv run python src\main.py

Quick Start

For someone who has already installed uv:

Linux
git clone <REPOSITORY_URL>
cd <PROJECT_DIRECTORY>

uv sync
uv run python src/main.py

Windows
git clone <REPOSITORY_URL>
cd <PROJECT_DIRECTORY>

uv sync
uv run python src\main.py


That's all that is required for the normal setup.

Reinforcement Learning

Reinforcement learning is not required for the current implementation.

The current objective is to imitate a predefined golf swing:

Target Swing
      ↓
Motion Retargeting
      ↓
Target Joint Positions
      ↓
Motion Controller
      ↓
MuJoCo
      ↓
Humanoid + Golf Club


Reinforcement learning can be introduced later if the project is extended to learn the swing automatically rather than directly following target motion.

License

Add the project's license information here.