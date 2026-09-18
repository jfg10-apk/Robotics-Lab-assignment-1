Robotics Laboratory 1:

Authors:
- Tiago Duamnu
- Francisco Gonçalves
- Gonçalo

MuJoCo Humanoid Action Imitation

A Python project for simulating a humanoid in MuJoCo and making the model imitate a target action or motion.

The project uses a Python virtual environment to keep dependencies isolated from the system Python installation.

Requirements

Python 3.10 or newer

MuJoCo 3.x

Git (optional, but recommended)

The project does not require reinforcement learning libraries such as Stable-Baselines3 if the goal is direct action/motion imitation.

Project Structure

A typical project structure is:

project/
├── models/
│   └── humanoid.xml
├── src/
│   └── main.py
├── requirements.txt
├── README.md
└── .gitignore


Adjust the paths above if your project uses a different structure.

Linux
1. Install Python

Check whether Python is already installed:

python3 --version


Python 3.10+ is recommended.

On Ubuntu/Debian, if Python is not installed:

sudo apt update
sudo apt install python3 python3-pip python3-venv


Verify:

python3 --version
pip3 --version

2. Create the Virtual Environment

From the project directory:

python3 -m venv .venv


This creates a virtual environment in:

.venv/

3. Activate the Environment
source .venv/bin/activate


After activation, your terminal should show something similar to:

(.venv) user@computer:~/project$


You can verify that Python is using the virtual environment:

which python


It should point to:

.../project/.venv/bin/python

4. Install the Libraries

With the virtual environment activated:

python -m pip install --upgrade pip
python -m pip install -r requirements.txt

5. Update the Libraries

To update the packages specified by requirements.txt:

python -m pip install --upgrade -r requirements.txt


If you modify requirements.txt, simply run:

python -m pip install -r requirements.txt

6. Run the Program

With the environment activated:

python src/main.py


If your main file is located somewhere else, replace the path accordingly.

For example:

python main.py

7. Deactivate the Environment

When finished:

deactivate

Windows
1. Install Python

Download and install Python from the official Python website.

During installation, make sure to enable:

Add Python to PATH


Check the installation using PowerShell or Command Prompt:

python --version


and:

pip --version


Python 3.10+ is recommended.

2. Create the Virtual Environment

Open PowerShell or Command Prompt and navigate to the project directory:

cd path\to\project


Create the virtual environment:

python -m venv .venv


This creates:

.venv\


inside the project.

3. Activate the Environment
PowerShell
.\.venv\Scripts\Activate.ps1


After activation, you should see something similar to:

(.venv) PS C:\project>


If PowerShell prevents the activation script from running, you may need to allow local scripts for your user account:

Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser


Then activate again:

.\.venv\Scripts\Activate.ps1

Command Prompt

If you are using cmd.exe:

.venv\Scripts\activate.bat


You should see:

(.venv) C:\project>

4. Install the Libraries

With the virtual environment activated:

python -m pip install --upgrade pip
python -m pip install -r requirements.txt

5. Update the Libraries

To update the packages specified in requirements.txt:

python -m pip install --upgrade -r requirements.txt


If you modify requirements.txt, run:

python -m pip install -r requirements.txt

6. Run the Program

With the environment activated:

python src\main.py


If the main file is in the project root:

python main.py

7. Deactivate the Environment

When finished:

deactivate

Requirements

The project uses the following Python libraries:

mujoco
numpy
scipy
matplotlib
imageio
PyYAML
tqdm


The complete list and compatible versions are defined in:

requirements.txt


A typical requirements.txt is:

mujoco>=3.3,<4
numpy>=1.26,<3
scipy>=1.13,<2
matplotlib>=3.9,<4
imageio>=2.34,<3
PyYAML>=6.0,<7
tqdm>=4.66,<5

First-Time Setup
Linux
git clone <REPOSITORY_URL>
cd <PROJECT_DIRECTORY>

python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip
python -m pip install -r requirements.txt

python src/main.py

Windows
git clone <REPOSITORY_URL>
cd <PROJECT_DIRECTORY>

python -m venv .venv
.\.venv\Scripts\Activate.ps1

python -m pip install --upgrade pip
python -m pip install -r requirements.txt

python src\main.py


Replace <REPOSITORY_URL> and <PROJECT_DIRECTORY> with the appropriate values.

Subsequent Runs

You do not need to recreate the virtual environment every time.

Linux
cd <PROJECT_DIRECTORY>
source .venv/bin/activate
python src/main.py

Windows PowerShell
cd <PROJECT_DIRECTORY>
.\.venv\Scripts\Activate.ps1
python src\main.py

Updating the Project

If the project has been updated and requirements.txt has changed, activate the environment and run:

Linux
source .venv/bin/activate
python -m pip install --upgrade -r requirements.txt

Windows
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade -r requirements.txt

Testing the MuJoCo Installation

After installing the dependencies, you can check that MuJoCo is available:

python -c "import mujoco; print(mujoco.__version__)"


The command is the same on Linux and Windows.

You should see the installed MuJoCo version, for example:

3.x.x


You can also test the other main dependencies:

python -c "import numpy, scipy, matplotlib; print('Dependencies OK')"

Action Imitation

The purpose of this project is to make the MuJoCo humanoid reproduce a target action or motion.

A typical control pipeline is:

Target Action / Motion
        │
        ▼
Target Joint Positions
        │
        ▼
   Controller
        │
        ▼
     MuJoCo
        │
        ▼
     Humanoid


The project can use a controller such as a PD controller to make the humanoid follow target joint positions.

Conceptually:

torque =
    Kp * (target_position - current_position)
    - Kd * current_velocity


The exact implementation depends on how the target action is represented.

Troubleshooting
python is not recognized on Windows

Try:

py --version


If the Python launcher is available, create the environment with:

py -m venv .venv


Then activate it:

.\.venv\Scripts\Activate.ps1

pip is not recognized

Use Python to invoke pip instead:

python -m pip install -r requirements.txt


This is generally preferable to calling pip directly.

Virtual environment is not activated

Check the Python executable:

Linux
which python

Windows
where.exe python


The result should point to the project's .venv directory.

MuJoCo import error

Make sure the virtual environment is activated and reinstall the dependencies:

python -m pip install --upgrade -r requirements.txt


Then test:

python -c "import mujoco; print(mujoco.__version__)"

Starting From a Clean Environment

If the virtual environment becomes corrupted or you want to start over, delete .venv and recreate it.

Linux
rm -rf .venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

Windows PowerShell
Remove-Item -Recurse -Force .venv
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

Notes

Always activate .venv before running the project.

Do not commit .venv to Git.

Keep the project's dependencies in requirements.txt.

MuJoCo provides the physics simulation; it does not require reinforcement learning for direct motion/action imitation.

If the project later uses reinforcement learning, additional packages such as PyTorch and Stable-Baselines3 can be added to requirements.txt.

License

Add the project's license information here.s