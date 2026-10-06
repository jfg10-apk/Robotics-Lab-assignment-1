"""Backward-compatible entry point for the MuJoCo humanoid simulation.

The project logic has been split into dedicated modules for the environment,
controller logic, and trajectory definition. This file remains as a small wrapper
so older launch commands keep working.
"""

from main import main


if __name__ == "__main__":
    main()
