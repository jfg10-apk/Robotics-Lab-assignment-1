import argparse as ap

SIMULATION_MODES = ("physics", "kinematics", "ph", "ki")

def parse_args():
    """
        Get CMD line args
    """
    parser = ap.ArgumentParser(description="MuJoCo Golf-Player Sim.")
    parser.add_argument(
        "--mode",
        choices=SIMULATION_MODES,
        default="physics",
        help=(
            "physics/ph advances MuJoCo with actuators;"
            "positions directly without stepping physics"
        ),
    )
    return parser.parse_args()