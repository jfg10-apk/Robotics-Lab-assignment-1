"""Load and validate one TOML trial and its selected evaluation case."""

from pathlib import Path
import tomllib


def load_trial(path=None):
    if path is None:
        path = Path(__file__).resolve().parents[1] / "trials" / "trial_1.toml" # Default
    with Path(path).open("rb") as file:
        config = tomllib.load(file)

    trajectory = config["trajectory"]
    times = trajectory.get("phase_times_s")
    joints = trajectory.get("joints", {})
    if joints and (
        times is None
        or len(times) < 2
        or any(right <= left for left, right in zip(times, times[1:]))
    ):
        raise ValueError("trajectory.phase_times_s must be strictly increasing")
    for joint_name, joint in joints.items():
        if times is None or len(joint["keyframes_deg"]) != len(times):
            raise ValueError(
                f"Trajectory for {joint_name!r} must have one keyframe per phase time"
            )

    actuation = config["actuation"]
    feedback_mode = actuation.get("feedback_mode", "pd")
    if feedback_mode not in ("pd", "angle_only", "angle_speed"):
        raise ValueError(f"Unsupported actuation feedback mode {feedback_mode!r}")
    if actuation["kp"] < 0 or actuation["kv"] < 0:
        raise ValueError("Actuation kp and kv must be non-negative")
    if actuation.get("default_torque_limit_nm", 0.0) <= 0:
        raise ValueError("actuation.default_torque_limit_nm must be positive")
    if any(limit <= 0 for limit in actuation["torque_limit_nm"].values()):
        raise ValueError("All actuator torque limits must be positive")
    for joint_name, profile in actuation.get("torque_profiles_nm", {}).items():
        profile_type = profile.get("type")
        if profile_type == "sinusoidal":
            if profile["period_s"] <= 0 or profile["amplitude_nm"] < 0:
                raise ValueError(
                    f"Sinusoidal torque profile for {joint_name!r} needs a "
                    "positive period and non-negative amplitude"
                )
        elif profile_type == "keyframes":
            times = profile["times_s"]
            values = profile["torque_nm"]
            if len(times) < 2 or len(times) != len(values):
                raise ValueError(
                    f"Torque profile for {joint_name!r} needs matching time "
                    "and torque arrays with at least two points"
                )
            if any(right <= left for left, right in zip(times, times[1:])):
                raise ValueError(
                    f"Torque profile times for {joint_name!r} must be strictly increasing"
                )
        else:
            raise ValueError(
                f"Unsupported torque profile {profile_type!r} for {joint_name!r}"
            )
    previous_until = 0.0
    for region in actuation.get("regions", []):
        until = region["until_s"]
        if until <= previous_until:
            raise ValueError("Actuation region until_s values must increase")
        if region["position_resistance_nm_per_rad"] < 0:
            raise ValueError("Position resistance must be non-negative")
        if region["speed_resistance_nm_s_per_rad"] < 0:
            raise ValueError("Speed resistance must be non-negative")
        if region.get("speed_control_nm_s_per_rad", 0.0) < 0:
            raise ValueError("Speed control gain must be non-negative")
        if region.get("torque_scale", 1.0) < 0:
            raise ValueError("Regional torque_scale must be non-negative")
        previous_until = until
    return config


def select_case(config, case_name):
    cases = config.get("evaluation", {}).get("cases", [])
    if not cases and case_name == "baseline":
        return {"name": "baseline", "torque_scale": 1.0, "aim_yaw_deg": 0.0}
    for case in cases:
        if case["name"] == case_name:
            scale = case.get("torque_scale", 1.0)
            if scale <= 0:
                raise ValueError("Case torque_scale must be positive")
            return case
    available = ", ".join(case["name"] for case in cases) or "baseline"
    raise ValueError(f"Unknown case {case_name!r}; available cases: {available}")