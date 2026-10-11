import csv
import numpy as np
from pathlib import Path


csv_file_path = Path(__file__).resolve().parent / "data_points.csv"


class CSVData:
    def __init__(self, file_path=None):
        self.data = []
        self.file_path = Path(csv_file_path if file_path is None else file_path)


    def add_data_point(self, env):
        ball = env.get_ball_state()
        time = env.get_time()
        position = ball["position_m"]
        velocity = ball["velocity_m_s"]
        speed = ball["speed_m_s"]
        peak_actuator_effort = float(np.max(np.abs(env.data.actuator_force)))
        actuator_effort = env.data.actuator_force

        if (time is not None and
            position is not None and
            velocity is not None and
            speed is not None and
            peak_actuator_effort is not None and
            actuator_effort is not None):
            self.data.append([
                np.round(time,3), 
                *np.round(position,2),
                *np.round(velocity, 2),
                np.round(speed, 2), 
                np.round(peak_actuator_effort, 2),
                *np.round(actuator_effort, 2),
                ])
    

    @staticmethod
    def _create_csv_file(env, file_path=None):
        path = Path(csv_file_path if file_path is None else file_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            print(f"CSV file '{path}' already exists. Data will override to the existing file.")
            
        with path.open(mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([
                "Time (s)", 
                "Ball Pos X (m)", 
                "Ball Pos Y (m)", 
                "Ball Pos Z (m)", 
                "Ball Vel X (m/s)", 
                "Ball Vel Y (m/s)", 
                "Ball Vel Z (m/s)", 
                "Ball Speed (m/s)",
                "Current actuator max effort (N*m)",
                
            ] + [env.model.joint(env.actuator_joint_ids[i]).name
                 for i in range(env.model.nu)])
            
    def save_to_csv(self):
        with self.file_path.open(mode='a', newline='', encoding='utf-8') as file:
            csv.writer(file).writerows(self.data)
        self.data.clear()  # Clear the data after saving to CSV
