import csv
import numpy as np
from pathlib import Path


csv_file_path = "data_points.csv"


class CSVData:
    def __init__(self):
        self.data = []


    def add_data_point(self, env):
        ball = env.get_ball_state()
        time = env.get_time()
        position = ball["position_m"]
        velocity = ball["velocity_m_s"]
        speed = ball["speed_m_s"]

        if (time is not None and
            position is not None and
            velocity is not None and
            speed is not None):
            self.data.append([np.round(time,3), np.round(position,2), np.round(velocity, 2), np.round(speed, 2)])
    

    def _create_csv_file():
        if Path(csv_file_path).exists():
            print(f"CSV file '{csv_file_path}' already exists. Data will override to the existing file.")
            
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Ball Pos X (m)", "Ball Pos Y (m)", "Ball Pos Z (m)", "Ball Vel X (m)", "Ball Vel Y (m)", "Ball Vel Z (m)", "Ball Speed (m/s)"])
            
    def save_to_csv(self):
        with open(csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file, delimiter="\t")
            for data_point in self.data:
                time, position, velocity, speed = data_point
                writer.writerow([
                    time, 
                    position[0], 
                    position[1], 
                    position[2], 
                    velocity[0], 
                    velocity[1], 
                    velocity[2], 
                    speed]
                )
        self.data.clear()  # Clear the data after saving to CSV