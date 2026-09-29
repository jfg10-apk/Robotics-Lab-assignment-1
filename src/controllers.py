import numpy as np
import math

class SwingController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
    def get_action(self, time):
        # Calculate your IK or trajectory math here

        # Right shoulder - ACT 0
        target_shoulder_r = 0.5 * math.sin(time * 3.0)

        # Right elbow - ACT 1
        target_elbow_r = 0.2 * math.cos(time * 3.0)

        # Left shoulder - ACT 2
        target_shoulder_l = 0

        
        # Return the joint commands
        return [target_shoulder_r, target_elbow_r, target_shoulder_l]