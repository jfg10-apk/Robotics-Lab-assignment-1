import mujoco
import mujoco.viewer

class GolfEnv:
    def __init__(self, xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
    def step(self):
        mujoco.mj_step(self.model, self.data)
        
    def get_time(self):
        return self.data.time