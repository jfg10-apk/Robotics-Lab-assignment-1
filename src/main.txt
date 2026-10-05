import time
import mujoco.viewer
from env import GolfEnv
from controllers import SwingController

def main():
    env = GolfEnv('../assets/main.xml')
    controller = SwingController(env.model, env.data)
    
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        time.sleep(1) # Wait for viewer to load
        
        while viewer.is_running():
            step_start = time.time()
            
            # 1. Get commands from the brain
            action = controller.get_action(env.get_time())
            
            # 2. Apply commands to the motors
            env.data.ctrl[:] = action
            
            # 3. Step physics and sync viewer
            env.step()
            viewer.sync()
            
            # 4. Maintain real-time
            time_until_next = env.model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == "__main__":
    main()