import mujoco
import mujoco.viewer
import numpy as np
import time


model = mujoco.MjModel.from_xml_path("../assets/human.xml")
data = mujoco.MjData(model)


# ============================================================
# ATUADORES QUE VAMOS CONTROLAR
# ============================================================

swing_actuators = [
    "abdomen_x",       # [0] Rotação da cintura
    "abdomen_y",       # [0] Rotação da cintura
    "abdomen_z",       # [0] Rotação da cintura 
    "shoulder1_right", # [1] Rotação do ombro direito (frente/trás)
    "shoulder2_right", # [2] Elevação do ombro direito (cima/baixo)
    "elbow_right",     # [3] Dobra do cotovelo direito
    "pulso_right",     # [4] Rotação do pulso direito
    "shoulder1_left",  # [5] Rotação do ombro esquerdo (frente/trás)
    "shoulder2_left",  # [6] Elevação do ombro esquerdo (cima/baixo)
    "elbow_left",      # [7] Dobra do cotovelo esquerdo
    "pulso_left"       # [8] Rotação do pulso esquerdo
]

# Trajetória (Keyframes)
# Formato: (Tempo_em_segundos, [lista_de_11_angulos_em_graus])
trajectory = [
    #        [ abd_x, abd_y, abd_z, sh1_R, sh2_R, elb_R, pul_R, sh1_L, sh2_L, elb_L, pul_L ]
    (0.0, np.deg2rad([ 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0  ])), # Postura inicial
    (2.0, np.deg2rad([ 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0  ])), # Top of Backswing
    (5.0, np.deg2rad([ -20,    20,    25,   -40,   -20,   -30,    10,   -20,    20,   -20,   -10  ])), # Impact 
    (8.0, np.deg2rad([  40,   -40,   -30,    40,    -8,   -30,    10,    29,     7,     8,     2  ])), # Follow Through
]


#####################################################################################################################################
#############################Maybe pensar nos ângulos? Tenho de mudar para radianos com o 'pi' ainda n fiz################################################
#####################################################################################################################################


# Obter os IDs dos atuadores
actuator_ids = [
    model.actuator(name).id
    for name in swing_actuators
]


def get_target_angles(t):
    """Função para interpolar os ângulos entre os keyframes"""
    for i in range(len(trajectory) - 1):
        t1, pos1 = trajectory[i]
        t2, pos2 = trajectory[i+1]
        if t1 <= t <= t2:
            alpha = (t - t1) / (t2 - t1)
            # Interpolação linear simples
            return np.array(pos1) * (1 - alpha) + np.array(pos2) * alpha
    
    # Se o tempo passar do último keyframe, mantém a última pose
    return np.array(trajectory[-1][1])


# ============================================================
# SIMULAÇÃO
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()
        
        # Tempo atual da simulação (inicia em 0)
        t = data.time
        
        # Obter os ângulos desejados para o instante 't'
        targets = get_target_angles(t)
        
        for i, actuator_id in enumerate(actuator_ids):
       
            data.ctrl[actuator_id] = targets[i]
            
        # Avançar a física um passo (0.005 segundos)
        mujoco.mj_step(model, data)
        
        # Sincronizar o ecrã
        viewer.sync()
        
        
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        
        # Fazer 'reset' à simulação a cada 4 segundos para repetir o movimento eternamente
        if t > 10.0:
            mujoco.mj_resetData(model, data)