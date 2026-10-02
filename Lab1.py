import mujoco
import mujoco.viewer
import numpy as np
import time


model = mujoco.MjModel.from_xml_path("human.xml")
data = mujoco.MjData(model)



Kp = 150.0  
Kd = 10.0   


##########################################################################################################
############################### ESTA PARTE É OQ IMPORTA RAPAZAIADA ACHO EU #################################
##########################################################################################################
# Escolher os atuadores/motores que queremos controlar para o swing
swing_actuators = [
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
# Formato: (Tempo_em_segundos, [lista_de_9_angulos_em_radianos])
trajectory = [
    #        [ abd,   sh1_R, sh2_R, elb_R, pul_R, sh1_L, sh2_L, elb_L, pul_L ]
    (0.0, [ 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0  ]), # Postura inicial
    (1.5, [-0.8,   0.5,   0.0,  -1.0,  -0.5,  -0.5,   0.0,  -0.2,   0.0  ]), # Top of Backswing
    (2.0, [ 0.2,  -0.2,   0.0,  -0.2,   0.0,   0.2,   0.0,   0.0,   0.0  ]), # Impact 
    (3.0, [ 1.0,  -1.0,   0.0,  -0.5,   0.5,   1.0,   0.0,  -1.2,   0.5  ])  # Follow-through 
]

#####################################################################################################################################
#############################Maybe pensar nos ângulos? Tenho de mudar para radianos com o 'pi' ainda n fiz################################################
#####################################################################################################################################

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


with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()
        
        # Tempo atual da simulação (inicia em 0)
        t = data.time
        
        # Obter os ângulos desejados para o instante 't'
        targets = get_target_angles(t)
        
        # Loop do Controlador PD
        for idx, act_name in enumerate(swing_actuators):
            # Obter o ID do atuador através do nome
            act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            
            # Encontrar a junta (joint) correspondente a este motor
            joint_id = model.actuator_trnid[act_id, 0]
            qpos_idx = model.jnt_qposadr[joint_id] # Índice da posição
            qvel_idx = model.jnt_dofadr[joint_id]  # Índice da velocidade
            
            # Ler a posição (ângulo) e velocidade atuais da junta
            current_pos = data.qpos[qpos_idx]
            current_vel = data.qvel[qvel_idx]
            
            # Matemática do PID (só P e D)
            error = targets[idx] - current_pos
            torque = (Kp * error) - (Kd * current_vel)
            
            
            # Como a força máxima ('gear') também afeta isto, vamos aplicar o torque
            data.ctrl[act_id] = torque
            
        # Avançar a física um passo (0.005 segundos)
        mujoco.mj_step(model, data)
        
        # Sincronizar o ecrã
        viewer.sync()
        
        
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        
        # Fazer 'reset' à simulação a cada 4 segundos para repetir o movimento eternamente
        if t > 4.0:
            mujoco.mj_resetData(model, data)