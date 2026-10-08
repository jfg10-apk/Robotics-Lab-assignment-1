"""
    Description of the trajectories + actuation (com fase de Pull Back)
"""

from collections.abc import Callable
import numpy as np

# Nomes das juntas no MuJoCo
SHOULDER_RX = "shoulder1_right"
ELBOW_RY = "elbow_right"
ANKLE_Z = "abdomen_z"
ANKLE_Y = "abdomen_y"

# VELOCIDADES ORIGINAIS (Fase de Swing)
OMEGA_RX_A = 60.0       # [degrees/s] Velocidade de descida do ombro
OMEGA_RY_B = 300.0      # [degrees/s] Velocidade de extensão do cotovelo
OMEGA_ANKLE = 1.0       # [degrees/s] Velocidade de rotação do tronco

# TEMPOS 
T_BACKSWING = 1.5       # Tempo gasto a fazer o "pull back" 

#  ÂNGULOS ALVO 
# Ombro (shoulder1_right)
ADDRESS_SHOULDER = 0.0          # Arranque (esticado para baixo)
BACKSWING_SHOULDER = -100.0     # Topo do backswing
FOLLOW_THROUGH_SHOULDER = 90.0  # Fim do movimento

# Cotovelo (elbow_right)
ADDRESS_ELBOW = 0.0             # Esticado no arranque
BACKSWING_ELBOW = 90.0          # Dobra no topo do backswing para ganhar balanço
IMPACT_ELBOW = 0.0              # Limite para esticar no impacto

# Tronco (abdomen_z e abdomen_y)
ADDRESS_ANKLE = 0.0             # Neutro de frente
BACKSWING_ANKLE = -20.0         # Torção para trás
FOLLOW_THROUGH_ANKLE = 45.0     # Torção para a frente


def shoulder_rx(sim_t: float) -> float:
    if sim_t <= T_BACKSWING:
        # Pull Back lento
        progress = sim_t / T_BACKSWING
        return ADDRESS_SHOULDER + progress * (BACKSWING_SHOULDER - ADDRESS_SHOULDER)
    else:
        # Swing usando a tua velocidade (OMEGA_RX_A)
        delta_t = sim_t - T_BACKSWING
        angle = BACKSWING_SHOULDER + (OMEGA_RX_A * delta_t)
        # Limita o ângulo para não passar do follow-through
        return min(angle, FOLLOW_THROUGH_SHOULDER)


def elbow_ry(sim_t: float) -> float:
    if sim_t <= T_BACKSWING:
        #  Dobra o cotovelo no pull back
        progress = sim_t / T_BACKSWING
        return ADDRESS_ELBOW + progress * (BACKSWING_ELBOW - ADDRESS_ELBOW)
    else:
        # Estica usando a tua velocidade super rápida (OMEGA_RY_B)
        delta_t = sim_t - T_BACKSWING
        angle = BACKSWING_ELBOW - (OMEGA_RY_B * delta_t)
        # Limita para não dobrar ao contrário (0.0 = esticado)
        return max(angle, IMPACT_ELBOW)


def ankle_z(sim_t: float) -> float:
    if sim_t <= T_BACKSWING:
        
        progress = sim_t / T_BACKSWING
        return ADDRESS_ANKLE + progress * (BACKSWING_ANKLE - ADDRESS_ANKLE)
    else:
        
        delta_t = sim_t - T_BACKSWING
        angle = BACKSWING_ANKLE + (OMEGA_ANKLE * delta_t)
        return min(angle, FOLLOW_THROUGH_ANKLE)


def ankle_y(sim_t: float) -> float:
    # Segue a mesma lógica do eixo Z
    if sim_t <= T_BACKSWING:
        progress = sim_t / T_BACKSWING
        return ADDRESS_ANKLE + progress * (BACKSWING_ANKLE - ADDRESS_ANKLE)
    else:
        delta_t = sim_t - T_BACKSWING
        angle = BACKSWING_ANKLE + (OMEGA_ANKLE * delta_t)
        return min(angle, FOLLOW_THROUGH_ANKLE)


# Mantido o dicionário com todas as tuas juntas
JOINT_TRAJECTORIES: dict[str, Callable[[float], float]] = {
    SHOULDER_RX: shoulder_rx,
    ELBOW_RY: elbow_ry,
    ANKLE_Z: ankle_z,
    ANKLE_Y: ankle_y
}


def get_joint_targets(sim_t: float) -> dict[str, float]:
    """Converte os ângulos de graus para radianos para o MuJoCo."""
    return {
        joint_name: float(np.deg2rad(joint_trajectory(sim_t)))
        for joint_name, joint_trajectory in JOINT_TRAJECTORIES.items()
    }