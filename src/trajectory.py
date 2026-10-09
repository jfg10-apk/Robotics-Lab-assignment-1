"""
    Description of the trajectories + actuation (com fase de Pull Back)
"""

from collections.abc import Callable
import numpy as np

# Nomes das juntas no MuJoCo
SHOULDER_RX = "shoulder1_right"
SHOULDER_RY = "shoulder2_right"
SHOULDER_RZ = "shoulder3_right"

ELBOW_RY = "elbow_right"
ANKLE_Z = "abdomen_z"
ANKLE_Y = "abdomen_y"

# VELOCIDADES ORIGINAIS (Fase de Swing)
OMEGA_RX_A = 60.0       # [degrees/s] Velocidade de descida do ombro
OMEGA_RY_B = 300.0      # [degrees/s] Velocidade de extensão do cotovelo
OMEGA_ANKLE = 1.0       # [degrees/s] Velocidade de rotação do tronco

# Instantes das fases, em segundos.
T_BACKSWING = 1.5


# shoulder1_right — eixo X
ADDRESS_SHOULDER_RX = 0.0
BACKSWING_SHOULDER_RX = -90
IMPACT_SHOULDER_RX = 0.0
FOLLOW_THROUGH_SHOULDER_RX = 0.0

# shoulder2_right — eixo Y
ADDRESS_SHOULDER_RY = 0.0
BACKSWING_SHOULDER_RY = -90
IMPACT_SHOULDER_RY = 0.0
FOLLOW_THROUGH_SHOULDER_RY = 0.0

# shoulder3_right — eixo Z
ADDRESS_SHOULDER_RZ = 0.0
BACKSWING_SHOULDER_RZ = 0.0
IMPACT_SHOULDER_RZ = 0.0
FOLLOW_THROUGH_SHOULDER_RZ = 0.0

# elbow_right — cotovelo
ADDRESS_ELBOW = 0.0
BACKSWING_ELBOW = 150
IMPACT_ELBOW = 0.0
FOLLOW_THROUGH_ELBOW = 80

# Tronco (abdomen_z e abdomen_y)
ADDRESS_ANKLE = 0.0             # Neutro de frente
BACKSWING_ANKLE = -45.0         # Torção para trás
FOLLOW_THROUGH_ANKLE = 60.0     # Torção para a frente


def shoulder_rx(sim_t: float) -> float:
    if sim_t <= T_BACKSWING:
        # Pull Back lento
        progress = sim_t / T_BACKSWING
        return ADDRESS_SHOULDER_RX + progress * (BACKSWING_SHOULDER_RX - ADDRESS_SHOULDER_RX)
    else:
        # Swing usando a tua velocidade (OMEGA_RX_A)
        delta_t = sim_t - T_BACKSWING
        angle = BACKSWING_SHOULDER_RX + (OMEGA_RX_A * delta_t)
        # Limita o ângulo para não passar do follow-through
        return min(angle, FOLLOW_THROUGH_SHOULDER_RX)

def shoulder_ry(sim_t: float) -> float:
    if sim_t <= T_BACKSWING:
        # Pull Back lento
        progress = sim_t / T_BACKSWING
        return ADDRESS_SHOULDER_RY + progress * (BACKSWING_SHOULDER_RY - ADDRESS_SHOULDER_RY)
    else:
        # Swing usando a tua velocidade (OMEGA_RX_A)
        delta_t = sim_t - T_BACKSWING
        angle = BACKSWING_SHOULDER_RY + (OMEGA_RX_A * delta_t)
        # Limita o ângulo para não passar do follow-through
        return min(angle, FOLLOW_THROUGH_SHOULDER_RY)

def shoulder_rz(sim_t: float) -> float:
    if sim_t <= T_BACKSWING:
        # Pull Back lento
        progress = sim_t / T_BACKSWING
        return ADDRESS_SHOULDER_RZ + progress * (BACKSWING_SHOULDER_RZ - ADDRESS_SHOULDER_RZ)
    else:
        # Swing usando a tua velocidade (OMEGA_RX_A)
        delta_t = sim_t - T_BACKSWING
        angle = BACKSWING_SHOULDER_RZ + (OMEGA_RX_A * delta_t)
        # Limita o ângulo para não passar do follow-through
        return min(angle, FOLLOW_THROUGH_SHOULDER_RZ)


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


SWING_TIMES = [0.0, T_BACKSWING, 3.2, 4.5]

def ankle_z(sim_t: float) -> float:
    return float(np.interp(
        sim_t,
        SWING_TIMES,
        [ADDRESS_ANKLE, BACKSWING_ANKLE, 0.0, FOLLOW_THROUGH_ANKLE],
    ))

def ankle_y(sim_t: float) -> float:
    return 10.0


# Mantido o dicionário com todas as tuas juntas
JOINT_TRAJECTORIES: dict[str, Callable[[float], float]] = {
    SHOULDER_RX: shoulder_rx,
    SHOULDER_RY: shoulder_ry,
    SHOULDER_RZ: shoulder_rz,
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
