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
ANKLE_X = "abdomen_x"

# VELOCIDADES ORIGINAIS (Fase de Swing)
OMEGA_RX_A = 60.0       # [degrees/s] Velocidade de descida do ombro
OMEGA_RY_B = 300.0      # [degrees/s] Velocidade de extensão do cotovelo
OMEGA_ANKLE = 60.0       # [degrees/s] Velocidade de rotação do tronco


# Instantes das fases, em segundos.
T_BACKSWING = 4
T_BEFORE_IMPACT = 4.3
T_AFTER_IMPACT = 4.45
T_FOLLOW_THROUGH = 4.50
SWING_TIMES = [0.0, T_BACKSWING, T_BEFORE_IMPACT, T_AFTER_IMPACT, T_FOLLOW_THROUGH]

# shoulder1_right — eixo X
ADDRESS_SHOULDER_RX = 0.0
BACKSWING_SHOULDER_RX = -90
BEFORE_IMPACT_SHOULDER_RX = 0.0
AFTER_IMPACT_SHOULDER_RX = 0.0
FOLLOW_THROUGH_SHOULDER_RX = 0.0

# shoulder2_right — eixo Y
ADDRESS_SHOULDER_RY = -20
BACKSWING_SHOULDER_RY = -90
BEFORE_IMPACT_SHOULDER_RY = -20
AFTER_IMPACT_SHOULDER_RY = -20
FOLLOW_THROUGH_SHOULDER_RY = -50

# shoulder3_right — eixo Z
ADDRESS_SHOULDER_RZ = 0.0
BACKSWING_SHOULDER_RZ = 0.0
BEFORE_IMPACT_SHOULDER_RZ = 0.0
AFTER_IMPACT_SHOULDER_RZ = 0.0
FOLLOW_THROUGH_SHOULDER_RZ = 50

# elbow_right — cotovelo
ADDRESS_ELBOW = 0.0
BACKSWING_ELBOW = 150
BEFORE_IMPACT_ELBOW = 0.0
AFTER_IMPACT_ELBOW = 0.0
FOLLOW_THROUGH_ELBOW = 90

# Tronco (abdomen_z)
ADDRESS_ANKLE_Z = 0.0             # Neutro de frente
BACKSWING_ANKLE_Z = 40.0         # Torção para trás
BEFORE_IMPACT_ANKLE_Z = 40
AFTER_IMPACT_ANKLE_Z = -40
FOLLOW_THROUGH_ANKLE_Z = -55     # Torção para a frente

# Tronco (abdomen_x)
ADDRESS_ANKLE_X = 0.0             # Neutro de frente
BACKSWING_ANKLE_X = 35      # Torção para trás
BEFORE_IMPACT_ANKLE_X = 20
AFTER_IMPACT_ANKLE_X = -20
FOLLOW_THROUGH_ANKLE_X = -35   # Torção para a frente

def joint_angle(
    sim_t, address, backswing,
    before_impact, after_impact, follow_through
):
    return float(np.interp(
        sim_t,
        SWING_TIMES,
        [address, backswing,
         before_impact, after_impact, follow_through],
    ))


def shoulder_rx(sim_t: float) -> float:
    return joint_angle(
        sim_t,
        ADDRESS_SHOULDER_RX,
        BACKSWING_SHOULDER_RX,
        BEFORE_IMPACT_SHOULDER_RX,
        AFTER_IMPACT_SHOULDER_RX,
        FOLLOW_THROUGH_SHOULDER_RX,
    )


def shoulder_ry(sim_t: float) -> float:
    return joint_angle(
        sim_t,
        ADDRESS_SHOULDER_RY,
        BACKSWING_SHOULDER_RY,
        BEFORE_IMPACT_SHOULDER_RY,
        AFTER_IMPACT_SHOULDER_RY,
        FOLLOW_THROUGH_SHOULDER_RY,
    )


def shoulder_rz(sim_t: float) -> float:
    return joint_angle(
        sim_t,
        ADDRESS_SHOULDER_RZ,
        BACKSWING_SHOULDER_RZ,
        BEFORE_IMPACT_SHOULDER_RZ,
        AFTER_IMPACT_SHOULDER_RZ,
        FOLLOW_THROUGH_SHOULDER_RZ,
    )


def elbow_ry(sim_t: float) -> float:
    return joint_angle(
        sim_t,
        ADDRESS_ELBOW,
        BACKSWING_ELBOW,
        BEFORE_IMPACT_ELBOW,
        AFTER_IMPACT_ELBOW,
        FOLLOW_THROUGH_ELBOW,
    )

def ankle_z(sim_t: float) -> float:
    return joint_angle(
        sim_t,
        ADDRESS_ANKLE_Z,
        BACKSWING_ANKLE_Z,
        BEFORE_IMPACT_ANKLE_Z,
        AFTER_IMPACT_ANKLE_Z,
        FOLLOW_THROUGH_ANKLE_Z,
    )

def ankle_x(sim_t: float) -> float:
    return joint_angle(
        sim_t,
        ADDRESS_ANKLE_X,
        BACKSWING_ANKLE_X,
        BEFORE_IMPACT_ANKLE_X,
        AFTER_IMPACT_ANKLE_X,
        FOLLOW_THROUGH_ANKLE_X,
    )


# Mantido o dicionário com todas as tuas juntas
JOINT_TRAJECTORIES: dict[str, Callable[[float], float]] = {
    SHOULDER_RX: shoulder_rx,
    SHOULDER_RY: shoulder_ry,
    SHOULDER_RZ: shoulder_rz,
    ELBOW_RY: elbow_ry,
    ANKLE_Z: ankle_z,
    ANKLE_X: ankle_x,
}


def get_joint_targets(sim_t: float) -> dict[str, float]:
    """Converte os ângulos de graus para radianos para o MuJoCo."""
    return {
        joint_name: float(np.deg2rad(joint_trajectory(sim_t)))
        for joint_name, joint_trajectory in JOINT_TRAJECTORIES.items()
    }
