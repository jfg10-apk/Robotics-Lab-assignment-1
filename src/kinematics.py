"""Planar two-link helpers, separate from the current MuJoCo hinge trajectory."""

import numpy as np
from scipy.spatial.transform import Rotation


def dir_kinematics(th1, th2, l1, l2):
    """Return endpoint (x, z); zero angles point both links down the negative z-axis."""
    x = -l1 * np.sin(th1) - l2 * np.sin(th1 + th2)
    z = -l1 * np.cos(th1) - l2 * np.cos(th1 + th2)
    return float(x), float(z)


def inv_kinematics(x, z, l1, l2):
    """Return the positive-elbow solution, projecting targets into the reachable annulus."""
    if not np.isfinite((x, z, l1, l2)).all():
        raise ValueError("Target coordinates and link lengths must be finite.")
    if l1 <= 0.0 or l2 <= 0.0:
        raise ValueError("Link lengths must be positive.")

    dist = np.hypot(x, z)
    minimum_reach = abs(l1 - l2)
    maximum_reach = l1 + l2
    # Project targets onto the annulus the two links can physically reach.
    projected_dist = np.clip(dist, minimum_reach, maximum_reach)
    if projected_dist != dist:
        if dist > 0.0:
            scale = projected_dist / dist
            x *= scale
            z *= scale
        else:
            x = 0.0
            z = -projected_dist
        dist = projected_dist

    dist_sq = dist**2

    # The law of cosines gives the positive-elbow solution.
    cos_th2 = np.clip(
        (dist_sq - l1**2 - l2**2) / (2.0 * l1 * l2),
        -1.0,
        1.0,
    )
    th2 = np.arccos(cos_th2)

    k1 = l1 + l2 * np.cos(th2)
    k2 = l2 * np.sin(th2)
    th1 = -np.arctan2(x, -z) - np.arctan2(k2, k1)
    return float(th1), float(th2)


def plane_rotation_deg(x_loc, y_loc, z_loc, p, q, r):
    """
        Performs a full rotation of the matrix
        p - Pitch   (degree)
        q - Roll    (degree)
        r - Yaw     (degree)
    """
    rot = Rotation.from_euler('zyx', [r, q, p], degrees=True)
    rp = rot.apply(np.array([x_loc, y_loc, z_loc]))

    return float(rp[0]), float(rp[1]), float(rp[2])




def dir_kinematics_path1_pA(env, th1, th2):
    """
        Path 1 part A
        Return endpoint of the elbow (x1, z1) only. The rotation starts in the z negative semi-axis.
        
        Notes:
            The part A have theta 2 static.
            No other limbs need to change in this path.

        env: environment and humanoid body variables
        th1: theta 1
    """

    l1 = env.link_lengths[0] # Extracts link one only
    x1 = -l1 * np.sin(th1)
    z1 = -l1 * np.cos(th1)
    
    return float(x1), float(z1)


def dir_kinematics_path1_pB(env, th1, th2):
    """
        Path 1 part B - No ankle movement
        Return endpoint of the elbow (x1, z1). The rotation starts in the z negative semi-axis. No ankle movement.
        
        Notes:
            The part B have theta 1 and 2 moving.
            No other limbs need to change in this path.

        env: environment and humanoid body variables
        th1: theta 1
    """

    l1 = env.link_lengths[0] # Extracts link one only
    x1 = -l1 * np.sin(th1)
    z1 = -l1 * np.cos(th1)
    return float(x1), float(z1)