"""Cinématique de l'araignée : pattes, posture du corps et marche.

Tout passe par une seule fonction, joint_targets() : on lui donne la position des
6 pieds au sol et la posture voulue du corps (décalage + inclinaison), elle renvoie
les 18 angles de servos. C'est ce qui permet de danser : les pieds restent posés
pendant que le corps ondule, tourne, se baisse ou se cabre.
Le même calcul tournera sur l'ESP32.
"""
import math

import numpy as np

from . import config as C


def leg_ik(lx, ly, lz):
    """Pied (lx, ly, lz) dans le repère de la patte → (hanche, cuisse, tibia) en radians."""
    L1, L2, L3 = C.COXA_LEN, C.FEMUR_LEN, C.TIBIA_LEN
    q1 = math.atan2(ly, lx)
    r = math.hypot(lx, ly) - L1
    h = -lz
    d = min(max(math.hypot(r, h), abs(L2 - L3) + 1e-3), L2 + L3 - 1e-3)
    c3 = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    q3 = math.acos(max(-1.0, min(1.0, c3)))
    q2 = math.atan2(h, r) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    return q1, q2, q3


def rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class Kinematics:
    # Marche tripode : (LF, LR, RM) puis (LM, RF, RR)
    PHASE_OFFSETS = {"LF": 0.0, "LM": 0.5, "LR": 0.0, "RF": 0.5, "RM": 0.0, "RR": 0.5}

    def __init__(self):
        self.mounts = []
        nominal = []
        for name, mx, my, ang in C.LEGS:
            a = ang * C.DEG
            reach = C.COXA_LEN + C.NEUTRAL_REACH
            self.mounts.append((mx, my, a, self.PHASE_OFFSETS[name]))
            nominal.append((mx + reach * math.cos(a), my + reach * math.sin(a), -C.NEUTRAL_DROP))
        self.nominal = np.array(nominal)          # pieds au repos, repère « sol » du corps
        self.lower = np.array([C.JOINT_LIMITS[n.split("_")[1]][0] for n in C.JOINT_NAMES])
        self.upper = np.array([C.JOINT_LIMITS[n.split("_")[1]][1] for n in C.JOINT_NAMES])

    def joint_targets(self, feet, body=(0, 0, 0, 0, 0, 0)):
        """feet : (6, 3) positions des pieds ; body : (x, y, z, roulis, tangage, lacet)."""
        x, y, z, roll, pitch, yaw = body
        R = rotation(roll, pitch, yaw)
        local = (np.asarray(feet) - np.array([x, y, z])) @ R      # = R.T @ (f - offset)
        q = []
        for (mx, my, a, _), (fx, fy, fz) in zip(self.mounts, local):
            px, py = fx - mx, fy - my
            c, s = math.cos(-a), math.sin(-a)
            q.extend(leg_ik(c * px - s * py, s * px + c * py, fz))
        return np.clip(np.array(q), self.lower, self.upper)

    def gait_offsets(self, phase, cmd, freq):
        """Décalage des pieds pour marcher. cmd = (vx m/s, vy m/s, wz rad/s), freq en Hz."""
        vx, vy, wz = cmd
        out = np.zeros((6, 3))
        if abs(vx) + abs(vy) + 0.1 * abs(wz) < 2e-3:
            return out
        t_stance = 0.5 / freq
        for i, ((mx, my, a, off), (nx, ny, _)) in enumerate(zip(self.mounts, self.nominal)):
            dx = (vx - wz * ny) * t_stance
            dy = (vy + wz * nx) * t_stance
            n = math.hypot(dx, dy)
            if n > C.MAX_STEP:
                dx, dy = dx * C.MAX_STEP / n, dy * C.MAX_STEP / n
            p = (phase + off) % 1.0
            if p < 0.5:
                k, dz = 0.5 - p / 0.5, 0.0
            else:
                s = (p - 0.5) / 0.5
                k = -0.5 + (1 - math.cos(math.pi * s)) / 2
                dz = C.STEP_HEIGHT * math.sin(math.pi * s)
            out[i] = (dx * k, dy * k, dz)
        return out

    def neutral(self):
        return self.joint_targets(self.nominal)
