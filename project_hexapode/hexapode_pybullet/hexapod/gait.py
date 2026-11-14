"""Démarche tripode + cinématique inverse.

Le robot marche déjà avec ce générateur seul (sans IA). L'apprentissage
par renforcement ajoute ensuite des petites corrections (« résidus ») sur
les 18 servos pour améliorer stabilité et précision.
C'est ce code-là que tu pourras porter sur l'ESP32.
"""
import math
import numpy as np
from . import config as C


def leg_ik(lx, ly, lz):
    """Cinématique inverse d'une patte 3 axes.

    (lx, ly, lz) : position du pied dans le repère de la patte
    (origine = axe de hanche, x vers l'extérieur, z vers le haut).
    Retourne (coxa, femur, tibia) en radians. Fémur/tibia positifs = vers le bas.
    """
    L1, L2, L3 = C.COXA_LEN, C.FEMUR_LEN, C.TIBIA_LEN
    q1 = math.atan2(ly, lx)
    r = math.hypot(lx, ly) - L1
    h = -lz
    d = math.hypot(r, h)
    d = min(max(d, abs(L2 - L3) + 1e-3), L2 + L3 - 1e-3)
    c3 = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    q3 = math.acos(max(-1.0, min(1.0, c3)))
    alpha = math.atan2(h, r)
    q2 = alpha - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    return q1, q2, q3


class TripodGait:
    # Groupe A (LF, LR, RM) et groupe B (LM, RF, RR) alternent
    PHASE_OFFSETS = {"LF": 0.0, "LM": 0.5, "LR": 0.0, "RF": 0.5, "RM": 0.0, "RR": 0.5}

    def __init__(self, freq=C.GAIT_FREQ, step_height=C.STEP_HEIGHT, max_step=C.MAX_STEP):
        self.freq = freq
        self.step_height = step_height
        self.max_step = max_step
        self.legs = []
        for name, mx, my, ang in C.LEGS:
            a = ang * C.DEG
            reach = C.COXA_LEN + C.NEUTRAL_REACH
            self.legs.append({
                "name": name, "mx": mx, "my": my, "a": a,
                "nx": mx + reach * math.cos(a), "ny": my + reach * math.sin(a),
                "offset": self.PHASE_OFFSETS[name],
            })

    def foot_targets(self, phase, cmd):
        """Positions des 6 pieds dans le repère du corps. cmd = (vx m/s, vy m/s, wz rad/s)."""
        vx, vy, wz = cmd
        moving = abs(vx) + abs(vy) + 0.1 * abs(wz) > 2e-3
        t_stance = 0.5 / self.freq
        out = []
        for leg in self.legs:
            nx, ny = leg["nx"], leg["ny"]
            # déplacement du sol sous ce pied pendant l'appui
            dx = (vx - wz * ny) * t_stance
            dy = (vy + wz * nx) * t_stance
            n = math.hypot(dx, dy)
            if n > self.max_step:
                dx, dy = dx * self.max_step / n, dy * self.max_step / n
            p = (phase + leg["offset"]) % 1.0
            if p < 0.5:                       # appui : le pied recule sous le corps
                k = 0.5 - p / 0.5
                oz = 0.0
            else:                             # vol : le pied revient vers l'avant, en l'air
                s = (p - 0.5) / 0.5
                k = -0.5 + (1 - math.cos(math.pi * s)) / 2
                oz = self.step_height * math.sin(math.pi * s) if moving else 0.0
            out.append((nx + dx * k, ny + dy * k, -C.NEUTRAL_DROP + oz))
        return out

    def joint_targets(self, phase, cmd):
        """Angles des 18 servos (ordre de config.JOINT_NAMES)."""
        q = []
        for leg, (fx, fy, fz) in zip(self.legs, self.foot_targets(phase, cmd)):
            px, py = fx - leg["mx"], fy - leg["my"]
            c, s = math.cos(-leg["a"]), math.sin(-leg["a"])
            q.extend(leg_ik(c * px - s * py, s * px + c * py, fz))
        return np.array(q, dtype=np.float32)

    def neutral(self):
        return self.joint_targets(0.0, (0.0, 0.0, 0.0))
