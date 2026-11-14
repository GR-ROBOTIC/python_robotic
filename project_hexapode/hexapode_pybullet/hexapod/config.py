"""Tous les paramètres du robot au même endroit.

Le générateur d'URDF, la démarche et l'environnement lisent ce fichier :
si tu changes une longueur de patte ici, régénère l'URDF (./run.sh urdf)
et tout reste cohérent. Unités : mètres, kilogrammes, radians.
"""
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = str(ROOT / "urdf" / "hexapod.urdf")
RUNS_DIR = ROOT / "runs"

# ---------------------------------------------------------------- Géométrie
BODY_LENGTH = 0.24
BODY_WIDTH = 0.14
BODY_THICK = 0.04
BODY_MASS = 0.60

COXA_LEN = 0.04     # hanche (servo horizontal)
FEMUR_LEN = 0.08    # cuisse
TIBIA_LEN = 0.12    # tibia
LEG_RADIUS = 0.008
FOOT_RADIUS = 0.012

COXA_MASS = 0.03
FEMUR_MASS = 0.05
TIBIA_MASS = 0.04

# nom, x, y (fixation sur le corps), angle de montage en degrés
# x vers l'avant, y vers la gauche. L = gauche, R = droite ; F/M/R = avant/milieu/arrière
LEGS = [
    ("LF", 0.10, 0.07, 45.0),
    ("LM", 0.00, 0.08, 90.0),
    ("LR", -0.10, 0.07, 135.0),
    ("RF", 0.10, -0.07, -45.0),
    ("RM", 0.00, -0.08, -90.0),
    ("RR", -0.10, -0.07, -135.0),
]
SEGMENTS = ("coxa", "femur", "tibia")
JOINT_NAMES = [f"{leg[0]}_{seg}" for leg in LEGS for seg in SEGMENTS]  # 18 servos

# Limites articulaires (rad), couple max (N.m, ~servo MG996R), vitesse max (rad/s)
JOINT_LIMITS = {"coxa": (-0.9, 0.9), "femur": (-1.6, 1.6), "tibia": (-0.3, 2.8)}
EFFORT = 2.0
VELOCITY = 7.0
KP = 0.3            # gain du contrôle en position PyBullet
KD = 1.0

# Position neutre du pied, dans le repère de la patte
NEUTRAL_REACH = 0.09   # distance horizontale après la hanche
NEUTRAL_DROP = 0.09    # hauteur du pied sous l'articulation de hanche
START_HEIGHT = 0.13

# ---------------------------------------------------------------- Démarche
GAIT_FREQ = 1.4        # cycles par seconde
STEP_HEIGHT = 0.035    # levée du pied
MAX_STEP = 0.06        # longueur de pas max

# Vitesses max : une commande normalisée [-1, 1] est multipliée par ceci
MAX_VX = 0.12          # m/s avant/arrière
MAX_VY = 0.08          # m/s latéral
MAX_WZ = 0.50          # rad/s rotation
CMD_SCALE = (MAX_VX, MAX_VY, MAX_WZ)

# ---------------------------------------------------------------- Simulation
SIM_HZ = 240
CONTROL_HZ = 30

# Commandes prédéfinies (vx, vy, rotation), normalisées
COMMANDS = {
    "stop": (0.0, 0.0, 0.0),
    "avancer": (1.0, 0.0, 0.0),
    "reculer": (-1.0, 0.0, 0.0),
    "gauche": (0.0, 1.0, 0.0),
    "droite": (0.0, -1.0, 0.0),
    "tourner_gauche": (0.0, 0.0, 1.0),
    "tourner_droite": (0.0, 0.0, -1.0),
    "cercle": (0.8, 0.0, 0.7),
}

DEG = math.pi / 180.0
