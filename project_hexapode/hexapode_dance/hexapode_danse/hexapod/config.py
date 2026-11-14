"""Tous les réglages du robot danseur au même endroit (mètres, kg, radians)."""
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = str(ROOT / "urdf" / "hexapod.urdf")
DANCES_DIR = ROOT / "dances"
VIDEOS_DIR = ROOT / "videos"

# ---------------------------------------------------------------- Géométrie
BODY_LENGTH = 0.24
BODY_WIDTH = 0.14
BODY_THICK = 0.04
BODY_MASS = 0.60

COXA_LEN = 0.04
FEMUR_LEN = 0.08
TIBIA_LEN = 0.12
LEG_RADIUS = 0.008
FOOT_RADIUS = 0.012

COXA_MASS = 0.03
FEMUR_MASS = 0.05
TIBIA_MASS = 0.04

# nom, x, y (fixation), angle de montage (degrés). L/R = gauche/droite, F/M/R = avant/milieu/arrière
LEGS = [
    ("LF", 0.10, 0.07, 45.0),
    ("LM", 0.00, 0.08, 90.0),
    ("LR", -0.10, 0.07, 135.0),
    ("RF", 0.10, -0.07, -45.0),
    ("RM", 0.00, -0.08, -90.0),
    ("RR", -0.10, -0.07, -135.0),
]
LEG_INDEX = {leg[0]: i for i, leg in enumerate(LEGS)}
SEGMENTS = ("coxa", "femur", "tibia")
JOINT_NAMES = [f"{leg[0]}_{seg}" for leg in LEGS for seg in SEGMENTS]

JOINT_LIMITS = {"coxa": (-0.9, 0.9), "femur": (-1.6, 1.6), "tibia": (-0.3, 2.8)}
EFFORT = 2.0          # N.m (servo type MG996R)
VELOCITY = 7.0        # rad/s
KP = 0.4
KD = 1.0

# Pose debout = tous les servos à 90° : cuisse horizontale, tibia vertical
NEUTRAL_REACH = FEMUR_LEN
NEUTRAL_DROP = TIBIA_LEN
START_HEIGHT = 0.17

# ---------------------------------------------------------------- Danse
DEFAULT_BPM = 110
STEP_HEIGHT = 0.035   # levée de pied quand l'araignée marche
MAX_STEP = 0.06
MAX_VX = 0.12         # m/s
MAX_VY = 0.08
MAX_WZ = 0.60         # rad/s
CMD_SCALE = (MAX_VX, MAX_VY, MAX_WZ)

# ---------------------------------------------------------------- Simulation
SIM_HZ = 240
CONTROL_HZ = 60

DEG = math.pi / 180.0
