"""
Constantes et outils partagés par tous les scripts de l'étape 3.

Ordre officiel des 16 servos (le même partout : simulation, export, ESP32) :
    0-5   jambe gauche : hanche_lacet, hanche_roulis, hanche_tangage, genou, cheville_tangage, cheville_roulis
    6-11  jambe droite : idem
    12-13 bras gauche  : epaule_tangage, epaule_roulis
    14-15 bras droit   : epaule_tangage, epaule_roulis
Les 12 premiers sont pilotés par le réseau de neurones (marche),
les 4 bras par le programme (danses, gestes).
"""
import os

import numpy as np
import pybullet as p

DOSSIER = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(DOSSIER, "bipede_16servos.urdf")

_JAMBE = ["hanche_lacet", "hanche_roulis", "hanche_tangage", "genou", "cheville_tangage", "cheville_roulis"]
JAMBES = [f"{n}_g" for n in _JAMBE] + [f"{n}_d" for n in _JAMBE]
BRAS = ["epaule_tangage_g", "epaule_roulis_g", "epaule_tangage_d", "epaule_roulis_d"]
ORDRE_SERVOS = JAMBES + BRAS
N_JAMBES, N_BRAS, N_SERVOS = len(JAMBES), len(BRAS), len(ORDRE_SERVOS)

# Pose de repos : genoux légèrement fléchis, pieds à plat, bras le long du corps
_REPOS = {"hanche_tangage": -0.3, "genou": 0.6, "cheville_tangage": -0.3}
POSE_REPOS = np.array([_REPOS.get(n.rsplit("_", 1)[0], 0.0) for n in ORDRE_SERVOS])

# Servos MG996R
COUPLE_SERVO = 1.0       # N·m (~10 kg·cm à 6 V)
VITESSE_SERVO = 7.0      # rad/s (~0,14 s / 60°)

# Temps
DT_SIM = 1 / 200         # pas de la physique
SOUS_PAS = 4             # -> 50 décisions par seconde
DT = DT_SIM * SOUS_PAS
HAUTEUR_DEPART = 0.33


def charger_robot(bc, fixe=False, hauteur=HAUTEUR_DEPART):
    """Charge le robot et renvoie (id, indices des 16 servos dans l'ordre officiel, bas, haut, pieds)."""
    robot = bc.loadURDF(URDF, [0, 0, hauteur], flags=p.URDF_USE_INERTIA_FROM_FILE, useFixedBase=fixe)
    par_nom, pieds = {}, {}
    for i in range(bc.getNumJoints(robot)):
        info = bc.getJointInfo(robot, i)
        par_nom[info[1].decode()] = (i, info[8], info[9])
        lien = info[12].decode()
        if lien.startswith("pied_"):
            pieds[lien] = i
    indices = [par_nom[n][0] for n in ORDRE_SERVOS]
    bas = np.array([par_nom[n][1] for n in ORDRE_SERVOS])
    haut = np.array([par_nom[n][2] for n in ORDRE_SERVOS])
    return robot, indices, bas, haut, pieds


def commander_servos(bc, robot, indices, angles, couple=COUPLE_SERVO):
    """Envoie une position cible à chaque servo, avec le couple et la vitesse d'un MG996R."""
    for j, a in zip(indices, angles):
        bc.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=float(a),
                                 force=couple, maxVelocity=VITESSE_SERVO)


def preparer_fenetre(bc):
    """Fenêtre PyBullet : cache le panneau latéral et coupe les raccourcis clavier internes
    (pour pouvoir utiliser les lettres a à h sans déclencher d'options de PyBullet)."""
    bc.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    bc.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
