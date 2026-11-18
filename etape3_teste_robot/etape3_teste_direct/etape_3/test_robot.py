"""
Test manuel du robot à 16 servos : un curseur par servo.
    python test_robot.py
Réglage : SUSPENDU = True pour accrocher le robot en l'air.
"""
import time

import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client

from robot_commun import (COUPLE_SERVO, DT_SIM, ORDRE_SERVOS, POSE_REPOS, charger_robot,
                          commander_servos)

SUSPENDU = False

bc = bullet_client.BulletClient(connection_mode=p.GUI)
bc.setAdditionalSearchPath(pybullet_data.getDataPath())
bc.setGravity(0, 0, -9.81)
bc.setTimeStep(DT_SIM)
bc.loadURDF("plane.urdf")
bc.resetDebugVisualizerCamera(1.0, 50, -15, [0, 0, 0.3])

hauteur = 0.6 if SUSPENDU else 0.33
robot, servos, bas, haut, _ = charger_robot(bc, fixe=SUSPENDU, hauteur=hauteur)
for j, a in zip(servos, POSE_REPOS):
    bc.resetJointState(robot, j, a)

print("\n=== 16 servomoteurs (ordre officiel) ===")
curseurs = []
for k, (nom, lo, hi, a0) in enumerate(zip(ORDRE_SERVOS, bas, haut, POSE_REPOS)):
    curseurs.append(bc.addUserDebugParameter(f"{k:2d} {nom}", lo, hi, a0))
    print(f"{k:2d} : {nom:20s} [{lo:+.2f}, {hi:+.2f}] rad")
bouton = bc.addUserDebugParameter("Remettre debout", 1, 0, 0)
clics = 0
texte, pas = None, 0

while bc.isConnected():
    if bc.readUserDebugParameter(bouton) > clics:
        clics = bc.readUserDebugParameter(bouton)
        bc.resetBasePositionAndOrientation(robot, [0, 0, hauteur], [0, 0, 0, 1])
        bc.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])
        for j, a in zip(servos, POSE_REPOS):
            bc.resetJointState(robot, j, a)
    commander_servos(bc, robot, servos, [bc.readUserDebugParameter(c) for c in curseurs])
    bc.stepSimulation()
    pas += 1
    if pas % 24 == 0:
        couples = [(abs(bc.getJointState(robot, j)[3]), n) for j, n in zip(servos, ORDRE_SERVOS)]
        c, nom = max(couples)
        pct = 100 * c / COUPLE_SERVO
        msg = f"Servo le plus chargé : {nom} {c:.2f} N.m ({pct:.0f} %)" + ("  SATURÉ !" if pct > 95 else "")
        kw = {"replaceItemUniqueId": texte} if texte is not None else {}
        texte = bc.addUserDebugText(msg, [0, -0.4, 0.75], textColorRGB=[0.8, 0, 0] if pct > 95 else [0, 0, 0],
                                    textSize=1.2, **kw)
    time.sleep(DT_SIM)
