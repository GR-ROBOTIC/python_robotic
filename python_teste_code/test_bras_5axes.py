"""
Test du bras robotisé à 5 axes dans PyBullet.
Placez ce script dans le même dossier que bras_5axes.urdf, puis :
    conda activate pybullet
    python test_bras_5axes.py
"""
import os
import time
import pybullet as p
import pybullet_data

DOSSIER = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(DOSSIER, "bras_5axes.urdf")

# --- Connexion et monde ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=45,
                             cameraPitch=-25, cameraTargetPosition=[0, 0, 0.4])

# --- Chargement du robot ---
robot = p.loadURDF(URDF, basePosition=[0, 0, 0], useFixedBase=True)

# --- Liste des articulations ---
print("\n=== Articulations du robot ===")
curseurs = []
outil = None
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    nom, type_joint = info[1].decode(), info[2]
    bas, haut = info[8], info[9]
    print(f"{i} : {nom:25s} type={type_joint}  limites=[{bas:.2f}, {haut:.2f}]")
    if type_joint == p.JOINT_REVOLUTE:
        curseurs.append((i, p.addUserDebugParameter(nom, bas, haut, 0.0)))
    if info[12].decode() == "outil":
        outil = i
print(f"\n{len(curseurs)} axes motorisés détectés.\n")

# --- Boucle de simulation ---
texte = None
compteur = 0
while p.isConnected():
    for joint, curseur in curseurs:
        cible = p.readUserDebugParameter(curseur)
        p.setJointMotorControl2(robot, joint, p.POSITION_CONTROL,
                                targetPosition=cible, force=50)
    p.stepSimulation()

    # Affiche la position de l'outil environ 10 fois par seconde
    compteur += 1
    if outil is not None and compteur % 24 == 0:
        pos = p.getLinkState(robot, outil)[0]
        msg = f"Outil : x={pos[0]:.3f}  y={pos[1]:.3f}  z={pos[2]:.3f} m"
        if texte is None:
            texte = p.addUserDebugText(msg, [0, 0, 1.0], textColorRGB=[0, 0, 0])
        else:
            texte = p.addUserDebugText(msg, [0, 0, 1.0], textColorRGB=[0, 0, 0],
                                       replaceItemUniqueId=texte)
    time.sleep(1 / 240)
