"""
Test du bipède à 12 servomoteurs MG996R dans PyBullet.
Placez ce script dans le même dossier que bipede_12servos.urdf, puis :
    conda activate pybullet
    python test_bipede.py

Réglage principal :
    SUSPENDU = True  -> le robot est accroché en l'air (pour tester les axes)
    SUSPENDU = False -> le robot est posé au sol et doit tenir debout
"""
import os
import time
import pybullet as p
import pybullet_data

SUSPENDU = False
COUPLE_MAX = 1.0          # N·m, couple d'un MG996R (~10 kg·cm à 6 V)
VITESSE_MAX = 7.0         # rad/s

DOSSIER = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(DOSSIER, "bipede_12servos.urdf")

# --- Monde ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)
sol = p.loadURDF("plane.urdf")
p.changeDynamics(sol, -1, lateralFriction=1.0)
p.resetDebugVisualizerCamera(cameraDistance=0.9, cameraYaw=50,
                             cameraPitch=-15, cameraTargetPosition=[0, 0, 0.3])

# --- Robot ---
hauteur = 0.6 if SUSPENDU else 0.335   # 0.335 m = pieds posés au sol
robot = p.loadURDF(URDF, basePosition=[0, 0, hauteur], useFixedBase=SUSPENDU)

# --- Articulations motorisées ---
print("\n=== Servomoteurs ===")
servos = []
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    if info[2] == p.JOINT_REVOLUTE:
        nom, bas, haut = info[1].decode(), info[8], info[9]
        curseur = p.addUserDebugParameter(nom, bas, haut, 0.0)
        servos.append((i, nom, curseur))
        print(f"{len(servos):2d}. joint {i:2d} : {nom:22s} [{bas:+.2f}, {haut:+.2f}] rad")
print(f"\n{len(servos)} servomoteurs détectés.\n")

bouton_reset = p.addUserDebugParameter("Remettre debout", 1, 0, 0)
clics_reset = 0
texte = None
pas = 0

# --- Boucle ---
while p.isConnected():
    # Réinitialisation si le robot est tombé
    clics = p.readUserDebugParameter(bouton_reset)
    if clics > clics_reset:
        clics_reset = clics
        p.resetBasePositionAndOrientation(robot, [0, 0, hauteur], [0, 0, 0, 1])
        p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])
        for j, _, _ in servos:
            p.resetJointState(robot, j, 0.0)

    # Commande des servos (comme un vrai servo : position + couple et vitesse limités)
    for j, _, curseur in servos:
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                targetPosition=p.readUserDebugParameter(curseur),
                                force=COUPLE_MAX, maxVelocity=VITESSE_MAX)
    p.stepSimulation()

    # Affichage : servo le plus sollicité (≈ 10 fois par seconde)
    pas += 1
    if pas % 24 == 0:
        couples = [(abs(p.getJointState(robot, j)[3]), nom) for j, nom, _ in servos]
        c, nom = max(couples)
        pct = 100 * c / COUPLE_MAX
        z = p.getBasePositionAndOrientation(robot)[0][2]
        etat = "SATURÉ !" if pct > 95 else "ok"
        msg = f"Servo le plus chargé : {nom} {c:.2f} N.m ({pct:.0f} %) {etat} | hauteur bassin {z:.3f} m"
        couleur = [0.8, 0, 0] if pct > 95 else [0, 0, 0]
        if texte is None:
            texte = p.addUserDebugText(msg, [0, -0.4, 0.75], textColorRGB=couleur, textSize=1.2)
        else:
            texte = p.addUserDebugText(msg, [0, -0.4, 0.75], textColorRGB=couleur,
                                       textSize=1.2, replaceItemUniqueId=texte)
    time.sleep(1 / 240)
