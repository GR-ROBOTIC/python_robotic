"""
Compare la vitesse de MuJoCo et de PyBullet sur le MÊME robot (sans fenêtre).
Les deux simulent 200 pas de physique par seconde et 50 décisions par seconde.

    python vitesse.py
"""
import os
import time

import numpy as np

DOSSIER = os.path.dirname(os.path.abspath(__file__))
SECONDES = 30.0                  # durée de robot simulée
REPOS = np.array([0, 0, -0.3, 0.6, -0.3, 0] * 2)


def mesurer_mujoco():
    import mujoco
    m = mujoco.MjModel.from_xml_path(os.path.join(DOSSIER, "bipede.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    rng = np.random.default_rng(0)
    t0 = time.perf_counter()
    decisions = int(SECONDES * 50)
    for _ in range(decisions):
        d.ctrl[:] = REPOS + rng.uniform(-0.1, 0.1, 12)
        for _ in range(4):
            mujoco.mj_step(m, d)
    return decisions / (time.perf_counter() - t0)


def mesurer_pybullet():
    import pybullet as p
    import pybullet_data
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / 200)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF(os.path.join(DOSSIER, "bipede_12servos.urdf"), [0, 0, 0.33],
                       flags=p.URDF_USE_INERTIA_FROM_FILE)
    joints = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE]
    for j, a in zip(joints, REPOS):
        p.resetJointState(robot, j, a)
    rng = np.random.default_rng(0)
    t0 = time.perf_counter()
    decisions = int(SECONDES * 50)
    for _ in range(decisions):
        for j, a in zip(joints, REPOS + rng.uniform(-0.1, 0.1, 12)):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=a, force=1.0, maxVelocity=7.0)
        for _ in range(4):
            p.stepSimulation()
    vitesse = decisions / (time.perf_counter() - t0)
    p.disconnect()
    return vitesse


def main():
    print(f"Simulation de {SECONDES:.0f} s de robot sur UN cœur...\n")
    mj = mesurer_mujoco()
    print(f"MuJoCo   : {mj:8.0f} décisions/s  ({mj / 50:5.0f} x plus vite que le temps réel)")
    try:
        pb = mesurer_pybullet()
        print(f"PyBullet : {pb:8.0f} décisions/s  ({pb / 50:5.0f} x plus vite que le temps réel)")
        print(f"\n==> MuJoCo est {mj / pb:.1f} x plus rapide sur ce PC.")
    except ImportError:
        print("PyBullet n'est pas installé dans cet environnement : comparaison impossible.")


if __name__ == "__main__":
    main()
