"""Voir le modèle et bouger chaque servo avec des curseurs (corps suspendu en l'air).

Au démarrage, tous les curseurs sont à la pose debout = tous les servos à 90°.
"""
import time

import pybullet as pb
import pybullet_data

from hexapod import config as C
from hexapod.kinematics import Kinematics


def main():
    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.loadURDF("plane.urdf")
    robot = pb.loadURDF(C.URDF_PATH, [0, 0, 0.3], useFixedBase=True)
    pb.resetDebugVisualizerCamera(0.65, 45, -25, [0, 0, 0.22])
    ids = {pb.getJointInfo(robot, j)[1].decode(): j for j in range(pb.getNumJoints(robot))}
    q0 = Kinematics().neutral()
    sliders = {}
    for name, q in zip(C.JOINT_NAMES, q0):
        lo, hi = C.JOINT_LIMITS[name.split("_")[1]]
        sliders[name] = pb.addUserDebugParameter(name, lo, hi, float(q))
    try:
        while pb.isConnected():
            for name, s in sliders.items():
                pb.resetJointState(robot, ids[name], pb.readUserDebugParameter(s))
            time.sleep(1 / 60)
    except (KeyboardInterrupt, pb.error):
        pass


if __name__ == "__main__":
    main()
