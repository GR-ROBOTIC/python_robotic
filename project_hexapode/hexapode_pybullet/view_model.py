"""Visualiser le modèle URDF et bouger chaque servo avec des curseurs.

Le corps est suspendu en l'air (base fixe) pour bien voir les pattes.
  python view_model.py            # curseurs par articulation
  python view_model.py --gait     # anime la démarche tripode en l'air
"""
import argparse
import math
import time

import pybullet as pb
import pybullet_data

from hexapod import config as C
from hexapod.gait import TripodGait


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", action="store_true", help="animer la démarche au lieu des curseurs")
    args = ap.parse_args()

    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.loadURDF("plane.urdf")
    robot = pb.loadURDF(C.URDF_PATH, [0, 0, 0.25], useFixedBase=True)
    pb.resetDebugVisualizerCamera(0.6, 45, -25, [0, 0, 0.2])

    ids = {pb.getJointInfo(robot, j)[1].decode(): j for j in range(pb.getNumJoints(robot))}
    gait = TripodGait()
    q0 = gait.neutral()
    print(f"{len(ids)} articulations : {', '.join(C.JOINT_NAMES)}")

    sliders = {}
    if not args.gait:
        for name, q in zip(C.JOINT_NAMES, q0):
            lo, hi = C.JOINT_LIMITS[name.split("_")[1]]
            sliders[name] = pb.addUserDebugParameter(name, lo, hi, float(q))

    t = 0.0
    try:
        while pb.isConnected():
            if args.gait:
                t += 1 / 60
                phase = (t * C.GAIT_FREQ) % 1.0
                cmd = (C.MAX_VX * math.cos(t * 0.3), 0.0, 0.0)
                q = gait.joint_targets(phase, cmd)
                for name, v in zip(C.JOINT_NAMES, q):
                    pb.resetJointState(robot, ids[name], float(v))
            else:
                for name, s in sliders.items():
                    pb.resetJointState(robot, ids[name], pb.readUserDebugParameter(s))
            time.sleep(1 / 60)
    except (KeyboardInterrupt, pb.error):
        pass


if __name__ == "__main__":
    main()
