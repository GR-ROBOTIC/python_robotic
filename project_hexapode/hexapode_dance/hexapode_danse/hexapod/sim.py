"""Simulation PyBullet : le robot, la piste de danse disco, la caméra et l'affichage."""
import colorsys
import random

import pybullet as pb
import pybullet_data
from pybullet_utils import bullet_client

from . import config as C


class DiscoFloor:
    """Piste de dalles colorées qui changent à chaque beat (purement visuelle)."""

    def __init__(self, p, n=9, size=0.13):
        self.p = p
        self.tiles = []
        half = size / 2 - 0.004
        shape = p.createVisualShape(pb.GEOM_BOX, halfExtents=[half, half, 0.001],
                                    rgbaColor=[0.2, 0.2, 0.2, 1])
        start = -(n - 1) * size / 2
        for i in range(n):
            for j in range(n):
                self.tiles.append(p.createMultiBody(
                    baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=shape,
                    basePosition=[start + i * size, start + j * size, 0.001]))
        self.flash()

    def flash(self):
        base = random.random()
        for t in self.tiles:
            if random.random() < 0.55:
                r, g, b = colorsys.hsv_to_rgb((base + random.random() * 0.35) % 1.0, 0.85, 1.0)
            else:
                r = g = b = 0.12
            self.p.changeVisualShape(t, -1, rgbaColor=[r, g, b, 1])


class Sim:
    def __init__(self, gui=True, disco=True):
        self.gui = gui
        self.p = bullet_client.BulletClient(connection_mode=pb.GUI if gui else pb.DIRECT)
        p = self.p
        if gui:
            p.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
            p.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / C.SIM_HZ)
        p.setPhysicsEngineParameter(numSolverIterations=50)
        self.plane = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(C.URDF_PATH, [0, 0, C.START_HEIGHT], flags=pb.URDF_USE_INERTIA_FROM_FILE)
        ids = {p.getJointInfo(self.robot, j)[1].decode(): j for j in range(p.getNumJoints(self.robot))}
        self.joint_ids = [ids[n] for n in C.JOINT_NAMES]
        for leg in C.LEGS:
            p.changeDynamics(self.robot, ids[f"{leg[0]}_tibia"], lateralFriction=1.0)
        self.disco = DiscoFloor(p) if (disco and gui) else None
        self._text = -1
        self.substeps = C.SIM_HZ // C.CONTROL_HZ
        self.dt = 1.0 / C.CONTROL_HZ
        if gui:
            p.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
            p.resetDebugVisualizerCamera(0.75, 35, -25, [0, 0, 0.08])

    def reset(self, q):
        p = self.p
        p.resetBasePositionAndOrientation(self.robot, [0, 0, C.START_HEIGHT], [0, 0, 0, 1])
        p.resetBaseVelocity(self.robot, [0, 0, 0], [0, 0, 0])
        for jid, v in zip(self.joint_ids, q):
            p.resetJointState(self.robot, jid, float(v), 0.0)
        for _ in range(150):
            self.apply(q)
            p.stepSimulation()

    def apply(self, q):
        n = len(self.joint_ids)
        self.p.setJointMotorControlArray(
            self.robot, self.joint_ids, pb.POSITION_CONTROL, targetPositions=[float(v) for v in q],
            forces=[C.EFFORT] * n, positionGains=[C.KP] * n, velocityGains=[C.KD] * n)

    def step(self, q):
        self.apply(q)
        for _ in range(self.substeps):
            self.p.stepSimulation()

    def pose(self):
        pos, orn = self.p.getBasePositionAndOrientation(self.robot)
        return pos, self.p.getEulerFromQuaternion(orn)

    def fallen(self):
        pos, (roll, pitch, _) = self.pose()
        return abs(roll) > 0.9 or abs(pitch) > 0.9 or pos[2] < 0.03

    def follow_camera(self):
        cam = self.p.getDebugVisualizerCamera()
        self.p.resetDebugVisualizerCamera(cam[10], cam[8], cam[9], self.pose()[0])

    def show_text(self, text):
        x, y, z = self.pose()[0]
        self._text = self.p.addUserDebugText(text, [x, y, z + 0.17], [0.05, 0.05, 0.05], textSize=1.5,
                                             replaceItemUniqueId=self._text)

    def body_color(self, rgba):
        self.p.changeVisualShape(self.robot, -1, rgbaColor=rgba)

    def start_video(self, path):
        return self.p.startStateLogging(pb.STATE_LOGGING_VIDEO_MP4, str(path))

    def stop_video(self, log_id):
        self.p.stopStateLogging(log_id)

    def close(self):
        try:
            self.p.disconnect()
        except pb.error:
            pass
