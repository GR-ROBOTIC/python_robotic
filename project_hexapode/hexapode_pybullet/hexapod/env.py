"""Environnement Gymnasium de l'hexapode dans PyBullet.

Action      : 18 corrections dans [-1, 1], ajoutées aux angles de la démarche tripode.
Observation : roulis, tangage, vitesses du corps, hauteur, angles et vitesses des servos,
              commande demandée, phase de la démarche, action précédente.
Récompense  : suivre la commande (vitesse + rotation) en restant stable et économe.
"""
import math

import gymnasium as gym
import numpy as np
import pybullet as pb
import pybullet_data
from gymnasium import spaces
from pybullet_utils import bullet_client

from . import config as C
from .gait import TripodGait


class HexapodEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": C.CONTROL_HZ}

    def __init__(self, render_mode=None, max_steps=600, residual_scale=0.25,
                 randomize=True, command=None):
        self.render_mode = render_mode
        self.max_steps = max_steps
        self.residual_scale = residual_scale
        self.randomize = randomize
        self.fixed_command = None if command is None else np.asarray(command, np.float32)

        self.gait = TripodGait()
        self.q_neutral = self.gait.neutral()
        self.n_j = len(C.JOINT_NAMES)
        self.action_repeat = C.SIM_HZ // C.CONTROL_HZ
        self.sim_dt = 1.0 / C.SIM_HZ
        self.control_dt = 1.0 / C.CONTROL_HZ
        self.cmd_scale = np.array(C.CMD_SCALE, np.float32)
        self.lower = np.array([C.JOINT_LIMITS[n.split("_")[1]][0] for n in C.JOINT_NAMES], np.float32)
        self.upper = np.array([C.JOINT_LIMITS[n.split("_")[1]][1] for n in C.JOINT_NAMES], np.float32)

        self.action_space = spaces.Box(-1.0, 1.0, (self.n_j,), np.float32)
        obs_dim = 2 + 3 + 3 + 1 + self.n_j + self.n_j + 3 + 2 + self.n_j
        self.observation_space = spaces.Box(-np.inf, np.inf, (obs_dim,), np.float32)

        self.p = None
        self.nominal_height = None
        self.command = np.zeros(3, np.float32)
        self.cmd_smooth = np.zeros(3, np.float32)
        self._text_id = -1

    # ------------------------------------------------------------ installation
    def _setup(self):
        mode = pb.GUI if self.render_mode == "human" else pb.DIRECT
        self.p = bullet_client.BulletClient(connection_mode=mode)
        p = self.p
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.sim_dt)
        p.setPhysicsEngineParameter(numSolverIterations=50)
        self.plane = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(C.URDF_PATH, [0, 0, C.START_HEIGHT],
                                flags=pb.URDF_USE_INERTIA_FROM_FILE)
        name_to_id = {}
        for j in range(p.getNumJoints(self.robot)):
            name_to_id[p.getJointInfo(self.robot, j)[1].decode()] = j
        self.joint_ids = [name_to_id[n] for n in C.JOINT_NAMES]
        self.foot_links = [name_to_id[f"{leg[0]}_tibia"] for leg in C.LEGS]
        for link in self.foot_links:
            p.changeDynamics(self.robot, link, lateralFriction=1.0)
        if self.render_mode == "human":
            p.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 1)
            p.resetDebugVisualizerCamera(0.7, 40, -30, [0, 0, 0.05])

    def _apply(self, q):
        self.p.setJointMotorControlArray(
            self.robot, self.joint_ids, pb.POSITION_CONTROL,
            targetPositions=q.tolist(),
            forces=[C.EFFORT] * self.n_j,
            positionGains=[C.KP] * self.n_j,
            velocityGains=[C.KD] * self.n_j)

    def _sample_command(self):
        r = self.np_random.random()
        if r < 0.10:
            return np.zeros(3, np.float32)
        if r < 0.70:
            names = [k for k in C.COMMANDS if k != "stop"]
            return np.array(C.COMMANDS[names[self.np_random.integers(len(names))]], np.float32)
        c = self.np_random.uniform(-1, 1, 3).astype(np.float32)
        c[self.np_random.random(3) < 0.4] = 0.0
        return c

    def set_command(self, cmd):
        """Impose une commande normalisée (vx, vy, rotation) dans [-1, 1]."""
        self.fixed_command = np.clip(np.asarray(cmd, np.float32), -1, 1)
        self.command = self.fixed_command.copy()

    # ------------------------------------------------------------ gym API
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self.p is None:
            self._setup()
        p = self.p
        yaw = self.np_random.uniform(-math.pi, math.pi) if self.randomize else 0.0
        p.resetBasePositionAndOrientation(self.robot, [0, 0, C.START_HEIGHT],
                                          p.getQuaternionFromEuler([0, 0, yaw]))
        p.resetBaseVelocity(self.robot, [0, 0, 0], [0, 0, 0])
        for jid, q in zip(self.joint_ids, self.q_neutral):
            p.resetJointState(self.robot, jid, float(q), 0.0)
        if self.randomize:
            mu = float(self.np_random.uniform(0.7, 1.3))
            for link in self.foot_links:
                p.changeDynamics(self.robot, link, lateralFriction=mu)
        for _ in range(120):                       # le robot se pose au sol
            self._apply(self.q_neutral)
            p.stepSimulation()
        if self.nominal_height is None:
            self.nominal_height = p.getBasePositionAndOrientation(self.robot)[0][2]

        self.phase = 0.0
        self.steps = 0
        self.prev_action = np.zeros(self.n_j, np.float32)
        self.command = self.fixed_command.copy() if self.fixed_command is not None else self._sample_command()
        self.cmd_smooth = np.zeros(3, np.float32)
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, np.float32), -1.0, 1.0)
        if self.fixed_command is None and self.randomize and self.np_random.random() < 1 / 250:
            self.command = self._sample_command()
        # lissage de la commande : évite les sauts de pieds quand on change de direction
        self.cmd_smooth += (self.command - self.cmd_smooth) * 0.15
        cmd_real = self.cmd_smooth * self.cmd_scale

        for _ in range(self.action_repeat):
            self.phase = (self.phase + C.GAIT_FREQ * self.sim_dt) % 1.0
            q = self.gait.joint_targets(self.phase, cmd_real) + a * self.residual_scale
            self._apply(np.clip(q, self.lower, self.upper))
            self.p.stepSimulation()
        self.steps += 1

        obs = self._obs(a)
        roll, pitch, z = self._roll, self._pitch, self._z
        v_b, w_b = self._v_body, self._w_body
        target = self.command * self.cmd_scale

        lin_err = float(np.sum((v_b[:2] - target[:2]) ** 2))
        yaw_err = float((w_b[2] - target[2]) ** 2)
        r_track = 1.0 * math.exp(-lin_err / 0.004) + 0.5 * math.exp(-yaw_err / 0.04)
        r_stab = -2.0 * (roll ** 2 + pitch ** 2) - 0.05 * float(w_b[0] ** 2 + w_b[1] ** 2)
        r_height = -3.0 * abs(z - self.nominal_height) - 2.0 * float(v_b[2] ** 2)
        r_act = -0.02 * float(np.mean(a ** 2)) - 0.05 * float(np.mean((a - self.prev_action) ** 2))
        r_energy = -0.002 * float(np.sum(np.abs(self._tau * self._qd)))
        reward = r_track + r_stab + r_height + r_act + r_energy + 0.2

        terminated = abs(roll) > 0.7 or abs(pitch) > 0.7 or z < 0.035
        if terminated:
            reward -= 5.0
        truncated = self.steps >= self.max_steps
        self.prev_action = a
        info = {"vx": float(v_b[0]), "vy": float(v_b[1]), "wz": float(w_b[2]),
                "roll": roll, "pitch": pitch, "height": z, "command": self.command.copy()}
        return obs, float(reward), terminated, truncated, info

    def _obs(self, action=None):
        p = self.p
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        roll, pitch, _ = p.getEulerFromQuaternion(orn)
        lin, ang = p.getBaseVelocity(self.robot)
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        v_b = R.T @ np.array(lin)
        w_b = R.T @ np.array(ang)
        js = p.getJointStates(self.robot, self.joint_ids)
        q = np.array([s[0] for s in js], np.float32)
        qd = np.array([s[1] for s in js], np.float32)
        self._tau = np.array([s[3] for s in js], np.float32)
        self._qd = qd
        self._roll, self._pitch, self._z = roll, pitch, pos[2]
        self._v_body, self._w_body = v_b, w_b
        prev = self.prev_action if action is None else action
        ph = 2 * math.pi * self.phase
        return np.concatenate([
            [roll, pitch], w_b, v_b, [pos[2] - (self.nominal_height or pos[2])],
            q - self.q_neutral, qd * 0.1, self.command, [math.sin(ph), math.cos(ph)], prev,
        ]).astype(np.float32)

    # ------------------------------------------------------------ outils d'affichage
    def base_position(self):
        return self.p.getBasePositionAndOrientation(self.robot)[0]

    def follow_camera(self):
        cam = self.p.getDebugVisualizerCamera()
        self.p.resetDebugVisualizerCamera(cam[10], cam[8], cam[9], self.base_position())

    def show_text(self, text, color=(0.1, 0.1, 0.1)):
        x, y, z = self.base_position()
        self._text_id = self.p.addUserDebugText(text, [x, y, z + 0.15], color, textSize=1.4,
                                                replaceItemUniqueId=self._text_id)

    def close(self):
        if self.p is not None:
            try:
                self.p.disconnect()
            except pb.error:
                pass
            self.p = None
