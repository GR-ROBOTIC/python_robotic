"""
Environnement d'apprentissage par renforcement pour le bipède à 12 servos MG996R.

Observations (23 valeurs) — uniquement ce qu'un vrai robot avec un ESP32 + une IMU peut connaître :
    [0:3]   gravité vue par le robot (inclinaison du buste, depuis l'IMU)
    [3:6]   vitesse de rotation du buste (gyroscope de l'IMU)
    [6:18]  dernière commande envoyée aux 12 servos (le MG996R ne renvoie pas sa position)
    [18:20] horloge de marche sin/cos (rythme des pas)
    [20:23] CONSIGNE de déplacement : vitesse avant, vitesse latérale, vitesse de rotation
            -> c'est le "volant" du robot : on lui dit où aller, il apprend à obéir.

Actions (12 valeurs entre -1 et 1) : décalage de chaque servo autour de la pose de repos.

Test rapide (le robot doit tenir debout, genoux légèrement fléchis) :
    python env_bipede.py
"""
import math
import os
import time

import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
from gymnasium import spaces
from pybullet_utils import bullet_client

DOSSIER = os.path.dirname(os.path.abspath(__file__))

# Pose de repos : genoux légèrement fléchis, pieds à plat (hanche + genou + cheville = 0)
POSE_REPOS = {"hanche_tangage": -0.3, "genou": 0.6, "cheville_tangage": -0.3}

ECHELLE_ACTION = 0.5     # rad : amplitude max autour de la pose de repos
COUPLE_SERVO = 1.0       # N·m (MG996R ~10 kg·cm à 6 V)
VITESSE_SERVO = 7.0      # rad/s
DT_SIM = 1 / 200         # pas de la physique
SOUS_PAS = 4             # -> la politique décide 50 fois par seconde
PERIODE_PAS = 0.6        # s, durée d'un cycle complet (pas gauche + pas droit)
DUREE_MAX = 1000         # pas de décision par épisode (= 20 s)
HAUTEUR_DEPART = 0.33

# Plages des consignes tirées au hasard pendant l'entraînement
#                avant/arrière (m/s)  latéral (m/s)  rotation (rad/s)
CONSIGNE_MIN = np.array([-0.10,          -0.06,         -0.5])
CONSIGNE_MAX = np.array([ 0.20,           0.06,          0.5])
DUREE_CONSIGNE = 200     # pas (= 4 s) avant de changer de consigne


class BipedeEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, render_mode=None, randomiser=True, consigne_manuelle=False, urdf=None):
        super().__init__()
        self.render_mode = render_mode
        self.randomiser = randomiser
        self.consigne_manuelle = consigne_manuelle   # True : la consigne vient du clavier
        self.consigne = np.zeros(3)
        self.dt = DT_SIM * SOUS_PAS

        mode = p.GUI if render_mode == "human" else p.DIRECT
        self.bc = bullet_client.BulletClient(connection_mode=mode)
        self.bc.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.bc.setGravity(0, 0, -9.81)
        self.bc.setTimeStep(DT_SIM)
        if render_mode == "human":
            self.bc.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        self.sol = self.bc.loadURDF("plane.urdf")
        chemin = urdf or os.path.join(DOSSIER, "bipede_12servos.urdf")
        self.robot = self.bc.loadURDF(chemin, [0, 0, HAUTEUR_DEPART],
                                      flags=p.URDF_USE_INERTIA_FROM_FILE)

        # Articulations motorisées, pieds, masses d'origine
        self.joints, self.bas, self.haut, repos = [], [], [], []
        self.pieds = {}
        for i in range(self.bc.getNumJoints(self.robot)):
            info = self.bc.getJointInfo(self.robot, i)
            nom_lien = info[12].decode()
            if nom_lien.startswith("pied_"):
                self.pieds[nom_lien] = i
            if info[2] == p.JOINT_REVOLUTE:
                nom = info[1].decode()
                self.joints.append(i)
                self.bas.append(info[8])
                self.haut.append(info[9])
                cle = nom.rsplit("_", 1)[0]          # "genou_g" -> "genou"
                repos.append(POSE_REPOS.get(cle, 0.0))
        self.bas, self.haut = np.array(self.bas), np.array(self.haut)
        self.pose_repos = np.array(repos)
        self.n = len(self.joints)
        self.liens = [-1] + list(range(self.bc.getNumJoints(self.robot)))
        self.masses = {l: self.bc.getDynamicsInfo(self.robot, l)[0] for l in self.liens}

        self.action_space = spaces.Box(-1.0, 1.0, (self.n,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (23,), dtype=np.float32)

        self.couple = COUPLE_SERVO
        self.action_prec = np.zeros(self.n)
        self.phase = 0.0
        self.compteur = 0

    # ------------------------------------------------------------------
    def set_consigne(self, avant, lateral, rotation):
        """Choisir la direction : vitesse avant (m/s), latérale (m/s, + = gauche), rotation (rad/s, + = gauche)."""
        self.consigne = np.clip([avant, lateral, rotation], CONSIGNE_MIN, CONSIGNE_MAX)

    def _tirer_consigne(self):
        rng = self.np_random
        if rng.random() < 0.15:                    # parfois : marcher sur place
            self.consigne = np.zeros(3)
            return
        c = rng.uniform(CONSIGNE_MIN, CONSIGNE_MAX)
        for k in range(3):                         # souvent une seule direction à la fois
            if rng.random() < 0.4:
                c[k] = 0.0
        self.consigne = c

    def _imu(self):
        """Gravité et gyroscope exprimés dans le repère du robot (comme une vraie IMU)."""
        _, orn = self.bc.getBasePositionAndOrientation(self.robot)
        _, ang = self.bc.getBaseVelocity(self.robot)
        R = np.array(self.bc.getMatrixFromQuaternion(orn)).reshape(3, 3)
        gravite = R.T @ np.array([0, 0, -1.0])
        gyro = R.T @ np.array(ang)
        return gravite, gyro

    def _observation(self):
        gravite, gyro = self._imu()
        if self.randomiser:   # bruit de capteur
            gravite = gravite + self.np_random.normal(0, 0.02, 3)
            gyro = gyro + self.np_random.normal(0, 0.05, 3)
        horloge = [math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)]
        return np.concatenate([gravite, gyro, self.action_prec, horloge, self.consigne]).astype(np.float32)

    def _commander(self, action):
        cibles = np.clip(self.pose_repos + ECHELLE_ACTION * action, self.bas, self.haut)
        for j, c in zip(self.joints, cibles):
            self.bc.setJointMotorControl2(self.robot, j, p.POSITION_CONTROL, targetPosition=c,
                                          force=self.couple, maxVelocity=VITESSE_SERVO)

    def _contact(self, pied):
        return len(self.bc.getContactPoints(self.robot, self.sol, linkIndexA=self.pieds[pied])) > 0

    # ------------------------------------------------------------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random

        # Randomisation du domaine : rend la politique robuste aux écarts sim -> réel
        if self.randomiser:
            for l in self.liens:
                self.bc.changeDynamics(self.robot, l, mass=self.masses[l] * rng.uniform(0.85, 1.15))
            friction = rng.uniform(0.5, 1.2)
            self.bc.changeDynamics(self.sol, -1, lateralFriction=friction)
            for pied in self.pieds.values():
                self.bc.changeDynamics(self.robot, pied, lateralFriction=friction)
            self.couple = COUPLE_SERVO * rng.uniform(0.8, 1.1)

        self.bc.resetBasePositionAndOrientation(self.robot, [0, 0, HAUTEUR_DEPART], [0, 0, 0, 1])
        self.bc.resetBaseVelocity(self.robot, [0, 0, 0], [0, 0, 0])
        bruit = rng.uniform(-0.05, 0.05, self.n) if self.randomiser else np.zeros(self.n)
        for j, q in zip(self.joints, self.pose_repos + bruit):
            self.bc.resetJointState(self.robot, j, q)

        self.action_prec = np.zeros(self.n)
        self.phase = 0.0
        self.compteur = 0
        if not self.consigne_manuelle:
            self._tirer_consigne()
        self._commander(self.action_prec)
        return self._observation(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        self._commander(action)

        energie = 0.0
        for _ in range(SOUS_PAS):
            self.bc.stepSimulation()
            for etat in self.bc.getJointStates(self.robot, self.joints):
                energie += abs(etat[1] * etat[3]) * DT_SIM   # |vitesse x couple|
            if self.render_mode == "human":
                time.sleep(DT_SIM)

        self.phase = (self.phase + self.dt / PERIODE_PAS) % 1.0
        self.compteur += 1

        pos, orn = self.bc.getBasePositionAndOrientation(self.robot)
        lin, ang = self.bc.getBaseVelocity(self.robot)
        roulis, tangage, lacet = self.bc.getEulerFromQuaternion(orn)

        # Vitesse exprimée dans le repère du robot (avant = là où il regarde)
        v_avant = math.cos(lacet) * lin[0] + math.sin(lacet) * lin[1]
        v_lateral = -math.sin(lacet) * lin[0] + math.cos(lacet) * lin[1]
        c_avant, c_lateral, c_rotation = self.consigne

        # ---- Récompense : obéir à la consigne ----
        erreur_v = (v_avant - c_avant) ** 2 + (v_lateral - c_lateral) ** 2
        r_vitesse = 2.0 * math.exp(-erreur_v / (2 * 0.07 ** 2))
        r_rotation = 1.0 * math.exp(-((ang[2] - c_rotation) ** 2) / (2 * 0.25 ** 2))
        r_vivant = 0.3
        r_droit = -1.0 * (roulis ** 2 + tangage ** 2)
        r_cap = -0.2 * ang[0] ** 2 - 0.2 * ang[1] ** 2   # pas de balancement inutile
        r_hauteur = -5.0 * max(0.0, 0.28 - pos[2])
        r_douceur = -0.02 * float(np.sum((action - self.action_prec) ** 2))
        r_energie = -0.05 * energie / self.dt

        # Rythme des pas : le pied en phase de vol doit être en l'air
        s = math.sin(2 * math.pi * self.phase)
        voulu_g, voulu_d = s <= 0.2, s >= -0.2          # True = pied au sol
        ok = (self._contact("pied_g") == voulu_g) + (self._contact("pied_d") == voulu_d)
        r_rythme = 0.25 * ok

        recompense = r_vitesse + r_rotation + r_vivant + r_droit + r_cap + r_hauteur + r_douceur + r_energie + r_rythme

        tombe = pos[2] < 0.2 or abs(roulis) > 0.7 or abs(tangage) > 0.7
        if tombe:
            recompense -= 5.0
        tronque = self.compteur >= DUREE_MAX

        self.action_prec = action
        if (not self.consigne_manuelle and self.compteur % DUREE_CONSIGNE == 0):
            self._tirer_consigne()                 # nouvelle direction en cours de route
        if self.render_mode == "human":
            self.bc.resetDebugVisualizerCamera(0.9, 50, -15, [pos[0], pos[1], 0.3])

        info = {"distance": math.hypot(pos[0], pos[1]), "v_avant": v_avant,
                "v_lateral": v_lateral, "rotation": ang[2]}
        return self._observation(), float(recompense), tombe, tronque, info

    def close(self):
        try:
            self.bc.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    # Test : actions nulles -> le robot doit rester debout dans sa pose de repos
    env = BipedeEnv(render_mode="human", randomiser=False, consigne_manuelle=True)
    obs, _ = env.reset()
    print("Observation de départ :", np.round(obs, 3))
    total = 0.0
    for t in range(500):
        obs, r, fin, tronque, info = env.step(np.zeros(env.n))
        total += r
        if fin:
            print(f"Tombé au pas {t}")
            break
    print(f"Récompense cumulée sur {t + 1} pas : {total:.1f} (robot immobile)")
    env.close()
