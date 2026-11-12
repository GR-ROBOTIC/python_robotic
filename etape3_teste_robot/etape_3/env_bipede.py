"""
Environnement d'apprentissage de la MARCHE pour le robot à 16 servos (étape 3).

Le réseau pilote les 12 servos des jambes. Les 4 servos des bras sont pilotés par le
programme : pendant l'entraînement ils bougent au hasard, pour que le robot apprenne
à garder l'équilibre quels que soient les mouvements des bras.

Observations (27 valeurs) — uniquement ce que l'ESP32 peut connaître :
    [0:3]   gravité vue par le robot (IMU)
    [3:6]   vitesse de rotation du buste (gyroscope de l'IMU)
    [6:18]  dernière commande envoyée aux 12 servos des jambes
    [18:20] horloge de marche sin/cos
    [20:23] consigne : vitesse avant, latérale, rotation
    [23:27] angles commandés aux 4 servos des bras

Actions (12 valeurs entre -1 et 1) : décalage de chaque servo de jambe autour de la pose de repos.

Test rapide (le robot doit tenir debout) :  python env_bipede.py
"""
import math
import time

import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
from gymnasium import spaces
from pybullet_utils import bullet_client

from robot_commun import (COUPLE_SERVO, DT, DT_SIM, HAUTEUR_DEPART, N_BRAS, N_JAMBES, POSE_REPOS,
                          SOUS_PAS, charger_robot, commander_servos, preparer_fenetre)

ECHELLE_ACTION = 0.5     # rad : amplitude max autour de la pose de repos
PERIODE_PAS = 0.6        # s : cycle de marche complet
DUREE_MAX = 1000         # pas de décision par épisode (20 s)

# Consignes tirées au hasard : avant/arrière (m/s), latéral (m/s), rotation (rad/s)
CONSIGNE_MIN = np.array([-0.10, -0.06, -0.5])
CONSIGNE_MAX = np.array([0.20, 0.06, 0.5])
DUREE_CONSIGNE = 200     # pas (4 s) avant de changer de consigne

VITESSE_BRAS_ALEATOIRE = 1.5   # rad/s : vitesse des mouvements de bras aléatoires
N_OBS = 27


class BipedeEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, render_mode=None, randomiser=True, consigne_manuelle=False):
        super().__init__()
        self.render_mode = render_mode
        self.randomiser = randomiser
        self.consigne_manuelle = consigne_manuelle
        self.dt = DT

        mode = p.GUI if render_mode == "human" else p.DIRECT
        self.bc = bullet_client.BulletClient(connection_mode=mode)
        self.bc.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.bc.setGravity(0, 0, -9.81)
        self.bc.setTimeStep(DT_SIM)
        self.bc.setPhysicsEngineParameter(numSolverIterations=10)   # ~2x plus rapide, reste stable
        if render_mode == "human":
            preparer_fenetre(self.bc)

        self.sol = self.bc.loadURDF("plane.urdf")
        self.robot, self.servos, self.bas, self.haut, self.pieds = charger_robot(self.bc)
        self.liens = [-1] + list(range(self.bc.getNumJoints(self.robot)))
        self.masses = {l: self.bc.getDynamicsInfo(self.robot, l)[0] for l in self.liens}

        self.action_space = spaces.Box(-1.0, 1.0, (N_JAMBES,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (N_OBS,), dtype=np.float32)

        self.couple = COUPLE_SERVO
        self.action_prec = np.zeros(N_JAMBES)
        self.consigne = np.zeros(3)
        self.bras = np.zeros(N_BRAS)            # angles commandés aux bras
        self.bras_cible = np.zeros(N_BRAS)
        self.phase = 0.0
        self.compteur = 0

    # ------------------------------------------------------------------
    #  Commandes extérieures (clavier, danses)
    # ------------------------------------------------------------------
    def set_consigne(self, avant, lateral, rotation):
        self.consigne = np.clip([avant, lateral, rotation], CONSIGNE_MIN, CONSIGNE_MAX)

    def set_bras(self, angles4):
        """Fixe la position des bras (utilisé hors entraînement)."""
        self.bras = np.clip(angles4, self.bas[N_JAMBES:], self.haut[N_JAMBES:])
        self.bras_cible = self.bras.copy()

    def angles_commandes(self):
        """Les 16 angles actuellement demandés aux servos."""
        jambes = np.clip(POSE_REPOS[:N_JAMBES] + ECHELLE_ACTION * self.action_prec,
                         self.bas[:N_JAMBES], self.haut[:N_JAMBES])
        return np.concatenate([jambes, self.bras])

    def pas_libre(self, angles16):
        """Avance la physique de 20 ms en imposant directement les 16 angles (danses)."""
        commander_servos(self.bc, self.robot, self.servos, angles16, self.couple)
        for _ in range(SOUS_PAS):
            self.bc.stepSimulation()
            if self.render_mode == "human":
                time.sleep(DT_SIM)
        self._suivre_camera()

    def reprendre_marche(self):
        """Après une danse : le réseau reprend la main depuis la pose de repos."""
        self.action_prec = np.zeros(N_JAMBES)
        self.bras = np.zeros(N_BRAS)
        self.bras_cible = np.zeros(N_BRAS)
        self.phase = 0.0
        return self._observation()

    # ------------------------------------------------------------------
    def _tirer_consigne(self):
        rng = self.np_random
        if rng.random() < 0.15:
            self.consigne = np.zeros(3)
            return
        c = rng.uniform(CONSIGNE_MIN, CONSIGNE_MAX)
        for k in range(3):
            if rng.random() < 0.4:
                c[k] = 0.0
        self.consigne = c

    def _bouger_bras_au_hasard(self):
        rng = self.np_random
        if rng.random() < 1 / 75:                           # nouvelle cible ~ toutes les 1,5 s
            if rng.random() < 0.3:
                self.bras_cible = np.zeros(N_BRAS)
            else:
                lo, hi = self.bas[N_JAMBES:], self.haut[N_JAMBES:]
                self.bras_cible = rng.uniform(lo, hi) * 0.8
        pas = VITESSE_BRAS_ALEATOIRE * self.dt
        self.bras = self.bras + np.clip(self.bras_cible - self.bras, -pas, pas)

    def _imu(self):
        _, orn = self.bc.getBasePositionAndOrientation(self.robot)
        _, ang = self.bc.getBaseVelocity(self.robot)
        R = np.array(self.bc.getMatrixFromQuaternion(orn)).reshape(3, 3)
        return R.T @ np.array([0, 0, -1.0]), R.T @ np.array(ang)

    def _observation(self):
        gravite, gyro = self._imu()
        if self.randomiser:
            gravite = gravite + self.np_random.normal(0, 0.02, 3)
            gyro = gyro + self.np_random.normal(0, 0.05, 3)
        horloge = [math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)]
        return np.concatenate([gravite, gyro, self.action_prec, horloge,
                               self.consigne, self.bras]).astype(np.float32)

    def _contact(self, pied):
        return len(self.bc.getContactPoints(self.robot, self.sol, linkIndexA=self.pieds[pied])) > 0

    def _suivre_camera(self):
        if self.render_mode == "human":
            pos = self.bc.getBasePositionAndOrientation(self.robot)[0]
            self.bc.resetDebugVisualizerCamera(1.0, 50, -15, [pos[0], pos[1], 0.3])

    # ------------------------------------------------------------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
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
        bruit = rng.uniform(-0.05, 0.05, len(self.servos)) if self.randomiser else 0.0
        for j, q in zip(self.servos, POSE_REPOS + bruit):
            self.bc.resetJointState(self.robot, j, q)

        self.action_prec = np.zeros(N_JAMBES)
        self.bras = np.zeros(N_BRAS)
        self.bras_cible = np.zeros(N_BRAS)
        self.phase = 0.0
        self.compteur = 0
        if not self.consigne_manuelle:
            self._tirer_consigne()
        commander_servos(self.bc, self.robot, self.servos, self.angles_commandes(), self.couple)
        return self._observation(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if not self.consigne_manuelle:
            self._bouger_bras_au_hasard()
        cible_jambes = np.clip(POSE_REPOS[:N_JAMBES] + ECHELLE_ACTION * action,
                               self.bas[:N_JAMBES], self.haut[:N_JAMBES])
        commander_servos(self.bc, self.robot, self.servos,
                         np.concatenate([cible_jambes, self.bras]), self.couple)

        energie = 0.0
        jambes = self.servos[:N_JAMBES]
        for _ in range(SOUS_PAS):
            self.bc.stepSimulation()
            if self.render_mode == "human":
                time.sleep(DT_SIM)
        for etat in self.bc.getJointStates(self.robot, jambes):
            energie += abs(etat[1] * etat[3])                 # puissance instantanée (W)

        self.phase = (self.phase + self.dt / PERIODE_PAS) % 1.0
        self.compteur += 1

        pos, orn = self.bc.getBasePositionAndOrientation(self.robot)
        lin, ang = self.bc.getBaseVelocity(self.robot)
        roulis, tangage, lacet = self.bc.getEulerFromQuaternion(orn)
        v_avant = math.cos(lacet) * lin[0] + math.sin(lacet) * lin[1]
        v_lateral = -math.sin(lacet) * lin[0] + math.cos(lacet) * lin[1]
        c_avant, c_lateral, c_rotation = self.consigne

        # ---- Récompense : obéir à la consigne en restant stable ----
        erreur_v = (v_avant - c_avant) ** 2 + (v_lateral - c_lateral) ** 2
        r_vitesse = 2.0 * math.exp(-erreur_v / (2 * 0.07 ** 2))
        r_rotation = 1.0 * math.exp(-((ang[2] - c_rotation) ** 2) / (2 * 0.25 ** 2))
        r_vivant = 0.3
        r_droit = -1.0 * (roulis ** 2 + tangage ** 2)
        r_cap = -0.2 * ang[0] ** 2 - 0.2 * ang[1] ** 2
        r_hauteur = -5.0 * max(0.0, 0.28 - pos[2])
        r_douceur = -0.02 * float(np.sum((action - self.action_prec) ** 2))
        r_energie = -0.05 * energie

        s = math.sin(2 * math.pi * self.phase)
        voulu_g, voulu_d = s <= 0.2, s >= -0.2
        ok = (self._contact("pied_g") == voulu_g) + (self._contact("pied_d") == voulu_d)
        r_rythme = 0.25 * ok

        recompense = (r_vitesse + r_rotation + r_vivant + r_droit + r_cap
                      + r_hauteur + r_douceur + r_energie + r_rythme)

        tombe = pos[2] < 0.2 or abs(roulis) > 0.7 or abs(tangage) > 0.7
        if tombe:
            recompense -= 5.0
        tronque = self.compteur >= DUREE_MAX

        self.action_prec = action
        if not self.consigne_manuelle and self.compteur % DUREE_CONSIGNE == 0:
            self._tirer_consigne()
        self._suivre_camera()

        info = {"distance": math.hypot(pos[0], pos[1]), "v_avant": v_avant,
                "v_lateral": v_lateral, "rotation": ang[2]}
        return self._observation(), float(recompense), tombe, tronque, info

    def close(self):
        try:
            self.bc.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    env = BipedeEnv(render_mode="human", randomiser=False, consigne_manuelle=True)
    obs, _ = env.reset()
    print("Observation de départ :", np.round(obs, 3))
    for t in range(500):
        obs, r, fin, _, _ = env.step(np.zeros(N_JAMBES))
        if fin:
            print(f"Tombé au pas {t}")
            break
    else:
        print("Le robot tient debout : l'environnement est prêt pour l'entraînement.")
    env.close()
