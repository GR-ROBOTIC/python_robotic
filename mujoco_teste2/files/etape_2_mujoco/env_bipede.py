"""
Environnement d'apprentissage de la marche avec MuJoCo — ÉTAPE 2 (directions).

Mêmes règles du jeu que la version PyBullet, pour que le cerveau exporté
fonctionne sur le MÊME programme ESP32 :

Observations (23 valeurs) :
    [0:3]   gravité vue par le robot (IMU)
    [3:6]   vitesse de rotation du buste (gyroscope de l'IMU)
    [6:18]  dernière commande envoyée aux 12 servos
    [18:20] horloge de marche sin/cos
    [20:23] consigne : vitesse avant, latérale, rotation

Actions (12 valeurs entre -1 et 1) : décalage de chaque servo autour de la pose de repos.

Test rapide (le robot doit tenir debout) :  python env_bipede.py
"""
import math
import os
import time

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

DOSSIER = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(DOSSIER, "bipede.xml")

ECHELLE_ACTION = 0.5     # rad : amplitude max autour de la pose de repos
VITESSE_SERVO = 7.0      # rad/s : vitesse max d'un MG996R
SOUS_PAS = 4             # 4 x 5 ms = 20 ms -> 50 décisions par seconde
PERIODE_PAS = 0.6        # s : cycle de marche complet
DUREE_MAX = 1000         # pas de décision par épisode (20 s)

CONSIGNE_MIN = np.array([-0.10, -0.06, -0.5])   # avant/arrière (m/s), latéral (m/s), rotation (rad/s)
CONSIGNE_MAX = np.array([0.20, 0.06, 0.5])
DUREE_CONSIGNE = 200     # pas (4 s) avant de changer de consigne


class BipedeEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, render_mode=None, randomiser=True, consigne_manuelle=False, touches=None):
        super().__init__()
        self.render_mode = render_mode
        self.randomiser = randomiser
        self.consigne_manuelle = consigne_manuelle
        self.touches = touches            # fonction appelée quand une touche est pressée (jouer.py)
        self.viewer = None

        self.m = mujoco.MjModel.from_xml_path(XML)
        self.d = mujoco.MjData(self.m)
        self.dt = self.m.opt.timestep * SOUS_PAS

        # Articulations pilotées par chaque servo (dans l'ordre des actionneurs = ordre de l'ESP32)
        self.n = self.m.nu
        joints = self.m.actuator_trnid[:, 0]
        self.noms = [self.m.joint(j).name for j in joints]
        self.qpos_adr = self.m.jnt_qposadr[joints]
        self.qvel_adr = self.m.jnt_dofadr[joints]
        self.bas, self.haut = self.m.jnt_range[joints, 0].copy(), self.m.jnt_range[joints, 1].copy()
        self.pose_repos = self.m.key_ctrl[0].copy()        # pose "repos" de bipede.xml

        # Capteurs (lus directement dans le tableau des mesures)
        def capteur(nom):
            s = self.m.sensor(nom).id
            a = self.m.sensor_adr[s]
            return slice(a, a + self.m.sensor_dim[s])
        self.c_orientation = capteur("orientation")
        self.c_gyro = capteur("gyro")
        self.c_pied_g = capteur("contact_pied_g")
        self.c_pied_d = capteur("contact_pied_d")

        # Valeurs d'origine, pour la randomisation
        self.masse0 = self.m.body_mass.copy()
        self.inertie0 = self.m.body_inertia.copy()
        self.friction0 = self.m.geom_friction.copy()
        self.force0 = self.m.actuator_forcerange.copy()
        self.sol = self.m.geom("sol").id
        self.semelles = [self.m.geom("semelle_g").id, self.m.geom("semelle_d").id]

        self.action_space = spaces.Box(-1.0, 1.0, (self.n,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (23,), dtype=np.float32)

        self.action_prec = np.zeros(self.n)
        self.cible_servo = self.pose_repos.copy()
        self.consigne = np.zeros(3)
        self.phase = 0.0
        self.compteur = 0

    # ------------------------------------------------------------------
    def set_consigne(self, avant, lateral, rotation):
        """Choisir la direction : avant (m/s), latéral (m/s, + = gauche), rotation (rad/s, + = gauche)."""
        self.consigne = np.clip([avant, lateral, rotation], CONSIGNE_MIN, CONSIGNE_MAX)

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

    def _rotation(self):
        """Matrice d'orientation du buste (repère robot -> monde), depuis le capteur d'orientation."""
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.d.sensordata[self.c_orientation])
        return R.reshape(3, 3)

    def _observation(self):
        R = self._rotation()
        gravite = R.T @ np.array([0, 0, -1.0])
        gyro = self.d.sensordata[self.c_gyro].copy()
        if self.randomiser:
            gravite = gravite + self.np_random.normal(0, 0.02, 3)
            gyro = gyro + self.np_random.normal(0, 0.05, 3)
        horloge = [math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)]
        return np.concatenate([gravite, gyro, self.action_prec, horloge, self.consigne]).astype(np.float32)

    # ------------------------------------------------------------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        m = self.m

        # Randomisation du domaine
        m.body_mass[:] = self.masse0
        m.body_inertia[:] = self.inertie0
        m.geom_friction[:] = self.friction0
        m.actuator_forcerange[:] = self.force0
        if self.randomiser:
            echelle = rng.uniform(0.85, 1.15, m.nbody)
            m.body_mass[:] = self.masse0 * echelle
            m.body_inertia[:] = self.inertie0 * echelle[:, None]
            friction = rng.uniform(0.5, 1.2)
            for g in [self.sol] + self.semelles:
                m.geom_friction[g, 0] = friction
            m.actuator_forcerange[:] = self.force0 * rng.uniform(0.8, 1.1)

        mujoco.mj_resetDataKeyframe(m, self.d, 0)
        if self.randomiser:
            self.d.qpos[self.qpos_adr] += rng.uniform(-0.05, 0.05, self.n)
        self.cible_servo = self.d.qpos[self.qpos_adr].copy()
        self.d.ctrl[:] = self.cible_servo
        mujoco.mj_forward(m, self.d)

        self.action_prec = np.zeros(self.n)
        self.phase = 0.0
        self.compteur = 0
        if not self.consigne_manuelle:
            self._tirer_consigne()
        return self._observation(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        cible = np.clip(self.pose_repos + ECHELLE_ACTION * action, self.bas, self.haut)

        # Le servo rejoint sa cible à 7 rad/s au maximum, comme un MG996R
        pas_max = VITESSE_SERVO * self.m.opt.timestep
        for _ in range(SOUS_PAS):
            self.cible_servo += np.clip(cible - self.cible_servo, -pas_max, pas_max)
            self.d.ctrl[:] = self.cible_servo
            mujoco.mj_step(self.m, self.d)
        energie = float(np.sum(np.abs(self.d.actuator_force * self.d.qvel[self.qvel_adr])))

        self.phase = (self.phase + self.dt / PERIODE_PAS) % 1.0
        self.compteur += 1

        # État du buste
        pos = self.d.qpos[0:3]
        R = self._rotation()
        lin = self.d.qvel[0:3]                       # vitesse dans le repère du monde
        ang = R @ self.d.qvel[3:6]                   # rotation exprimée dans le monde
        roulis = math.atan2(R[2, 1], R[2, 2])
        tangage = math.asin(max(-1.0, min(1.0, -R[2, 0])))
        lacet = math.atan2(R[1, 0], R[0, 0])
        v_avant = math.cos(lacet) * lin[0] + math.sin(lacet) * lin[1]
        v_lateral = -math.sin(lacet) * lin[0] + math.cos(lacet) * lin[1]
        c_avant, c_lateral, c_rotation = self.consigne

        # ---- Récompense : obéir à la consigne en restant stable (identique à PyBullet) ----
        erreur_v = (v_avant - c_avant) ** 2 + (v_lateral - c_lateral) ** 2
        r_vitesse = 2.0 * math.exp(-erreur_v / (2 * 0.07 ** 2))
        r_rotation = 1.0 * math.exp(-((ang[2] - c_rotation) ** 2) / (2 * 0.25 ** 2))
        r_vivant = 0.3
        r_droit = -1.0 * (roulis ** 2 + tangage ** 2)
        r_cap = -0.2 * ang[0] ** 2 - 0.2 * ang[1] ** 2
        r_hauteur = -5.0 * max(0.0, 0.28 - pos[2])
        r_douceur = -0.02 * float(np.sum((action - self.action_prec) ** 2))
        r_energie = -0.05 * energie

        # Rythme : les capteurs de contact sous les pieds disent quel pied touche le sol
        s = math.sin(2 * math.pi * self.phase)
        voulu_g, voulu_d = s <= 0.2, s >= -0.2
        au_sol_g = self.d.sensordata[self.c_pied_g][0] > 0.5
        au_sol_d = self.d.sensordata[self.c_pied_d][0] > 0.5
        r_rythme = 0.25 * ((au_sol_g == voulu_g) + (au_sol_d == voulu_d))

        recompense = (r_vitesse + r_rotation + r_vivant + r_droit + r_cap
                      + r_hauteur + r_douceur + r_energie + r_rythme)

        tombe = pos[2] < 0.2 or abs(roulis) > 0.7 or abs(tangage) > 0.7
        if tombe:
            recompense -= 5.0
        tronque = self.compteur >= DUREE_MAX

        self.action_prec = action
        if not self.consigne_manuelle and self.compteur % DUREE_CONSIGNE == 0:
            self._tirer_consigne()
        if self.render_mode == "human":
            self._afficher()

        info = {"distance": math.hypot(pos[0], pos[1]), "v_avant": v_avant,
                "v_lateral": v_lateral, "rotation": ang[2]}
        return self._observation(), float(recompense), tombe, tronque, info

    # ------------------------------------------------------------------
    def _afficher(self):
        import mujoco.viewer
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.m, self.d, key_callback=self.touches)
            with self.viewer.lock():
                self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                self.viewer.cam.trackbodyid = self.m.body("bassin").id
                self.viewer.cam.distance, self.viewer.cam.elevation, self.viewer.cam.azimuth = 1.2, -15, 135
            self._horloge = time.time()
        self.viewer.sync()
        self._horloge += self.dt                    # temps réel
        attente = self._horloge - time.time()
        if attente > 0:
            time.sleep(attente)
        else:
            self._horloge = time.time()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


if __name__ == "__main__":
    env = BipedeEnv(render_mode="human", randomiser=False, consigne_manuelle=True)
    obs, _ = env.reset()
    print("Observation de départ :", np.round(obs, 3))
    for t in range(500):
        obs, r, fin, _, _ = env.step(np.zeros(env.n))
        if fin:
            print(f"Tombé au pas {t}")
            break
    else:
        print("Le robot tient debout : l'environnement est prêt pour l'entraînement.")
    env.close()
