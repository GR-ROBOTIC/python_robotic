"""
Piloter le robot entraîné au clavier, dans la fenêtre MuJoCo.

    Flèche haut / bas       : avancer / reculer
    Flèche gauche / droite  : tourner à gauche / à droite
    Page préc. / Page suiv. : pas chassé à gauche / à droite
    Espace                  : s'arrêter (marche sur place)
    Entrée                  : remettre le robot debout

    python jouer.py
    python jouer.py --modele modeles/bipede_2000000_steps.zip \
                    --stats  modeles/bipede_vecnormalize_2000000_steps.pkl
"""
import argparse
import os
import queue

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from env_bipede import BipedeEnv

DOSSIER = os.path.dirname(os.path.abspath(__file__))
PAS_VITESSE = 0.05
PAS_ROTATION = 0.25

# Codes des touches (GLFW, utilisés par la fenêtre MuJoCo)
HAUT, BAS, GAUCHE, DROITE = 265, 264, 263, 262
PAGE_PREC, PAGE_SUIV, ESPACE, ENTREE = 266, 267, 32, 257

touches = queue.Queue()      # la fenêtre dépose les touches ici, la boucle principale les lit


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--modele", default=os.path.join(DOSSIER, "modeles", "bipede_final.zip"))
    parser.add_argument("--stats", default=os.path.join(DOSSIER, "modeles", "vecnormalize_final.pkl"))
    parser.add_argument("--randomiser", action="store_true")
    args = parser.parse_args()

    env = DummyVecEnv([lambda: BipedeEnv(render_mode="human", randomiser=args.randomiser,
                                         consigne_manuelle=True, touches=touches.put)])
    env = VecNormalize.load(args.stats, env)
    env.training = False
    env.norm_reward = False
    robot = env.venv.envs[0]
    model = PPO.load(args.modele, device="cpu")

    obs = env.reset()
    avant = lateral = rotation = 0.0
    print(__doc__)

    while True:
        # --- Clavier ---
        while not touches.empty():
            t = touches.get()
            if t == HAUT:        avant += PAS_VITESSE
            elif t == BAS:       avant -= PAS_VITESSE
            elif t == GAUCHE:    rotation += PAS_ROTATION
            elif t == DROITE:    rotation -= PAS_ROTATION
            elif t == PAGE_PREC: lateral += PAS_VITESSE
            elif t == PAGE_SUIV: lateral -= PAS_VITESSE
            elif t == ESPACE:    avant = lateral = rotation = 0.0
            elif t == ENTREE:    obs = env.reset()
        robot.set_consigne(avant, lateral, rotation)
        avant, lateral, rotation = robot.consigne

        # --- Le réseau décide ---
        action, _ = model.predict(obs, deterministic=True)
        obs, _, fini, infos = env.step(action)
        i = infos[0]
        if robot.viewer is None or not robot.viewer.is_running():
            break
        robot.viewer.set_texts((None, None,
                                "Consigne\nRéel",
                                f"avant {avant:+.2f} m/s   latéral {lateral:+.2f}   rotation {rotation:+.2f} rad/s\n"
                                f"avant {i['v_avant']:+.2f} m/s   latéral {i['v_lateral']:+.2f}   "
                                f"rotation {i['rotation']:+.2f} rad/s"))
        if fini[0]:
            print(f"Fin d'épisode ({'temps écoulé' if i.get('TimeLimit.truncated') else 'chute'}) : "
                  f"{i['distance']:.2f} m parcourus")
    env.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
