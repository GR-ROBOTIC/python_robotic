"""
Piloter le robot entraîné au clavier (cliquez d'abord dans la fenêtre PyBullet).

    Flèche haut / bas      : avancer / reculer
    Flèche gauche / droite : tourner à gauche / à droite
    Page préc. / Page suiv. : pas chassé à gauche / à droite
    Espace                 : s'arrêter (marche sur place)

    python voir_robot.py
    python voir_robot.py --modele modeles/bipede_2000000_steps.zip \
                         --stats  modeles/bipede_vecnormalize_2000000_steps.pkl
"""
import argparse
import os

import pybullet as p
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from env_bipede import BipedeEnv, CONSIGNE_MAX, CONSIGNE_MIN

DOSSIER = os.path.dirname(os.path.abspath(__file__))
PAS_VITESSE = 0.05     # m/s par appui
PAS_ROTATION = 0.25    # rad/s par appui


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--modele", default=os.path.join(DOSSIER, "modeles", "bipede_final.zip"))
    parser.add_argument("--stats", default=os.path.join(DOSSIER, "modeles", "vecnormalize_final.pkl"))
    parser.add_argument("--randomiser", action="store_true", help="tester avec frottement/masses aléatoires")
    args = parser.parse_args()

    env = DummyVecEnv([lambda: BipedeEnv(render_mode="human", randomiser=args.randomiser,
                                         consigne_manuelle=True)])
    env = VecNormalize.load(args.stats, env)
    env.training = False
    env.norm_reward = False
    robot = env.venv.envs[0]          # accès direct à la simulation pour changer la consigne

    model = PPO.load(args.modele, device="cpu")
    obs = env.reset()
    avant = lateral = rotation = 0.0
    texte = None
    print(__doc__)

    try:
        while True:
            # --- Clavier ---
            touches = robot.bc.getKeyboardEvents()
            change = False
            for t, etat in touches.items():
                if not etat & p.KEY_WAS_TRIGGERED:
                    continue
                change = True
                if t == p.B3G_UP_ARROW:      avant += PAS_VITESSE
                elif t == p.B3G_DOWN_ARROW:  avant -= PAS_VITESSE
                elif t == p.B3G_LEFT_ARROW:  rotation += PAS_ROTATION
                elif t == p.B3G_RIGHT_ARROW: rotation -= PAS_ROTATION
                elif t == p.B3G_PAGE_UP:     lateral += PAS_VITESSE
                elif t == p.B3G_PAGE_DOWN:   lateral -= PAS_VITESSE
                elif t == p.B3G_SPACE:       avant = lateral = rotation = 0.0
                else:                        change = False
            if change:
                robot.set_consigne(avant, lateral, rotation)
                avant, lateral, rotation = robot.consigne     # valeurs bornées

            # --- Le réseau décide ---
            action, _ = model.predict(obs, deterministic=True)
            obs, _, fini, infos = env.step(action)
            i = infos[0]

            msg = (f"Consigne : avant {avant:+.2f} m/s | latéral {lateral:+.2f} | rotation {rotation:+.2f} rad/s"
                   f"    Réel : {i['v_avant']:+.2f} | {i['v_lateral']:+.2f} | {i['rotation']:+.2f}")
            pos = robot.bc.getBasePositionAndOrientation(robot.robot)[0]
            texte = robot.bc.addUserDebugText(msg, [pos[0], pos[1] - 0.3, 0.75], textColorRGB=[0, 0, 0],
                                              textSize=1.1, **({"replaceItemUniqueId": texte} if texte is not None else {}))
            if fini[0]:
                print(f"Fin d'épisode ({'temps écoulé' if i.get('TimeLimit.truncated') else 'chute'}) : "
                      f"{i['distance']:.2f} m parcourus")
                robot.set_consigne(avant, lateral, rotation)
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
