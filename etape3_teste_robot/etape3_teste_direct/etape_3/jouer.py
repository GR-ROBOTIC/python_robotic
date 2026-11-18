"""
Piloter le robot entraîné : marcher ET danser (cliquez d'abord dans la fenêtre PyBullet).

  Marche (réseau de neurones)          Danses et mouvements (chorégraphies)
    Flèche haut / bas : avancer / reculer    a Salut      e Balancement
    Flèche gauche / droite : tourner         b Victoire   f Vague
    Page préc. / suiv. : pas chassé          c Révérence  g Robot
    Espace : s'arrêter (ou arrêter la danse) d Flexions   h Disco
    Entrée : remettre le robot debout

À la fin d'une danse, le robot revient au repos puis le réseau reprend la marche.

    python jouer.py
    python jouer.py --modele modeles/bipede_2000000_steps.zip \
                    --stats  modeles/bipede_vecnormalize_2000000_steps.pkl
"""
import argparse
import os

import numpy as np
import pybullet as p
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from choregraphies import DANSES, Lecteur
from env_bipede import BipedeEnv
from robot_commun import DT

DOSSIER = os.path.dirname(os.path.abspath(__file__))
PAS_VITESSE = 0.05
PAS_ROTATION = 0.25


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--modele", default=os.path.join(DOSSIER, "modeles", "bipede_final.zip"))
    parser.add_argument("--stats", default=os.path.join(DOSSIER, "modeles", "vecnormalize_final.pkl"))
    parser.add_argument("--randomiser", action="store_true")
    args = parser.parse_args()

    env = DummyVecEnv([lambda: BipedeEnv(render_mode="human", randomiser=args.randomiser,
                                         consigne_manuelle=True)])
    env = VecNormalize.load(args.stats, env)
    env.training = False
    env.norm_reward = False
    robot = env.venv.envs[0]
    model = PPO.load(args.modele, device="cpu")

    obs = env.reset()
    avant = lateral = rotation = 0.0
    lecteur = None
    angles_actuels = robot.angles_commandes()     # les 16 angles envoyés au dernier pas
    texte = None
    print(__doc__)

    while robot.bc.isConnected():
        # ---------------- Clavier ----------------
        for t, etat in robot.bc.getKeyboardEvents().items():
            if not etat & p.KEY_WAS_TRIGGERED:
                continue
            touche = chr(t) if 0 < t < 256 else ""
            if touche in DANSES:
                lecteur = Lecteur(touche, angles_actuels, robot.bas, robot.haut)
                print(f"Danse : {lecteur.nom}")
            elif t == p.B3G_SPACE:
                if lecteur:
                    lecteur = Lecteur.retour_repos(angles_actuels, robot.bas, robot.haut)
                avant = lateral = rotation = 0.0
            elif t == p.B3G_RETURN:
                lecteur = None
                obs = env.reset()
                angles_actuels = robot.angles_commandes()
            elif t == p.B3G_UP_ARROW:    avant += PAS_VITESSE
            elif t == p.B3G_DOWN_ARROW:  avant -= PAS_VITESSE
            elif t == p.B3G_LEFT_ARROW:  rotation += PAS_ROTATION
            elif t == p.B3G_RIGHT_ARROW: rotation -= PAS_ROTATION
            elif t == p.B3G_PAGE_UP:     lateral += PAS_VITESSE
            elif t == p.B3G_PAGE_DOWN:   lateral -= PAS_VITESSE
        robot.set_consigne(avant, lateral, rotation)
        avant, lateral, rotation = robot.consigne

        # ---------------- Danse ou marche ----------------
        if lecteur:
            angles_actuels = lecteur.suivant(DT)
            robot.pas_libre(angles_actuels)
            if lecteur.fini:
                lecteur = None
                obs = env.normalize_obs(robot.reprendre_marche()[None, :])
            elif robot.bc.getBasePositionAndOrientation(robot.robot)[0][2] < 0.2:
                print("Chute pendant la danse !")
                lecteur = None
                obs = env.reset()
                angles_actuels = robot.angles_commandes()
            etat_txt = f"Danse : {lecteur.nom if lecteur else 'fin'}"
        else:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, fini, infos = env.step(action)
            angles_actuels = robot.angles_commandes()
            i = infos[0]
            if fini[0]:
                print(f"Fin d'épisode ({'temps écoulé' if i.get('TimeLimit.truncated') else 'chute'})")
            etat_txt = (f"Marche  consigne : avant {avant:+.2f} | latéral {lateral:+.2f} | rot {rotation:+.2f}"
                        f"   réel : {i['v_avant']:+.2f} | {i['v_lateral']:+.2f} | {i['rotation']:+.2f}")

        pos = robot.bc.getBasePositionAndOrientation(robot.robot)[0]
        kw = {"replaceItemUniqueId": texte} if texte is not None else {}
        texte = robot.bc.addUserDebugText(etat_txt, [pos[0], pos[1] - 0.3, 0.78],
                                          textColorRGB=[0, 0, 0], textSize=1.1, **kw)
    env.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
