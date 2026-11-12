"""
Tester les danses et mouvements SANS modèle entraîné (le robot reste sur place).
Cliquez dans la fenêtre PyBullet, puis appuyez sur une touche :

    a Salut   b Victoire   c Révérence   d Flexions
    e Balancement   f Vague   g Robot   h Disco
    Espace : arrêter la danse      Entrée : remettre le robot debout

    python danser.py
"""
import numpy as np
import pybullet as p

from choregraphies import DANSES, REPOS, Lecteur, liste_touches
from env_bipede import BipedeEnv
from robot_commun import COUPLE_SERVO, DT


def main():
    env = BipedeEnv(render_mode="human", randomiser=False, consigne_manuelle=True)
    env.reset()
    bc = env.bc
    print(__doc__)
    lecteur = None
    angles = REPOS.copy()
    texte, pas, charge_max = None, 0, 0.0

    while bc.isConnected():
        for t, etat in bc.getKeyboardEvents().items():
            if not etat & p.KEY_WAS_TRIGGERED:
                continue
            touche = chr(t) if 0 < t < 256 else ""
            if touche in DANSES:
                lecteur = Lecteur(touche, angles, env.bas, env.haut)
                charge_max = 0.0
                print(f"-> {lecteur.nom} ({lecteur.duree:.1f} s)")
            elif t == p.B3G_SPACE and lecteur:
                lecteur = Lecteur.retour_repos(angles, env.bas, env.haut)
            elif t == p.B3G_RETURN:
                env.reset()
                lecteur, angles = None, REPOS.copy()

        if lecteur:
            angles = lecteur.suivant(DT)
            if lecteur.fini:
                print(f"   fin de {lecteur.nom} — servo le plus chargé : {100 * charge_max / COUPLE_SERVO:.0f} %")
                lecteur = None
        env.pas_libre(angles)

        couples = [abs(s[3]) for s in bc.getJointStates(env.robot, env.servos)]
        charge_max = max(charge_max, max(couples))
        pas += 1
        if pas % 10 == 0:
            msg = f"Danse : {lecteur.nom}" if lecteur else "Touches a à h : danses   (Entrée : relever)"
            kw = {"replaceItemUniqueId": texte} if texte is not None else {}
            pos = bc.getBasePositionAndOrientation(env.robot)[0]
            texte = bc.addUserDebugText(msg, [pos[0], pos[1] - 0.3, 0.75], textColorRGB=[0, 0, 0],
                                        textSize=1.3, **kw)


if __name__ == "__main__":
    print(liste_touches())
    main()
