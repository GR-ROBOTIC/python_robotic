"""
Voir le robot s'entraîner EN DIRECT, dans un deuxième terminal.

    Terminal 1 :  ./lancer.sh entrainer
    Terminal 2 :  ./lancer.sh direct        (ou : python suivre_entrainement.py)

Les robots de l'entraînement tournent sans fenêtre (sinon tout serait 10 à 100 fois plus lent).
Ce script affiche donc un robot à part, piloté par le cerveau en cours d'entraînement :
à chaque fois que l'entraînement améliore son cerveau, ce robot le récupère automatiquement.
Il vit dans les mêmes conditions que l'entraînement : consignes qui changent toutes les 4 s,
bras qui bougent au hasard.

Avec entrainer.py de cette version, le cerveau est mis à jour toutes les ~5 secondes.
Avec un entraînement lancé avant, il suit les sauvegardes (toutes les 500 000 étapes).
"""
import glob
import os
import pickle
import re
import time

import numpy as np
import pybullet as p
from stable_baselines3 import PPO

from env_bipede import BipedeEnv

DOSSIER = os.path.dirname(os.path.abspath(__file__))
MODELES = os.path.join(DOSSIER, "modeles")
VERIFIER_TOUTES = 2.0     # s : fréquence de recherche d'un nouveau cerveau


def trouver_cerveau():
    """Renvoie (modèle .zip, stats .pkl, nombre de pas, horodatage) du cerveau le plus récent."""
    direct = os.path.join(MODELES, "en_direct.zip")
    if os.path.exists(direct):
        try:
            pas = int(open(os.path.join(MODELES, "en_direct.txt")).read())
        except (OSError, ValueError):
            pas = 0
        return direct, os.path.join(MODELES, "en_direct.pkl"), pas, os.path.getmtime(direct)
    sauvegardes = glob.glob(os.path.join(MODELES, "bipede_*_steps.zip"))
    if sauvegardes:
        dernier = max(sauvegardes, key=lambda f: int(re.search(r"_(\d+)_steps", f).group(1)))
        pas = int(re.search(r"_(\d+)_steps", dernier).group(1))
        stats = os.path.join(MODELES, f"bipede_vecnormalize_{pas}_steps.pkl")
        if os.path.exists(stats):
            return dernier, stats, pas, os.path.getmtime(dernier)
    return None


def charger(zip_modele, pkl_stats):
    model = PPO.load(zip_modele, device="cpu")
    with open(pkl_stats, "rb") as f:
        stats = pickle.load(f)
    moyenne = stats.obs_rms.mean
    ecart = np.sqrt(stats.obs_rms.var + stats.epsilon)
    return model, moyenne, ecart, stats.clip_obs


def main():
    print(__doc__)
    print("En attente du premier cerveau dans modeles/ ...")
    while (source := trouver_cerveau()) is None:
        time.sleep(VERIFIER_TOUTES)

    env = BipedeEnv(render_mode="human", randomiser=False, consigne_manuelle=False)
    obs, _ = env.reset()
    bc = env.bc
    model, moyenne, ecart, clip = charger(source[0], source[1])
    horodatage, pas_cerveau = source[3], source[2]
    mise_a_jour = time.time()
    print(f"Cerveau chargé : {pas_cerveau:,} pas".replace(",", " "))

    texte = [None, None]                                  # identifiants des deux lignes de texte
    prochaine_verif = time.time() + VERIFIER_TOUTES
    essais, chutes, affichage = 1, 0, 0

    while bc.isConnected():
        # --- Un nouveau cerveau est-il disponible ? ---
        if time.time() > prochaine_verif:
            prochaine_verif = time.time() + VERIFIER_TOUTES
            nouveau = trouver_cerveau()
            if nouveau and nouveau[3] != horodatage:
                try:
                    model, moyenne, ecart, clip = charger(nouveau[0], nouveau[1])
                    horodatage, pas_cerveau = nouveau[3], nouveau[2]
                    mise_a_jour = time.time()
                    print(f"Nouveau cerveau : {pas_cerveau:,} pas".replace(",", " "))
                except Exception:
                    pass        # fichier en cours d'écriture : on réessaiera

        # --- Le cerveau décide, le robot bouge ---
        obs_norm = np.clip((obs - moyenne) / ecart, -clip, clip).astype(np.float32)
        action, _ = model.predict(obs_norm, deterministic=True)
        obs, _, tombe, tronque, info = env.step(action)
        if tombe or tronque:
            chutes += int(tombe)
            essais += 1
            obs, _ = env.reset()

        # --- Texte au-dessus du robot ---
        affichage += 1
        if affichage % 10 == 0:
            c = env.consigne
            msg = (f"Cerveau : {pas_cerveau / 1e6:.2f} M pas (il y a {time.time() - mise_a_jour:.0f} s)"
                   f"   essai {essais}, chutes {chutes}\n")
            msg2 = (f"Consigne : avant {c[0]:+.2f} | lat. {c[1]:+.2f} | rot. {c[2]:+.2f}"
                    f"    Réel : {info['v_avant']:+.2f} | {info['v_lateral']:+.2f} | {info['rotation']:+.2f}")
            pos = bc.getBasePositionAndOrientation(env.robot)[0]
            for i, (ligne, z) in enumerate(((msg, 0.85), (msg2, 0.78))):
                kw = {"replaceItemUniqueId": texte[i]} if texte[i] is not None else {}
                texte[i] = bc.addUserDebugText(ligne.strip(), [pos[0], pos[1] - 0.35, z],
                                               textColorRGB=[0, 0, 0], textSize=1.1, **kw)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, p.error):     # Ctrl+C ou fenêtre fermée
        pass
