"""
Exporte vers l'ESP32 (dossier robot_esp32/) :
  - danses.h     : les 16 servos (ordre, pose de repos, limites) + toutes les chorégraphies
                   -> ne demande AUCUN entraînement
  - politique.h  : le cerveau de marche (réseau PPO), si un modèle entraîné existe

    python exporter_esp32.py              # les deux (ou seulement les danses s'il n'y a pas de modèle)
    python exporter_esp32.py --danses     # seulement les danses
    python exporter_esp32.py --modele modeles/bipede_3000000_steps.zip \
                             --stats  modeles/bipede_vecnormalize_3000000_steps.pkl
"""
import argparse
import os
import pickle

import numpy as np

from choregraphies import DANSES, DUREE_ENTREE, DUREE_SORTIE, images_en_secondes
from robot_commun import DT, N_JAMBES, N_SERVOS, ORDRE_SERVOS, POSE_REPOS

DOSSIER = os.path.dirname(os.path.abspath(__file__))
SORTIE = os.path.join(DOSSIER, "robot_esp32")


def tableau_c(nom, valeurs, par_ligne=8, type_c="float"):
    v = np.asarray(valeurs, dtype=np.float32).ravel()
    lignes = ["  " + ", ".join(f"{x:.8e}f" for x in v[i:i + par_ligne]) for i in range(0, len(v), par_ligne)]
    return f"static const {type_c} {nom}[{len(v)}] = {{\n" + ",\n".join(lignes) + "\n};\n"


def entete(titre):
    return ("// ============================================================\n"
            f"//  {titre}\n"
            "//  FICHIER GÉNÉRÉ AUTOMATIQUEMENT par exporter_esp32.py — ne pas modifier à la main.\n"
            "// ============================================================\n#pragma once\n\n")


# ----------------------------------------------------------------------
#  Danses (aucun entraînement nécessaire)
# ----------------------------------------------------------------------
def exporter_danses(bas, haut):
    t = [entete("Servos et chorégraphies")]
    t.append(f"#define N_SERVOS {N_SERVOS}\n#define N_JAMBES {N_JAMBES}\n\n")
    t.append(f"static const float DT_CONTROLE = {DT:.6f}f;   // s (50 Hz)\n")
    t.append(f"static const float DUREE_ENTREE = {DUREE_ENTREE:.3f}f;\n")
    t.append(f"static const float DUREE_SORTIE = {DUREE_SORTIE:.3f}f;\n\n")
    t.append("// Ordre officiel des servos :\n")
    t += [f"//   {i:2d} : {n}\n" for i, n in enumerate(ORDRE_SERVOS)]
    t.append("\n" + tableau_c("POSE_REPOS", POSE_REPOS) + tableau_c("LIM_BAS", bas) + tableau_c("LIM_HAUT", haut))

    t.append("\nstruct Danse {\n  char touche;\n  const char* nom;\n  int n_images;\n"
             "  const float* temps;    // s, depuis la 1re image\n"
             "  const float* angles;   // n_images x N_SERVOS\n};\n\n")
    lignes = []
    for touche, d in DANSES.items():
        images = images_en_secondes(d)
        temps = [ti for ti, _ in images]
        angles = np.clip(np.array([a for _, a in images]), bas, haut)
        t.append(f"// {touche} : {d['nom']} ({d['type']}, {d['tempo']} BPM, {d['boucles']} boucle(s))\n")
        t.append(tableau_c(f"DANSE_{touche}_TEMPS", temps))
        t.append(tableau_c(f"DANSE_{touche}_ANGLES", angles, par_ligne=N_SERVOS // 2))
        nom_c = d["nom"].replace('"', "")
        lignes.append(f'  {{ \'{touche}\', "{nom_c}", {len(images)}, DANSE_{touche}_TEMPS, DANSE_{touche}_ANGLES }}')
    t.append(f"\n#define N_DANSES {len(DANSES)}\n")
    t.append("static const Danse DANSES[N_DANSES] = {\n" + ",\n".join(lignes) + "\n};\n")

    chemin = os.path.join(SORTIE, "danses.h")
    os.makedirs(SORTIE, exist_ok=True)
    with open(chemin, "w", encoding="utf-8") as f:
        f.write("".join(t))
    print(f"Danses exportées : {chemin}  ({len(DANSES)} chorégraphies)")


# ----------------------------------------------------------------------
#  Cerveau de marche
# ----------------------------------------------------------------------
def politique_numpy(obs, moyenne, ecart, clip, couches):
    x = np.clip((obs - moyenne) / ecart, -clip, clip)
    for k, (W, b) in enumerate(couches):
        x = W @ x + b
        if k < len(couches) - 1:
            x = np.tanh(x)
    return np.clip(x, -1.0, 1.0)


def exporter_politique(chemin_modele, chemin_stats):
    import torch.nn as nn
    from stable_baselines3 import PPO
    import env_bipede
    from env_bipede import BipedeEnv

    print(f"Chargement de {chemin_modele}")
    model = PPO.load(chemin_modele, device="cpu")
    with open(chemin_stats, "rb") as f:
        stats = pickle.load(f)
    moyenne = stats.obs_rms.mean.astype(np.float64)
    ecart = np.sqrt(stats.obs_rms.var + stats.epsilon)
    clip = float(stats.clip_obs)

    pi = model.policy.mlp_extractor.policy_net
    lineaires = [m for m in pi if isinstance(m, nn.Linear)] + [model.policy.action_net]
    if len(lineaires) != 3 or not all(isinstance(m, nn.Tanh) for m in pi if not isinstance(m, nn.Linear)):
        raise SystemExit("Réseau attendu : 2 couches cachées avec tanh (réglage de entrainer.py).")
    couches = [(l.weight.detach().numpy().astype(np.float64), l.bias.detach().numpy().astype(np.float64))
               for l in lineaires]
    if couches[0][0].shape[1] != env_bipede.N_OBS:
        raise SystemExit(f"Ce modèle a {couches[0][0].shape[1]} entrées au lieu de {env_bipede.N_OBS} : "
                         "il vient d'une autre étape. Entraînez un modèle avec cette étape.")

    # Vérification : le calcul manuel pilote le robot en simulation
    env = BipedeEnv(randomiser=False, consigne_manuelle=True)
    env.set_consigne(0.15, 0.0, 0.0)
    obs, _ = env.reset(seed=0)
    ecart_max, test = 0.0, None
    for t in range(500):
        a_np = politique_numpy(obs, moyenne, ecart, clip, couches)
        a_sb3, _ = model.predict(np.clip((obs - moyenne) / ecart, -clip, clip).astype(np.float32),
                                 deterministic=True)
        ecart_max = max(ecart_max, float(np.max(np.abs(a_np - a_sb3))))
        if t == 100 or test is None:
            test = (obs.copy(), a_np.copy())
        obs, _, fin, tronque, info = env.step(a_np)
        if fin or tronque:
            break
    env.close()
    print(f"Écart calcul manuel / PyTorch : {ecart_max:.2e} (doit être < 1e-4)")
    print(f"Test en simulation (tout droit à 0,15 m/s) : {info['distance']:.2f} m en {(t + 1) * DT:.1f} s"
          + ("  -> CHUTE" if fin else ""))

    (W1, B1), (W2, B2), (W3, B3) = couches
    txt = [entete(f"Cerveau de marche — modèle {os.path.basename(chemin_modele)}")]
    txt.append(f"#define N_ENTREE {W1.shape[1]}\n#define N_C1 {W1.shape[0]}\n"
               f"#define N_C2 {W2.shape[0]}\n#define N_SORTIE {W3.shape[0]}\n\n")
    txt.append(f"static const float OBS_CLIP = {clip:.6f}f;\n")
    txt.append(f"static const float ECHELLE_ACTION = {env_bipede.ECHELLE_ACTION:.6f}f;   // rad\n")
    txt.append(f"static const float PERIODE_PAS = {env_bipede.PERIODE_PAS:.6f}f;      // s\n\n")
    txt.append("// Consignes vues à l'entraînement : avant (m/s), latéral (m/s), rotation (rad/s)\n")
    txt.append(tableau_c("CONSIGNE_MIN", env_bipede.CONSIGNE_MIN) + tableau_c("CONSIGNE_MAX", env_bipede.CONSIGNE_MAX))
    txt.append("\n// Normalisation des observations\n" + tableau_c("OBS_MOYENNE", moyenne) + tableau_c("OBS_ECART", ecart))
    txt.append("\n// Poids du réseau ([sortie][entrée])\n")
    for nom, arr in (("W1", W1), ("B1", B1), ("W2", W2), ("B2", B2), ("W3", W3), ("B3", B3)):
        txt.append(tableau_c(nom, arr))
    txt.append("\n// Vecteur de test (autotest au démarrage de l'ESP32)\n")
    txt.append(tableau_c("TEST_OBS", test[0]) + tableau_c("TEST_ACTION", test[1]))
    chemin = os.path.join(SORTIE, "politique.h")
    with open(chemin, "w", encoding="utf-8") as f:
        f.write("".join(txt))
    print(f"Cerveau exporté : {chemin}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--danses", action="store_true", help="exporter seulement les danses")
    parser.add_argument("--modele", default=os.path.join(DOSSIER, "modeles", "bipede_final.zip"))
    parser.add_argument("--stats", default=os.path.join(DOSSIER, "modeles", "vecnormalize_final.pkl"))
    args = parser.parse_args()

    import pybullet as p
    from robot_commun import charger_robot
    from pybullet_utils import bullet_client
    bc = bullet_client.BulletClient(connection_mode=p.DIRECT)
    _, _, bas, haut, _ = charger_robot(bc)
    bc.disconnect()

    exporter_danses(bas, haut)
    if args.danses:
        return
    if os.path.exists(args.modele) and os.path.exists(args.stats):
        exporter_politique(args.modele, args.stats)
    else:
        print("Pas de modèle entraîné trouvé : seules les danses sont exportées.\n"
              "Le robot pourra danser, la marche sera disponible après l'entraînement.")
    print("\nOuvrez robot_esp32/robot_esp32.ino dans l'IDE Arduino et téléversez.")


if __name__ == "__main__":
    main()
