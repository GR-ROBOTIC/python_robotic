"""
Exporte le "cerveau" entraîné (réseau PPO) vers un fichier C pour l'ESP32.

    python exporter_esp32.py
    python exporter_esp32.py --modele modeles/bipede_3000000_steps.zip \
                             --stats  modeles/bipede_vecnormalize_3000000_steps.pkl

Résultat : bipede_esp32/politique.h  (à côté de bipede_esp32.ino)
Le script vérifie aussi que le calcul "à la main" donne exactement
le même résultat que PyTorch, et fait marcher le robot en simulation avec.
"""
import argparse
import os
import pickle

import numpy as np

DOSSIER = os.path.dirname(os.path.abspath(__file__))


# ----------------------------------------------------------------------
# Calcul du réseau en numpy pur : c'est exactement ce que fera l'ESP32
# ----------------------------------------------------------------------
def politique_numpy(obs, moyenne, ecart, clip, couches):
    x = np.clip((obs - moyenne) / ecart, -clip, clip)
    for k, (W, b) in enumerate(couches):
        x = W @ x + b
        if k < len(couches) - 1:
            x = np.tanh(x)
    return np.clip(x, -1.0, 1.0)


def tableau_c(nom, valeurs, par_ligne=8):
    v = np.asarray(valeurs, dtype=np.float32).ravel()
    lignes = []
    for i in range(0, len(v), par_ligne):
        lignes.append("  " + ", ".join(f"{x:.8e}f" for x in v[i:i + par_ligne]))
    return f"static const float {nom}[{len(v)}] = {{\n" + ",\n".join(lignes) + "\n};\n"


def ecrire_entete(chemin, couches, moyenne, ecart, clip, noms, repos, bas, haut,
                  test_obs, test_action, echelle, periode, dt, source):
    (W1, B1), (W2, B2), (W3, B3) = couches
    n_in, n_c1, n_c2, n_out = W1.shape[1], W1.shape[0], W2.shape[0], W3.shape[0]
    t = []
    t.append("// ============================================================\n")
    t.append("//  FICHIER GÉNÉRÉ AUTOMATIQUEMENT par exporter_esp32.py\n")
    t.append(f"//  Modèle : {source}\n")
    t.append("//  Ne pas modifier à la main : relancer l'export à la place.\n")
    t.append("// ============================================================\n")
    t.append("#pragma once\n\n")
    t.append(f"#define N_ENTREE {n_in}\n#define N_C1 {n_c1}\n#define N_C2 {n_c2}\n#define N_SORTIE {n_out}\n\n")
    t.append(f"static const float OBS_CLIP = {clip:.6f}f;\n")
    t.append(f"static const float ECHELLE_ACTION = {echelle:.6f}f;   // rad\n")
    t.append(f"static const float PERIODE_PAS = {periode:.6f}f;      // s\n")
    t.append(f"static const float DT_CONTROLE = {dt:.6f}f;      // s (= {1 / dt:.0f} Hz)\n\n")
    t.append("// Ordre des servos (index 0 à 11) :\n")
    for i, n in enumerate(noms):
        t.append(f"//   {i:2d} : {n}\n")
    t.append("\n")
    t.append(tableau_c("POSE_REPOS", repos))
    t.append(tableau_c("LIM_BAS", bas))
    t.append(tableau_c("LIM_HAUT", haut))
    t.append("\n// Normalisation des observations\n")
    t.append(tableau_c("OBS_MOYENNE", moyenne))
    t.append(tableau_c("OBS_ECART", ecart))
    t.append("\n// Poids du réseau (matrices rangées ligne par ligne : [sortie][entrée])\n")
    for nom, arr in (("W1", W1), ("B1", B1), ("W2", W2), ("B2", B2), ("W3", W3), ("B3", B3)):
        t.append(tableau_c(nom, arr))
    t.append("\n// Vecteur de test : l'ESP32 vérifie au démarrage qu'il calcule la même chose que le PC\n")
    t.append(tableau_c("TEST_OBS", test_obs))
    t.append(tableau_c("TEST_ACTION", test_action))
    os.makedirs(os.path.dirname(chemin), exist_ok=True)
    with open(chemin, "w", encoding="utf-8") as f:
        f.write("".join(t))


# ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--modele", default=os.path.join(DOSSIER, "modeles", "bipede_final.zip"))
    parser.add_argument("--stats", default=os.path.join(DOSSIER, "modeles", "vecnormalize_final.pkl"))
    parser.add_argument("--sortie", default=os.path.join(DOSSIER, "bipede_esp32", "politique.h"))
    args = parser.parse_args()

    import torch.nn as nn
    from stable_baselines3 import PPO
    import env_bipede
    from env_bipede import BipedeEnv

    print(f"Chargement de {args.modele}")
    model = PPO.load(args.modele, device="cpu")
    with open(args.stats, "rb") as f:
        stats = pickle.load(f)

    moyenne = stats.obs_rms.mean.astype(np.float64)
    ecart = np.sqrt(stats.obs_rms.var + stats.epsilon)
    clip = float(stats.clip_obs)

    # Extraction des couches de l'acteur (celui qui choisit les actions)
    pi = model.policy.mlp_extractor.policy_net
    lineaires = [m for m in pi if isinstance(m, nn.Linear)] + [model.policy.action_net]
    activations = [m for m in pi if not isinstance(m, nn.Linear)]
    if len(lineaires) != 3 or not all(isinstance(a, nn.Tanh) for a in activations):
        raise SystemExit("Ce script attend un réseau à 2 couches cachées avec tanh (réglage de entrainer.py).")
    couches = [(l.weight.detach().numpy().astype(np.float64),
                l.bias.detach().numpy().astype(np.float64)) for l in lineaires]
    print("Architecture : " + " -> ".join(str(c[0].shape[1]) for c in couches) + f" -> {couches[-1][0].shape[0]}")

    # Vérification en simulation : le calcul numpy pilote le robot
    env = BipedeEnv(randomiser=False)
    obs, _ = env.reset(seed=0)
    ecart_max, test_obs, test_action = 0.0, None, None
    for t in range(500):
        a_np = politique_numpy(obs, moyenne, ecart, clip, couches)
        obs_norm = np.clip((obs - moyenne) / ecart, -clip, clip).astype(np.float32)
        a_sb3, _ = model.predict(obs_norm, deterministic=True)
        ecart_max = max(ecart_max, float(np.max(np.abs(a_np - a_sb3))))
        if t == 100:
            test_obs, test_action = obs.copy(), a_np.copy()
        obs, _, fin, tronque, info = env.step(a_np)
        if fin or tronque:
            break
    if test_obs is None:
        test_obs, test_action = obs.copy(), politique_numpy(obs, moyenne, ecart, clip, couches)
    print(f"Écart max calcul manuel / PyTorch : {ecart_max:.2e}  (doit être < 1e-4)")
    print(f"Test en simulation : {info['distance']:.2f} m en {(t + 1) * env.dt:.1f} s"
          + ("  -> CHUTE" if fin else ""))

    noms = [env.bc.getJointInfo(env.robot, j)[1].decode() for j in env.joints]
    ecrire_entete(args.sortie, couches, moyenne, ecart, clip, noms,
                  env.pose_repos, env.bas, env.haut, test_obs, test_action,
                  env_bipede.ECHELLE_ACTION, env_bipede.PERIODE_PAS, env.dt,
                  os.path.basename(args.modele))
    env.close()
    taille = sum(W.size + b.size for W, b in couches) * 4 / 1024
    print(f"\nFichier généré : {args.sortie}  ({taille:.1f} Ko de poids)")
    print("Ouvrez bipede_esp32/bipede_esp32.ino dans l'IDE Arduino et téléversez.")


if __name__ == "__main__":
    main()
