"""
Mesure la vitesse de simulation de ce PC, comme pendant un vrai entraînement.
À lancer sur chaque PC pour les comparer (environ 1 à 2 minutes) :

    python comparer_pc.py

Le chiffre important : le meilleur "pas/s" en parallèle.
Le PC qui a le plus grand chiffre entraînera le plus vite.
"""
import os
import platform
import time

import numpy as np
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv

from env_bipede import BipedeEnv

PAS_PAR_TEST = 1500


def mesurer(n_envs):
    env = make_vec_env(BipedeEnv, n_envs=n_envs, vec_env_cls=SubprocVecEnv)
    env.reset()
    rng = np.random.default_rng(0)
    for _ in range(50):                                     # échauffement
        env.step(rng.uniform(-0.3, 0.3, (n_envs, 12)))
    t0 = time.perf_counter()
    for _ in range(PAS_PAR_TEST // n_envs * 4):
        env.step(rng.uniform(-0.3, 0.3, (n_envs, 12)))
    duree = time.perf_counter() - t0
    env.close()
    return (PAS_PAR_TEST // n_envs * 4) * n_envs / duree


def nom_processeur():
    try:
        with open("/proc/cpuinfo") as f:
            for ligne in f:
                if ligne.startswith("model name"):
                    return ligne.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or platform.machine()


def main():
    coeurs = os.cpu_count()
    print(f"Système    : {platform.system()} {platform.release()}")
    print(f"Processeur : {nom_processeur()}  ({coeurs} threads)")
    print()

    # Vitesse d'un seul robot (= vitesse d'un cœur)
    env = BipedeEnv()
    env.reset(seed=0)
    t0 = time.perf_counter()
    for _ in range(1500):
        _, _, fin, tronque, _ = env.step(env.action_space.sample() * 0.3)
        if fin or tronque:
            env.reset()
    un = 1500 / (time.perf_counter() - t0)
    env.close()
    print(f"1 simulation      : {un:7.0f} pas/s  (vitesse d'un cœur)")

    resultats = {}
    for n in sorted({4, 8, 12, 16, coeurs} - {0}):
        if n > coeurs:
            continue
        resultats[n] = mesurer(n)
        print(f"{n:2d} simulations    : {resultats[n]:7.0f} pas/s")

    meilleur = max(resultats, key=resultats.get)
    v = resultats[meilleur]
    print()
    print(f"==> Meilleur réglage : --envs {meilleur}  ({v:.0f} pas/s)")
    print(f"    15 millions de pas ≈ {15e6 / v / 60:.0f} min de simulation"
          " (+ 20 à 40 % pour l'apprentissage du réseau)")


if __name__ == "__main__":
    main()
