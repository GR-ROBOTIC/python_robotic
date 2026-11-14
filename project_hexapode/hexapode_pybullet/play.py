"""Voir un entraînement (ou la démarche seule) dans la fenêtre PyBullet.

Exemples :
  python play.py                         # dernier entraînement, démo de tous les mouvements
  python play.py --cmd avancer           # une seule commande
  python play.py --run 20260101-120000   # un entraînement précis
  python play.py --no-policy             # démarche tripode seule, sans IA
  python play.py --list                  # lister les entraînements
"""
import argparse
import time

import numpy as np
import pybullet as pb

from hexapod import config as C
from hexapod.env import HexapodEnv
from hexapod.utils import Pilot, list_runs, resolve_run, run_summary

DEMO = ["avancer", "tourner_gauche", "gauche", "reculer", "droite", "tourner_droite", "cercle", "stop"]


def main():
    ap = argparse.ArgumentParser(description="Visualiser l'hexapode")
    ap.add_argument("--run", default="latest")
    ap.add_argument("--cmd", default="demo", choices=["demo"] + list(C.COMMANDS))
    ap.add_argument("--seconds", type=float, default=5.0, help="durée de chaque mouvement en démo")
    ap.add_argument("--no-policy", action="store_true", help="démarche tripode pure, sans réseau")
    ap.add_argument("--list", action="store_true", help="lister les entraînements et quitter")
    args = ap.parse_args()

    if args.list:
        runs = list_runs()
        if not runs:
            print("Aucun entraînement pour l'instant.")
        for d in runs:
            s = run_summary(d)
            print(f"  {d.name:35s} pas={s.get('total_timesteps', '?'):>9}  durée={s.get('duration_min', '?')} min")
        return

    run_dir = None if args.no_policy else resolve_run(args.run)
    if run_dir is None and not args.no_policy:
        print("Aucun modèle trouvé → démarche tripode seule (lance un entraînement pour l'IA).")
    pilot = Pilot(run_dir)
    label = f"IA : {run_dir.name}" if pilot.active else "Démarche tripode (sans IA)"
    print(label)

    env = HexapodEnv(render_mode="human", max_steps=10**9, randomize=False)
    obs, _ = env.reset()
    seq = DEMO if args.cmd == "demo" else [args.cmd]
    idx, t_cmd = 0, time.time()
    env.set_command(C.COMMANDS[seq[0]])
    obs = env._obs()

    try:
        while True:
            t0 = time.time()
            if len(seq) > 1 and t0 - t_cmd > args.seconds:
                idx = (idx + 1) % len(seq)
                t_cmd = t0
                env.set_command(C.COMMANDS[seq[idx]])
            obs, _, term, _, info = env.step(pilot.act(obs))
            env.follow_camera()
            env.show_text(f"{seq[idx]}  |  vx={info['vx']:+.2f}  vy={info['vy']:+.2f}  rot={info['wz']:+.2f}")
            if term:
                print("Chute ! Redémarrage.")
                obs, _ = env.reset()
            time.sleep(max(0.0, env.control_dt - (time.time() - t0)))
    except (KeyboardInterrupt, pb.error):
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
