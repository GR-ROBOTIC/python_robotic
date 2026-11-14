"""Vérifie que toutes les danses (et chaque pas) tiennent debout, sans fenêtre.

  python check_dances.py            # toutes les chorégraphies
  python check_dances.py --moves    # chaque pas de la bibliothèque, un par un
  python check_dances.py ma_danse   # une seule chorégraphie

Pour chaque danse : inclinaison max du corps, distance parcourue, chute ou non.
Utile avant de montrer une nouvelle danse… ou de la lancer sur le vrai robot.
"""
import argparse
import math
import sys

from hexapod.choreo import Choreographer, find_dance, list_dances, load_dance
from hexapod.moves import MOVES
from hexapod.sim import Sim


def run(sim, chor, settle_beats=2):
    sim.reset(chor.kin.neutral())
    max_tilt, fell, t = 0.0, False, 0.0
    start = sim.pose()[0]
    while chor.busy or t < settle_beats * 60 / chor.bpm:
        if not chor.busy:
            t += sim.dt
        q, _ = chor.update(sim.dt)
        sim.step(q)
        _, (roll, pitch, _) = sim.pose()
        max_tilt = max(max_tilt, abs(roll), abs(pitch))
        if sim.fallen():
            fell = True
            break
    end = sim.pose()[0]
    return math.degrees(max_tilt), math.hypot(end[0] - start[0], end[1] - start[1]), fell


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dance", nargs="?")
    ap.add_argument("--moves", action="store_true", help="tester chaque pas séparément")
    args = ap.parse_args()

    sim = Sim(gui=False)
    ok = True
    if args.moves:
        items = [(name, None) for name in MOVES]
    else:
        paths = [find_dance(args.dance)] if args.dance else list_dances()
        if None in paths:
            print(f"Danse « {args.dance} » introuvable.")
            sys.exit(1)
        items = [(None, p) for p in paths]

    print(f"{'':28s} {'incl. max':>9s} {'déplac.':>8s}  résultat")
    for name, path in items:
        chor = Choreographer()
        if path is not None:
            d = load_dance(path)
            chor.play_dance(d)
            label = d["name"]
        else:
            chor.enqueue(name)
            label = MOVES[name]["label"]
        tilt, dist, fell = run(sim, chor)
        verdict = "CHUTE ✗" if fell else ("instable ?" if tilt > 25 else "ok ✓")
        ok &= not fell
        print(f"{label:28s} {tilt:8.1f}° {dist:7.2f} m  {verdict}")
    sim.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
