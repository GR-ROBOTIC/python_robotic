"""Lancer une chorégraphie dans la fenêtre PyBullet.

  python play.py --list                 # lister les danses et les pas
  python play.py disco                  # une danse (nom ou morceau du nom)
  python play.py all                    # le spectacle complet, toutes les danses
  python play.py disco --bpm 128        # changer le tempo
  python play.py disco --loop           # en boucle
  python play.py disco --video          # enregistrer une vidéo MP4 dans videos/ (ffmpeg requis)
  python play.py --move coucou          # un seul pas
"""
import argparse
import shutil
import time

import pybullet as pb

from hexapod import config as C
from hexapod.choreo import Choreographer, find_dance, list_dances, load_dance
from hexapod.moves import MOVES
from hexapod.sim import Sim


def print_list():
    print("Chorégraphies :")
    for p in list_dances():
        d = load_dance(p)
        beats = sum(s[1] for s in d["steps"])
        print(f"  {p.stem:22s} {d['name']:20s} {d['bpm']} BPM, {len(d['steps'])} pas, ~{beats * 60 / d['bpm']:.0f} s")
    print("\nPas disponibles :")
    for name, m in MOVES.items():
        print(f"  {name:20s} {m['label']:28s} {m['beats']} beats{'  (lève des pattes)' if m['legs'] else ''}")


def main():
    ap = argparse.ArgumentParser(description="Faire danser l'araignée")
    ap.add_argument("dance", nargs="?", default="all")
    ap.add_argument("--move", help="jouer un seul pas")
    ap.add_argument("--bpm", type=float, help="forcer le tempo")
    ap.add_argument("--intensity", type=float, default=1.0, help="amplitude des mouvements (0.3 à 1.5)")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--no-disco", action="store_true", help="sans piste de danse colorée")
    ap.add_argument("--video", action="store_true", help="enregistrer en MP4 dans videos/")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        print_list()
        return

    if args.move:
        if args.move not in MOVES:
            print(f"Pas inconnu. Disponibles : {', '.join(MOVES)}")
            return
        playlist = [{"name": MOVES[args.move]["label"], "bpm": C.DEFAULT_BPM,
                     "steps": [(args.move, MOVES[args.move]["beats"], 1.0)]}]
    elif args.dance == "all":
        playlist = [load_dance(p) for p in list_dances()]
    else:
        path = find_dance(args.dance)
        if path is None:
            print(f"Danse « {args.dance} » introuvable. Lance : python play.py --list")
            return
        playlist = [load_dance(path)]

    sim = Sim(gui=True, disco=not args.no_disco)
    chor = Choreographer()
    chor.intensity = args.intensity
    sim.reset(chor.kin.neutral())

    log = None
    if args.video:
        if shutil.which("ffmpeg") is None:
            print("ffmpeg introuvable : installe-le (sudo apt install ffmpeg) pour enregistrer.")
        else:
            C.VIDEOS_DIR.mkdir(exist_ok=True)
            out = C.VIDEOS_DIR / f"{time.strftime('%Y%m%d-%H%M%S')}_{playlist[0]['name'].replace(' ', '_')}.mp4"
            log = sim.start_video(out)
            print(f"Enregistrement : {out}")

    idx = 0
    try:
        while True:
            if not chor.busy:
                if idx >= len(playlist):
                    if not args.loop:
                        time.sleep(1.0)
                        break
                    idx = 0
                dance = playlist[idx]
                idx += 1
                chor.play_dance(dance)
                if args.bpm:
                    chor.bpm = args.bpm
                print(f"♪ {dance['name']}  ({chor.bpm:.0f} BPM)")

            t0 = time.time()
            q, new_beat = chor.update(sim.dt)
            sim.step(q)
            if new_beat and sim.disco:
                sim.disco.flash()
            beat = int(chor.total_beats) % 4 + 1
            sim.show_text(f"{chor.dance_name or ''}   {chor.label}   {'●' * beat}{'○' * (4 - beat)}")
            sim.follow_camera()
            if sim.fallen():
                print("Chute ! On se relève.")
                sim.reset(chor.kin.neutral())
            time.sleep(max(0.0, sim.dt - (time.time() - t0)))
    except (KeyboardInterrupt, pb.error):
        pass
    finally:
        if log is not None:
            try:
                sim.stop_video(log)
            except pb.error:
                pass
        sim.close()


if __name__ == "__main__":
    main()
