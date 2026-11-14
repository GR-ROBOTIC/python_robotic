"""Télécommande de danse : chorégraphies, pas à la carte et déplacements, en direct.

  python dance_gui.py

- Colonne « Chorégraphies » : lance une danse complète (remplace ce qui est en cours).
- Grille « Pas de danse » : chaque clic ajoute le pas à la file d'attente → tu improvises.
- Flèches « Déplacement » : marcher pendant les pas qui gardent les 6 pieds au sol
  (ex. twist en avançant, rebond en tournant…).
- Tempo, intensité, piste disco et caméra réglables.
Clavier : flèches = marcher, A/E = tourner, Espace = stop, Échap = vider la file.
"""
import time
import tkinter as tk

import pybullet as pb

from hexapod.choreo import Choreographer, list_dances, load_dance
from hexapod.moves import MOVES
from hexapod.sim import Sim

WALK = {
    "▲": (1, 0, 0), "▼": (-1, 0, 0), "◀": (0, 1, 0), "▶": (0, -1, 0),
    "↺": (0, 0, 1), "↻": (0, 0, -1), "■": (0, 0, 0),
}


def main():
    sim = Sim(gui=True, disco=True)
    chor = Choreographer()
    sim.reset(chor.kin.neutral())
    state = {"follow": True, "disco": True}

    root = tk.Tk()
    root.title("🕷 Araignée danseuse")
    root.resizable(False, False)

    # --------------------------------------------------- chorégraphies
    left = tk.LabelFrame(root, text="Chorégraphies", padx=6, pady=6)
    left.grid(row=0, column=0, sticky="n", padx=6, pady=6)
    dances = [load_dance(p) for p in list_dances()]

    def play(d):
        chor.play_dance(d)
        bpm.set(d["bpm"])

    for d in dances:
        tk.Button(left, text=f"♪ {d['name']}", width=20, command=lambda d=d: play(d)).pack(pady=2)

    def show_all():
        chor.clear()
        for d in dances:
            chor.queue.extend(d["steps"])
        chor.dance_name = "Spectacle"

    tk.Button(left, text="★ Spectacle complet", width=20, command=show_all).pack(pady=(8, 2))
    tk.Button(left, text="✕ Vider la file", width=20, command=chor.clear).pack(pady=2)

    # --------------------------------------------------- pas à la carte
    mid = tk.LabelFrame(root, text="Pas de danse (ajoutés à la file)", padx=6, pady=6)
    mid.grid(row=0, column=1, sticky="n", padx=6, pady=6)
    for k, (name, m) in enumerate(MOVES.items()):
        tk.Button(mid, text=m["label"], width=18,
                  command=lambda n=name: chor.enqueue(n)).grid(row=k // 3, column=k % 3, padx=2, pady=2)

    # --------------------------------------------------- déplacement
    right = tk.LabelFrame(root, text="Déplacement", padx=6, pady=6)
    right.grid(row=0, column=2, sticky="n", padx=6, pady=6)
    layout = [("↺", 0, 0), ("▲", 0, 1), ("↻", 0, 2), ("◀", 1, 0), ("■", 1, 1), ("▶", 1, 2), ("▼", 2, 1)]
    for sym, r, c in layout:
        tk.Button(right, text=sym, width=4, height=2, font=("Helvetica", 14),
                  command=lambda s=sym: chor.set_walk(WALK[s])).grid(row=r, column=c, padx=2, pady=2)

    # --------------------------------------------------- réglages
    bottom = tk.Frame(root)
    bottom.grid(row=1, column=0, columnspan=3, pady=(0, 6))
    tk.Label(bottom, text="Tempo (BPM)").grid(row=0, column=0, sticky="e")
    bpm = tk.Scale(bottom, from_=60, to=160, orient=tk.HORIZONTAL, length=220,
                   command=lambda v: setattr(chor, "bpm", float(v)))
    bpm.set(chor.bpm)
    bpm.grid(row=0, column=1, padx=4)
    tk.Label(bottom, text="Intensité").grid(row=0, column=2, sticky="e")
    inten = tk.Scale(bottom, from_=0.3, to=1.5, resolution=0.05, orient=tk.HORIZONTAL, length=180,
                     command=lambda v: setattr(chor, "intensity", float(v)))
    inten.set(1.0)
    inten.grid(row=0, column=3, padx=4)

    disco = tk.BooleanVar(value=True)
    follow = tk.BooleanVar(value=True)
    tk.Checkbutton(bottom, text="Piste disco", variable=disco,
                   command=lambda: state.update(disco=disco.get())).grid(row=1, column=0, columnspan=2)
    tk.Checkbutton(bottom, text="Caméra suit l'araignée", variable=follow,
                   command=lambda: state.update(follow=follow.get())).grid(row=1, column=2, columnspan=2)
    status = tk.Label(root, text="", font=("Courier", 11))
    status.grid(row=2, column=0, columnspan=3, pady=(0, 8))

    keys = {"Up": "▲", "Down": "▼", "Left": "◀", "Right": "▶", "a": "↺", "e": "↻", "space": "■"}
    for k, s in keys.items():
        root.bind(f"<{k}>" if len(k) > 1 else k, lambda _e, s=s: chor.set_walk(WALK[s]))
    root.bind("<Escape>", lambda _e: chor.clear())

    def tick():
        t0 = time.time()
        try:
            q, new_beat = chor.update(sim.dt)
            sim.step(q)
            if new_beat and sim.disco and state["disco"]:
                sim.disco.flash()
            if state["follow"]:
                sim.follow_camera()
            sim.show_text(chor.label)
            if sim.fallen():
                sim.reset(chor.kin.neutral())
        except pb.error:
            root.destroy()
            return
        beat = int(chor.total_beats) % 4 + 1
        status.configure(text=f"{chor.label or 'en attente'}   |   file : {len(chor.queue)} pas   |   "
                              f"{'●' * beat}{'○' * (4 - beat)}")
        root.after(max(1, int(1000 * (sim.dt - (time.time() - t0)))), tick)

    root.after(100, tick)
    root.mainloop()
    sim.close()


if __name__ == "__main__":
    main()
