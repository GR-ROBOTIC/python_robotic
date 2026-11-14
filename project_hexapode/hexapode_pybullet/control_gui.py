"""Piloter l'araignée avec des boutons.

Deux interfaces :
  python control_gui.py                 # fenêtre Tkinter avec boutons + clavier (par défaut)
  python control_gui.py --ui pybullet   # boutons dans le panneau de la fenêtre PyBullet
                                        # (à utiliser sur macOS si Tkinter pose problème)
Options :
  --run NOM        utiliser un entraînement précis (défaut : le dernier)
  --no-policy      démarche tripode seule, sans IA
Clavier (fenêtre Tkinter) : flèches = avancer/reculer/gauche/droite,
  A / E = tourner gauche/droite, C = cercle, Espace = stop, R = remettre debout.
"""
import argparse
import time

import numpy as np
import pybullet as pb

from hexapod import config as C
from hexapod.env import HexapodEnv
from hexapod.utils import Pilot, resolve_run

LABELS = {
    "avancer": "▲  Avancer", "reculer": "▼  Reculer",
    "gauche": "◀  Gauche", "droite": "Droite  ▶",
    "tourner_gauche": "↺  Tourner G", "tourner_droite": "Tourner D  ↻",
    "cercle": "◯  Cercle", "stop": "■  Stop",
}


class Robot:
    """Simulation + pilote, avancée d'un pas de contrôle à chaque tick()."""

    def __init__(self, run_dir):
        self.pilot = Pilot(run_dir)
        self.env = HexapodEnv(render_mode="human", max_steps=10**9, randomize=False)
        self.obs, _ = self.env.reset()
        self.name = "stop"
        self.speed = 1.0
        self.follow = True
        self.info = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.set("stop")

    def set(self, name):
        self.name = name
        self.env.set_command(np.array(C.COMMANDS[name]) * self.speed)

    def set_speed(self, s):
        self.speed = float(s)
        self.set(self.name)

    def reset(self):
        self.obs, _ = self.env.reset()

    def tick(self):
        self.obs, _, term, _, self.info = self.env.step(self.pilot.act(self.obs))
        if self.follow:
            self.env.follow_camera()
        self.env.show_text(LABELS[self.name])
        if term:
            self.reset()
            return "Chute → remis debout"
        return None


# ---------------------------------------------------------------- Tkinter
def run_tk(robot):
    import tkinter as tk

    root = tk.Tk()
    root.title("Hexapode – télécommande")
    root.resizable(False, False)
    mode = "IA : " + robot.pilot.run_dir.name if robot.pilot.active else "Démarche tripode (sans IA)"
    tk.Label(root, text=mode, font=("Helvetica", 10, "italic")).grid(row=0, column=0, columnspan=3, pady=(8, 4))

    buttons = {}

    def press(name):
        robot.set(name)
        for n, b in buttons.items():
            b.configure(relief=tk.SUNKEN if n == name else tk.RAISED)

    layout = [("tourner_gauche", 1, 0), ("avancer", 1, 1), ("tourner_droite", 1, 2),
              ("gauche", 2, 0), ("stop", 2, 1), ("droite", 2, 2),
              ("cercle", 3, 0), ("reculer", 3, 1)]
    for name, r, c in layout:
        b = tk.Button(root, text=LABELS[name], width=13, height=2, command=lambda n=name: press(n))
        b.grid(row=r, column=c, padx=4, pady=4)
        buttons[name] = b
    tk.Button(root, text="⟲  Remettre debout", width=13, height=2,
              command=robot.reset).grid(row=3, column=2, padx=4, pady=4)

    tk.Label(root, text="Vitesse").grid(row=4, column=0, sticky="e")
    speed = tk.Scale(root, from_=0.2, to=1.0, resolution=0.05, orient=tk.HORIZONTAL, length=220,
                     command=robot.set_speed)
    speed.set(1.0)
    speed.grid(row=4, column=1, columnspan=2, sticky="w")

    follow = tk.BooleanVar(value=True)
    tk.Checkbutton(root, text="Caméra suit le robot", variable=follow,
                   command=lambda: setattr(robot, "follow", follow.get())).grid(row=5, column=0, columnspan=3)
    status = tk.Label(root, text="", font=("Courier", 10))
    status.grid(row=6, column=0, columnspan=3, pady=(4, 8))

    keys = {"Up": "avancer", "Down": "reculer", "Left": "gauche", "Right": "droite",
            "a": "tourner_gauche", "e": "tourner_droite", "c": "cercle", "space": "stop"}
    for k, n in keys.items():
        root.bind(f"<{k}>" if len(k) > 1 else k, lambda _e, n=n: press(n))
    root.bind("r", lambda _e: robot.reset())
    press("stop")

    def tick():
        t0 = time.time()
        try:
            msg = robot.tick()
        except pb.error:
            root.destroy()
            return
        i = robot.info
        status.configure(text=msg or f"vx {i['vx']:+.2f} m/s   vy {i['vy']:+.2f} m/s   rot {i['wz']:+.2f} rad/s")
        delay = max(1, int(1000 * (robot.env.control_dt - (time.time() - t0))))
        root.after(delay, tick)

    root.after(100, tick)
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()


# ---------------------------------------------------------------- Boutons PyBullet
def run_pybullet(robot):
    p = robot.env.p
    order = ["avancer", "reculer", "gauche", "droite", "tourner_gauche", "tourner_droite", "cercle", "stop"]
    btns = {n: p.addUserDebugParameter(LABELS[n], 1, 0, 0) for n in order}
    btn_reset = p.addUserDebugParameter("Remettre debout", 1, 0, 0)
    slider = p.addUserDebugParameter("Vitesse", 0.2, 1.0, 1.0)
    last = {n: p.readUserDebugParameter(b) for n, b in btns.items()}
    last_reset = p.readUserDebugParameter(btn_reset)
    print("Utilise les boutons dans le panneau à gauche de la fenêtre PyBullet.")
    try:
        while True:
            t0 = time.time()
            for n, b in btns.items():
                v = p.readUserDebugParameter(b)
                if v != last[n]:
                    last[n] = v
                    robot.set(n)
            v = p.readUserDebugParameter(btn_reset)
            if v != last_reset:
                last_reset = v
                robot.reset()
            s = p.readUserDebugParameter(slider)
            if abs(s - robot.speed) > 1e-3:
                robot.set_speed(s)
            robot.tick()
            time.sleep(max(0.0, robot.env.control_dt - (time.time() - t0)))
    except (KeyboardInterrupt, pb.error):
        pass


def main():
    ap = argparse.ArgumentParser(description="Télécommande de l'hexapode")
    ap.add_argument("--run", default="latest")
    ap.add_argument("--no-policy", action="store_true")
    ap.add_argument("--ui", choices=["tk", "pybullet"], default="tk")
    args = ap.parse_args()

    run_dir = None if args.no_policy else resolve_run(args.run)
    if run_dir is None and not args.no_policy:
        print("Aucun modèle trouvé → démarche tripode seule.")
    robot = Robot(run_dir)

    ui = args.ui
    if ui == "tk":
        try:
            import tkinter  # noqa: F401
        except ImportError:
            print("Tkinter indisponible → boutons dans la fenêtre PyBullet.")
            ui = "pybullet"
    try:
        run_tk(robot) if ui == "tk" else run_pybullet(robot)
    finally:
        robot.env.close()


if __name__ == "__main__":
    main()
