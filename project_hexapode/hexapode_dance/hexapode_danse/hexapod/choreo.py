"""Le chorégraphe : enchaîne les pas au tempo et calcule les 18 angles à chaque instant."""
import json
import math
from collections import deque

import numpy as np

from . import config as C
from .kinematics import Kinematics
from .moves import MOVES, Pose


def load_dance(path):
    """Lit une chorégraphie JSON. Chaque pas : ["nom", beats] ou ["nom", beats, {"amp": 1.2}]."""
    data = json.loads(open(path, encoding="utf-8").read())
    steps = []
    for s in data["steps"]:
        if isinstance(s, dict):
            name, beats, params = s["move"], s.get("beats"), s
        else:
            name, beats, params = s[0], (s[1] if len(s) > 1 else None), (s[2] if len(s) > 2 else {})
        if name not in MOVES:
            raise ValueError(f"{path.name} : pas inconnu « {name} ». Pas disponibles : {', '.join(MOVES)}")
        steps.append((name, beats or MOVES[name]["beats"], float(params.get("amp", 1.0))))
    return {"name": data.get("name", path.stem), "bpm": data.get("bpm", C.DEFAULT_BPM),
            "steps": steps, "path": path}


def list_dances():
    return sorted(d for d in C.DANCES_DIR.glob("*.json") if not d.name.startswith("_"))


def find_dance(name):
    for p in list_dances():
        if p.stem == name or name in p.stem:
            return p
    return None


class Choreographer:
    def __init__(self, bpm=C.DEFAULT_BPM):
        self.kin = Kinematics()
        self.bpm = bpm
        self.intensity = 1.0
        self.queue = deque()
        self.current = None           # (nom, beats, amp)
        self.t_move = 0.0
        self.total_beats = 0.0
        self.phase = 0.0
        self.walk_target = np.zeros(3)
        self.walk = np.zeros(3)
        self.dance_name = None

    # ------------------------------------------------------------ commandes
    def play_dance(self, dance):
        self.clear()
        self.bpm = dance["bpm"]
        self.dance_name = dance["name"]
        self.queue.extend(dance["steps"])

    def enqueue(self, name, beats=None, amp=1.0):
        self.queue.append((name, beats or MOVES[name]["beats"], amp))

    def clear(self):
        self.queue.clear()
        self.current = None
        self.dance_name = None

    def set_walk(self, cmd):
        self.walk_target = np.clip(np.asarray(cmd, float), -1, 1)

    @property
    def busy(self):
        return self.current is not None or bool(self.queue)

    @property
    def label(self):
        return MOVES[self.current[0]]["label"] if self.current else ""

    # ------------------------------------------------------------ calcul
    def update(self, dt):
        """Avance de dt secondes. Renvoie (angles des 18 servos, True si un nouveau beat commence)."""
        d_beats = dt * self.bpm / 60.0
        old_beat = math.floor(self.total_beats)
        self.total_beats += d_beats
        self.phase = (self.phase + d_beats / 2.0) % 1.0      # un cycle de marche = 2 beats

        if self.current is None and self.queue:
            self.current = self.queue.popleft()
            self.t_move = 0.0
            if MOVES[self.current[0]]["legs"]:
                self.walk_target[:] = 0.0                     # pas de marche pendant un jeu de pattes

        P = Pose()
        if self.current is not None:
            name, beats, amp = self.current
            MOVES[name]["fn"](self.t_move, beats, amp * self.intensity, P)
            self.t_move += d_beats
            if self.t_move >= beats:
                self.current = None
                if not self.queue:
                    self.dance_name = None

        legs_busy = self.current is not None and MOVES[self.current[0]]["legs"]
        target = np.zeros(3) if legs_busy else self.walk_target
        self.walk += (target - self.walk) * min(1.0, 4.0 * dt)

        cmd = np.clip(P.cmd + self.walk, -1, 1) * np.array(C.CMD_SCALE)
        freq = self.bpm / 120.0
        feet = self.kin.nominal + self.kin.gait_offsets(self.phase, cmd, freq) + P.feet
        q = self.kin.joint_targets(feet, P.body)
        return q, math.floor(self.total_beats) != old_beat
