"""Gestion des dossiers d'entraînement et chargement du « pilote » (réseau entraîné)."""
import json
import pickle
import time
from pathlib import Path

import numpy as np

from . import config as C

LATEST_FILE = C.RUNS_DIR / "LATEST"


def new_run_dir(name=None):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    d = C.RUNS_DIR / (f"{stamp}_{name}" if name else stamp)
    d.mkdir(parents=True, exist_ok=True)
    return d


def set_latest(run_dir):
    C.RUNS_DIR.mkdir(parents=True, exist_ok=True)
    LATEST_FILE.write_text(Path(run_dir).name, encoding="utf-8")


def list_runs():
    if not C.RUNS_DIR.exists():
        return []
    runs = [d for d in C.RUNS_DIR.iterdir() if d.is_dir() and (d / "model.zip").exists()]
    return sorted(runs, key=lambda d: (d / "model.zip").stat().st_mtime)


def latest_run():
    if LATEST_FILE.exists():
        d = C.RUNS_DIR / LATEST_FILE.read_text(encoding="utf-8").strip()
        if (d / "model.zip").exists():
            return d
    runs = list_runs()
    return runs[-1] if runs else None


def resolve_run(arg):
    """'latest' / None → dernier entraînement ; sinon un nom de dossier ou un chemin."""
    if arg in (None, "", "latest"):
        return latest_run()
    d = Path(arg)
    if not d.exists():
        d = C.RUNS_DIR / arg
    return d if (d / "model.zip").exists() else None


def run_summary(d):
    info = {}
    cfg = d / "config.json"
    if cfg.exists():
        info = json.loads(cfg.read_text(encoding="utf-8"))
    return info


class Pilot:
    """Politique entraînée. Sans modèle, renvoie zéro = démarche tripode pure."""

    def __init__(self, run_dir=None, n_actions=len(C.JOINT_NAMES)):
        self.model = None
        self.vecnorm = None
        self.n_actions = n_actions
        self.run_dir = run_dir
        if run_dir is not None:
            from stable_baselines3 import PPO
            self.model = PPO.load(str(Path(run_dir) / "model.zip"), device="cpu")
            vn = Path(run_dir) / "vecnormalize.pkl"
            if vn.exists():
                with open(vn, "rb") as f:
                    self.vecnorm = pickle.load(f)
                self.vecnorm.training = False

    @property
    def active(self):
        return self.model is not None

    def act(self, obs):
        if self.model is None:
            return np.zeros(self.n_actions, np.float32)
        o = self.vecnorm.normalize_obs(obs) if self.vecnorm is not None else obs
        action, _ = self.model.predict(o, deterministic=True)
        return action
