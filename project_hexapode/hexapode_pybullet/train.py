"""Entraînement PPO de l'hexapode.

Exemples :
  python train.py --steps 300000              # entraînement rapide
  python train.py --steps 3000000 --envs 12   # long, 12 simulations en parallèle
  python train.py --resume latest --steps 1000000
  python train.py --gui --steps 20000         # regarder l'entraînement (lent)
"""
import argparse
import json
import sys
import time

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

from hexapod import config as C
from hexapod.env import HexapodEnv
from hexapod.utils import new_run_dir, resolve_run, set_latest


def make_env(rank, seed, max_steps, gui=False):
    def _init():
        env = HexapodEnv(render_mode="human" if gui else None, max_steps=max_steps)
        env = Monitor(env)
        env.reset(seed=seed + rank)
        return env
    return _init


class SaveLatest(BaseCallback):
    """Sauvegarde model.zip + vecnormalize.pkl régulièrement (utilisable même si tu coupes)."""

    def __init__(self, run_dir, every_steps, n_envs):
        super().__init__()
        self.run_dir = run_dir
        self.every = max(every_steps // n_envs, 1)

    def _on_step(self):
        if self.n_calls % self.every == 0:
            save_all(self.model, self.run_dir)
        return True


def save_all(model, run_dir):
    model.save(str(run_dir / "model.zip"))
    vn = model.get_vec_normalize_env()
    if vn is not None:
        vn.save(str(run_dir / "vecnormalize.pkl"))


def main():
    ap = argparse.ArgumentParser(description="Entraîner l'hexapode (PPO)")
    ap.add_argument("--steps", type=int, default=1_000_000, help="nombre de pas de simulation")
    ap.add_argument("--envs", type=int, default=8, help="simulations en parallèle")
    ap.add_argument("--name", default=None, help="nom ajouté au dossier d'entraînement")
    ap.add_argument("--resume", default=None, help="'latest' ou nom/chemin d'un entraînement")
    ap.add_argument("--gui", action="store_true", help="afficher la simulation (1 env, lent)")
    ap.add_argument("--episode-steps", type=int, default=600, help="durée d'un épisode (pas de 1/30 s)")
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--save-every", type=int, default=50_000)
    args = ap.parse_args()

    n_envs = 1 if args.gui else max(1, args.envs)
    fns = [make_env(i, args.seed, args.episode_steps, args.gui) for i in range(n_envs)]
    venv = DummyVecEnv(fns) if n_envs == 1 else SubprocVecEnv(fns)

    if args.resume:
        run_dir = resolve_run(args.resume)
        if run_dir is None:
            print(f"Aucun entraînement trouvé pour '{args.resume}'.")
            sys.exit(1)
        print(f"Reprise de {run_dir.name}")
        vec = VecNormalize.load(str(run_dir / "vecnormalize.pkl"), venv)
        vec.training = True
        model = PPO.load(str(run_dir / "model.zip"), env=vec, device="cpu",
                         tensorboard_log=str(run_dir / "tb"))
        reset_counter = False
    else:
        run_dir = new_run_dir(args.name)
        print(f"Nouvel entraînement : {run_dir.name}")
        vec = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
        model = PPO(
            "MlpPolicy", vec, device="cpu", seed=args.seed, verbose=1,
            learning_rate=args.lr, n_steps=max(8192 // n_envs, 512), batch_size=1024,
            n_epochs=10, target_kl=0.03, gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0,
            policy_kwargs=dict(net_arch=dict(pi=[256, 256], vf=[256, 256]), log_std_init=-2.0),
            tensorboard_log=str(run_dir / "tb"),
        )
        reset_counter = True

    cfg = vars(args).copy()
    cfg.update(run=run_dir.name, started=time.strftime("%Y-%m-%d %H:%M:%S"),
               total_timesteps_before=int(model.num_timesteps))
    (run_dir / "config.json").write_text(json.dumps(cfg, indent=2), encoding="utf-8")
    set_latest(run_dir)

    callbacks = CallbackList([
        SaveLatest(run_dir, args.save_every, n_envs),
        CheckpointCallback(save_freq=max(args.save_every * 4 // n_envs, 1),
                           save_path=str(run_dir / "checkpoints"), name_prefix="ppo",
                           save_vecnormalize=True),
    ])

    t0 = time.time()
    try:
        model.learn(total_timesteps=args.steps, callback=callbacks,
                    reset_num_timesteps=reset_counter, tb_log_name="PPO")
    except KeyboardInterrupt:
        print("\nInterrompu : sauvegarde en cours…")
    finally:
        save_all(model, run_dir)
        cfg["total_timesteps"] = int(model.num_timesteps)
        cfg["duration_min"] = round((time.time() - t0) / 60, 1)
        (run_dir / "config.json").write_text(json.dumps(cfg, indent=2), encoding="utf-8")
        vec.close()
        print(f"Modèle sauvegardé dans {run_dir}")
        print("Pour le voir :      ./run.sh play")
        print("Pour le piloter :   ./run.sh control")


if __name__ == "__main__":
    main()
