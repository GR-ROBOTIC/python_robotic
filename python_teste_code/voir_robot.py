"""
Regarder le robot marcher avec un modèle entraîné.

    python voir_robot.py                                    # modèle final
    python voir_robot.py --modele modeles/bipede_2000000_steps.zip \
                         --stats  modeles/bipede_vecnormalize_2000000_steps.pkl
"""
import argparse
import os

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from env_bipede import BipedeEnv

DOSSIER = os.path.dirname(os.path.abspath(__file__))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--modele", default=os.path.join(DOSSIER, "modeles", "bipede_final.zip"))
    parser.add_argument("--stats", default=os.path.join(DOSSIER, "modeles", "vecnormalize_final.pkl"))
    parser.add_argument("--randomiser", action="store_true", help="tester avec frottement/masses aléatoires")
    args = parser.parse_args()

    env = DummyVecEnv([lambda: BipedeEnv(render_mode="human", randomiser=args.randomiser)])
    env = VecNormalize.load(args.stats, env)
    env.training = False          # ne plus modifier les statistiques
    env.norm_reward = False

    model = PPO.load(args.modele, device="cpu")
    obs = env.reset()
    episode, pas = 1, 0
    try:
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, fini, infos = env.step(action)
            pas += 1
            if fini[0]:
                d = infos[0].get("distance", 0.0)
                cause = "fin du temps" if infos[0].get("TimeLimit.truncated") else "chute"
                print(f"Épisode {episode} : {d:.2f} m parcourus en {pas / 50:.1f} s ({cause})")
                episode, pas = episode + 1, 0
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
