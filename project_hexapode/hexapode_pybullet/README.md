# Hexapode PyBullet 🕷

Araignée à 6 pattes et 18 servos (3 par patte : hanche, cuisse, tibia), entraînée avec PPO
(stable-baselines3) dans PyBullet, puis pilotable avec des boutons.

## Démarrage

```bash
conda activate pybullet      # ton environnement existant
cd hexapode_pybullet
chmod +x run.sh
./run.sh                     # menu
```

Ordre conseillé la première fois :
1. **Option 1** : installer (ajoute stable-baselines3, gymnasium et tensorboard à ton environnement).
2. **Option 2** : voir le modèle et bouger chaque servo avec les curseurs.
3. **Option 3** : voir la démarche tripode seule, sans IA (elle marche déjà).
4. **Option 4 ou 5** : entraîner. Dans un 2ᵉ terminal : `./run.sh tb` pour les courbes.
5. **Option 9** : piloter l'araignée avec les boutons.

`run.sh` utilise l'environnement Python déjà actif s'il contient pybullet. Sinon, il
active l'environnement conda `pybullet` (nom modifiable en haut du script), et en dernier
recours il crée un `.venv`.

## Commandes directes

| Commande | Rôle |
|---|---|
| `./run.sh model` | modèle + curseurs (`--gait` : démarche animée en l'air) |
| `./run.sh gait` | démarche tripode au sol, sans IA |
| `./run.sh quick` / `long` | entraînement 300 k / 3 M pas |
| `./run.sh train --steps 2000000 --envs 12 --name essai` | entraînement personnalisé |
| `./run.sh train --gui --steps 20000` | regarder l'entraînement en direct (lent) |
| `./run.sh resume 1000000` | reprendre le dernier entraînement |
| `./run.sh play` | voir le dernier entraînement (démo de tous les mouvements) |
| `./run.sh play --cmd cercle` | une seule commande |
| `./run.sh control` | télécommande à boutons |
| `./run.sh list` | lister les entraînements |
| `./run.sh tb` | TensorBoard |
| `./run.sh urdf` | régénérer l'URDF après avoir modifié `hexapod/config.py` |

Ctrl+C pendant un entraînement : le modèle est sauvegardé avant de quitter.

## Télécommande

Boutons : Avancer, Reculer, Gauche, Droite, Tourner G, Tourner D, Cercle, Stop,
Remettre debout, plus un curseur de vitesse.
Clavier (fenêtre des boutons) : flèches, `A` / `E` pour tourner, `C` cercle,
Espace stop, `R` remettre debout.
Si la fenêtre Tkinter pose problème : `./run.sh control --ui pybullet` place les
boutons dans le panneau de la fenêtre PyBullet.

## Principe

- `hexapod/gait.py` : démarche tripode (3 pattes au sol, 3 en l'air) et cinématique
  inverse. Elle sait déjà avancer, reculer, aller en crabe et tourner.
- L'IA (PPO) ajoute une petite correction à chacun des 18 servos, 30 fois par seconde,
  pour mieux suivre la commande en restant stable. Elle reçoit l'inclinaison du corps, les
  vitesses, les angles des servos et la commande demandée.
- La récompense favorise : suivre la vitesse et la rotation demandées, rester à plat et
  à hauteur constante, bouger les servos en douceur, consommer peu.
- Pendant l'entraînement, la commande change au hasard, le frottement au sol varie et
  l'orientation de départ aussi, pour que la politique reste robuste.

## Fichiers

```
run.sh               menu / lanceur
train.py             entraînement PPO
play.py              voir un entraînement
control_gui.py       télécommande à boutons
view_model.py        modèle + curseurs
hexapod/config.py    TOUS les réglages (dimensions, masses, servos, vitesses)
hexapod/gait.py      démarche tripode + cinématique inverse
hexapod/env.py       environnement Gymnasium (observations, récompense)
hexapod/utils.py     dossiers runs/ et chargement du modèle
tools/generate_urdf.py  génère urdf/hexapod.urdf
runs/                un dossier par entraînement (model.zip, vecnormalize.pkl, tb/, checkpoints/)
```

Pour coller à ton vrai robot, modifie les dimensions, les masses et le couple des servos
dans `hexapod/config.py`, puis lance `./run.sh urdf`.

L'export vers l'ESP32 est prévu (option 12 du menu), mais pas encore écrit.
