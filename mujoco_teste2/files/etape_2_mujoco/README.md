# Étape 2 — MuJoCo : apprendre à marcher

Même objectif que l'étape 2 avec PyBullet : entraîner le bipède à 12 servos MG996R à **marcher dans la direction demandée** (avancer, reculer, tourner, pas chassé). La seule différence : la physique est simulée par **MuJoCo**.

Les règles du jeu sont **identiques** (mêmes 23 observations, mêmes 12 actions, mêmes consignes, même récompense). Le cerveau exporté fonctionne donc avec **exactement le même programme ESP32** que l'étape 2 PyBullet.

## Contenu

```
etape_2_mujoco/
├── lancer.sh            Menu pour tout lancer
├── README.md            Ce fichier
├── bipede.xml           Le robot au format MuJoCo : corps, servos MG996R, IMU, capteurs de pieds
├── voir.py              Voir le robot et bouger chaque servo avec des curseurs
├── env_bipede.py        Règles du jeu (observations, actions, consignes, récompense) avec MuJoCo
├── entrainer.py         Entraînement PPO (identique à PyBullet)
├── jouer.py             Piloter le robot entraîné au clavier
├── exporter_esp32.py    Export du cerveau vers bipede_esp32/politique.h
├── comparer_pc.py       Mesure la vitesse d'entraînement de ce PC
└── bipede_esp32/        Programme ESP32 (identique à l'étape 2, version ESP32Servo)
```

## Utilisation

```bash
source ~/anaconda/bin/activate
cd etape_2_mujoco
chmod +x lancer.sh
./lancer.sh
```

1. **Option 1** : installer MuJoCo et les bibliothèques d'entraînement.
2. **Option 2** : voir le robot ; le panneau de droite, section Control, a un curseur par servo.
3. **Option 3** : vérifier que le robot tient debout.
4. **Option 4** : entraîner (15 millions de pas conseillés). Dans un 2e terminal : `./lancer.sh courbes`.
5. **Option 8** : piloter le robot entraîné au clavier.
6. **Option 9** : exporter vers l'ESP32, puis téléverser `bipede_esp32/bipede_esp32.ino`.

### Clavier (fenêtre MuJoCo)

| Touche | Action |
|---|---|
| Flèche haut / bas | Avancer / reculer |
| Flèche gauche / droite | Tourner à gauche / à droite |
| Page préc. / Page suiv. | Pas chassé à gauche / à droite |
| Espace | S'arrêter (marche sur place) |
| Entrée | Remettre le robot debout |

La consigne et la vitesse réelle s'affichent en haut à gauche de la fenêtre. Sur l'ESP32, les commandes ne changent pas : pavé numérique 2/4/5/6/7/8/9, `r` repos, `m` marche, etc.

## Ce qui change par rapport à PyBullet

| | Étape 2 PyBullet | Étape 2 MuJoCo |
|---|---|---|
| Fichier du robot | `bipede_12servos.urdf` | `bipede.xml` (servos et capteurs inclus) |
| Moteurs | Réglés dans le code Python | Déclarés dans `bipede.xml` (couple max 1 N·m) |
| Vitesse max des servos | Option de PyBullet | Simulée dans `env_bipede.py` : la cible avance à 7 rad/s au plus |
| Inclinaison et rotation du buste | Calculées par PyBullet | Lues sur les capteurs `orientation` et `gyro` |
| Pied au sol ou non | Recherche des points de contact | Capteurs de contact sous les pieds (`touch`) |
| Randomisation | Masses, frottement, force des servos | Identique |
| Vitesse de l'environnement (PC de test, 1 cœur) | ~1 500 à 2 000 pas/s | **~7 700 pas/s** |
| Programme ESP32 | `bipede_esp32/` | **Le même** |

L'environnement est environ **4 fois plus rapide** que la version PyBullet. Une partie du temps d'entraînement est prise par l'apprentissage du réseau lui-même, mais l'entraînement complet devrait être nettement plus court. Mesurez avec l'option 10.

## Vérifié

- L'environnement passe les vérifications de Gymnasium, et le robot tient debout dans sa pose de repos.
- Les conventions sont les mêmes que sur l'ESP32 : pencher le robot vers l'avant donne une gravité X positive, vers la gauche une gravité Y positive.
- L'ordre des 12 servos est identique à l'étape 2 et au programme ESP32.
- Le fichier exporté compile avec le programme ESP32, et l'autotest du réseau passe.

## Une idée de test : le test croisé

Comme les deux projets ont les mêmes règles du jeu, un cerveau entraîné ici et un cerveau entraîné avec PyBullet sont interchangeables pour l'ESP32. Si un cerveau marche bien dans les deux simulateurs, c'est très bon signe pour le vrai robot : sa démarche ne dépend pas des particularités d'un moteur physique.

Les modèles ne sont pas mélangés : chaque projet a ses propres dossiers `modeles/` et `logs/`.
