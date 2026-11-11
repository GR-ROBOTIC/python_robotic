# Projet bipède — version direction

Robot bipède à 12 servos MG996R qui apprend à marcher **dans la direction demandée** (avancer, reculer, tourner, pas chassé) par apprentissage par renforcement, puis qui exécute ce cerveau sur un ESP32.

## Contenu du dossier

```
projet_bipede_direction/
├── lancer.sh              Menu pour tout lancer (./lancer.sh)
├── README.md              Ce fichier
│
├── bipede_12servos.urdf   Le corps du robot (dimensions, masses, articulations)
├── test_bipede.py         Test manuel : 12 curseurs pour bouger chaque servo
├── env_bipede.py          Règles du jeu : ce que le robot perçoit, ce qu'il fait, la récompense
├── entrainer.py           Entraînement PPO (crée modeles/ et logs/)
├── voir_robot.py          Piloter le robot entraîné au clavier
├── exporter_esp32.py      Convertit le cerveau en fichier C pour l'ESP32
│
└── bipede_esp32/          Projet Arduino (ouvrir bipede_esp32.ino)
    ├── bipede_esp32.ino   Programme principal : IMU, modes, sécurité, direction
    ├── calibration.h      VOS réglages : broches, sens, décalages, orientation de l'IMU
    ├── reseau.h           Calcul du réseau de neurones (ne pas modifier)
    └── politique.h        Créé par exporter_esp32.py (ne pas modifier à la main)
```

Les dossiers `modeles/` et `logs/` apparaissent pendant l'entraînement.

## Ordre d'utilisation

```bash
source ~/anaconda/bin/activate
cd projet_bipede_direction
chmod +x lancer.sh
./lancer.sh
```

1. **Option 1** : installer les bibliothèques (une seule fois).
2. **Option 3** : vérifier que le robot tient debout.
3. **Option 4** : entraîner (15 millions de pas conseillés). Dans un 2e terminal : `./lancer.sh courbes`.
4. **Option 7** : piloter le robot au clavier dans la simulation.
5. **Option 9** : exporter vers l'ESP32, puis téléverser `bipede_esp32/bipede_esp32.ino`.

## Commandes

| Simulation (fenêtre PyBullet) | ESP32 (Moniteur série) | Action |
|---|---|---|
| Flèche haut | 8 | Avancer |
| Flèche bas | 2 | Reculer |
| Flèche gauche | 4 | Tourner à gauche |
| Flèche droite | 6 | Tourner à droite |
| Page précédente | 7 | Pas chassé à gauche |
| Page suivante | 9 | Pas chassé à droite |
| Espace | 5 | Marcher sur place |

Sur l'ESP32 : `l` libre, `r` repos, `m` marche, `z` zéro (calibration), `i` afficher l'IMU, `t` autotest.

## Différences avec la première version (marche tout droit)

| | Version 1 | Version direction |
|---|---|---|
| Entrées du réseau | 20 | **23** (+ consigne : avant, latéral, rotation) |
| Objectif appris | Avancer à 0,15 m/s | **Suivre la consigne**, tirée au hasard et changée toutes les 4 s |
| Récompense de vitesse | Écart à 0,15 m/s vers l'avant | Écart à la consigne, **dans le repère du robot** |
| Rotation | Pénalisée (rester droit) | **Récompensée si elle suit la consigne** |
| Dérive latérale | Pénalisée | Autorisée si demandée (pas chassé) |
| Entraînement conseillé | 5 M de pas | 15 M de pas |
| `voir_robot.py` | Regarder | **Piloter au clavier** |
| ESP32 | Marche tout droit | Touches 2/4/5/6/7/8/9 pour la direction |

Fichiers identiques dans les deux versions : `bipede_12servos.urdf`, `test_bipede.py`, `bipede_esp32/reseau.h`, et `bipede_esp32/calibration.h` (version ESP32Servo).

Les modèles ne sont **pas compatibles** entre les deux versions (20 entrées contre 23) : chaque projet a ses propres dossiers `modeles/`.

## Matériel

ESP32 + shield servos (servos pilotés directement par la bibliothèque **ESP32Servo**) + IMU MPU6050 sur SDA 21 / SCL 22. Renseignez les broches de votre shield dans `bipede_esp32/calibration.h` (tableau `BROCHE`).
