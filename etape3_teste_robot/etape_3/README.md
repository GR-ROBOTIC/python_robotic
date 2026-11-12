# Étape 3 — Bras, marche et danses

Robot humanoïde à **16 servos MG996R** : 12 dans les jambes, 4 dans les bras. Il marche dans la direction demandée grâce à un réseau de neurones entraîné par renforcement (comme à l'étape 2), et il exécute **8 danses et mouvements** programmés, déclenchés au clavier.

| Étape | Contenu |
|---|---|
| 1 | Bipède 12 servos, marche tout droit |
| 2 | Bipède 12 servos, marche dans toutes les directions |
| **3** | **Humanoïde 16 servos (bras), marche + danses et mouvements** |

## Le robot

Deux servos par bras, montés à l'épaule :

- **épaule_tangage** : lève le bras vers l'avant, jusqu'au-dessus de la tête, ou vers l'arrière ;
- **épaule_roulis** : écarte le bras sur le côté, jusqu'à l'horizontale.

Pourquoi seulement 2 servos par bras ? L'ESP32 pilote **au maximum 16 servos en direct**, et les jambes en utilisent déjà 12. Avec un coude en plus, il faudrait une carte PCA9685.

Le robot mesure environ 57 cm et pèse 2,1 kg.

## Deux cerveaux différents

**La marche** est apprise par renforcement. Le réseau pilote les 12 servos des jambes. Pendant l'entraînement, les bras bougent au hasard, pour que le robot apprenne à garder l'équilibre quoi que fassent ses bras. Le réseau reçoit 27 valeurs, contre 23 à l'étape 2 : la position des 4 bras s'ajoute.

**Les danses** sont des chorégraphies programmées : une suite de poses clés (images clés), avec une transition douce entre elles. Elles pilotent les 16 servos et **ne demandent aucun entraînement**, vous pouvez les tester tout de suite. Elles sont écrites une seule fois, dans `choregraphies.py`, et exportées telles quelles vers l'ESP32 : la simulation et le vrai robot dansent exactement pareil.

## Contenu du dossier

```
etape_3/
├── lancer.sh              Menu pour tout lancer
├── README.md              Ce fichier
│
├── bipede_16servos.urdf   Le corps du robot (avec les bras)
├── robot_commun.py        Ordre des 16 servos, pose de repos, réglages communs
├── choregraphies.py       Les 8 danses et mouvements (à modifier pour créer les vôtres)
├── test_robot.py          16 curseurs pour bouger chaque servo
├── danser.py              Tester les danses (sans modèle entraîné)
├── env_bipede.py          Environnement d'entraînement de la marche
├── entrainer.py           Entraînement PPO
├── jouer.py               Marcher + danser au clavier avec le modèle entraîné
├── exporter_esp32.py      Export vers l'ESP32 (danses, et cerveau s'il existe)
├── comparer_pc.py         Mesure la vitesse de simulation du PC
│
└── robot_esp32/           Projet Arduino (ouvrir robot_esp32.ino)
    ├── robot_esp32.ino    Programme principal : IMU, modes, marche, danses, sécurité
    ├── calibration.h      VOS réglages : broches, sens, centres, décalages, IMU
    ├── danses.h           Servos + chorégraphies (généré, déjà fourni)
    ├── reseau.h           Calcul du réseau de marche (ne pas modifier)
    └── politique.h        Cerveau de marche (créé par l'export APRÈS l'entraînement)
```

## Ordre d'utilisation

```bash
source ~/anaconda/bin/activate
cd etape_3
chmod +x lancer.sh
./lancer.sh
```

1. **Option 1** : installer les bibliothèques (si ce n'est pas déjà fait).
2. **Option 2** : vérifier le robot avec les curseurs.
3. **Option 3** : **tester les danses tout de suite**, sans entraînement.
4. **Option 4** : vérifier que l'environnement fonctionne.
5. **Option 5** : entraîner la marche (15 millions de pas conseillés). Dans un 2e terminal : `./lancer.sh courbes`.
6. **Option 8** : marcher et danser au clavier.
7. **Option 10** : exporter vers l'ESP32, puis téléverser `robot_esp32/robot_esp32.ino`.

L'ESP32 peut **danser avant même l'entraînement** : `danses.h` est déjà fourni. Sans `politique.h`, la marche est simplement désactivée.

## Les commandes

Dans la simulation, cliquez d'abord dans la fenêtre PyBullet.

| Action | Simulation | ESP32 (Moniteur série) |
|---|---|---|
| **a** Salut (mouvement) | a | a |
| **b** Victoire (mouvement) | b | b |
| **c** Révérence (mouvement) | c | c |
| **d** Flexions (mouvement) | d | d |
| **e** Balancement (danse) | e | e |
| **f** Vague (danse) | f | f |
| **g** Robot (danse) | g | g |
| **h** Disco (danse) | h | h |
| Arrêter la danse | Espace | x |
| Avancer / reculer | Flèche haut / bas | 8 / 2 |
| Tourner gauche / droite | Flèche gauche / droite | 4 / 6 |
| Pas chassé gauche / droite | Page préc. / suiv. | 7 / 9 |
| S'arrêter (marche sur place) | Espace | 5 |
| Relever le robot | Entrée | — |

Sur l'ESP32 uniquement : `l` libre (servos coupés), `r` repos, `m` marche, `z` zéro (calibration), `i` afficher l'IMU, `t` autotest.

Une danse se lance depuis le repos ou pendant la marche. À la fin, le robot revient au repos, puis reprend la marche s'il marchait avant.

## Les danses, vérifiées en simulation

Chaque chorégraphie a été jouée en simulation, robot posé au sol, avec les limites des MG996R. Aucune chute.

| Touche | Nom | Durée | Servo le plus sollicité |
|---|---|---|---|
| a | Salut | 4,0 s | 28 % |
| b | Victoire | 4,2 s | 30 % |
| c | Révérence | 4,2 s | 95 % (bref) |
| d | Flexions | 5,2 s | 67 % |
| e | Balancement | 8,4 s | 100 % (bref) |
| f | Vague | 5,2 s | 65 % |
| g | Robot | 5,6 s | 100 % (bref) |
| h | Disco | 9,9 s | 78 % |

« Bref » : le servo dépasse 90 % de sa force pendant moins de 7 % de la danse, seulement au moment des changements rapides. Sur le vrai robot, commencez par les danses a, b, d et f, qui sont les moins exigeantes.

Une rotation du bassin (« twist ») a été essayée puis retirée : avec les deux pieds au sol, les jambes se contredisent et les servos forcent en permanence.

## Créer votre propre danse

Dans `choregraphies.py`, copiez un bloc de `DANSES` et modifiez-le. Chaque image clé est un temps en battements et une pose :

```python
"e": {
    "nom": "Balancement", "type": "danse", "tempo": 100, "boucles": 3,
    "images": [
        (0.0, pose(flexion=0.2)),
        (1.0, pose(flexion=0.2, balance=0.12, g=(-0.3, 0.7))),
        ...
    ],
},
```

Les réglages de `pose()` sont les suivants :

- `flexion` : de 0 (repos) à 1 (accroupi) ;
- `balance` : déplace le bassin sur le côté, entre -0.12 et 0.12 environ ;
- `penche` : penche le buste en avant, jusqu'à 0.4 ;
- `g=(tangage, écart)` et `d=(tangage, écart)` pour chaque bras. Le tangage va de 0 (le long du corps) à -1.57 (devant) et -2.3 (en haut). L'écart va de 0 à 1.5 (sur le côté).

Testez avec `python danser.py`, en surveillant le pourcentage du servo le plus chargé affiché à la fin. Puis exportez avec l'option 11.

## Matériel et calibration

Le matériel est le suivant : un ESP32 avec un shield servos, piloté par la bibliothèque **ESP32Servo**, 16 servos, et une IMU MPU6050 sur SDA 21 / SCL 22.

Dans `robot_esp32/calibration.h` :

1. **`BROCHE`** : les 16 broches de votre shield, dans l'ordre officiel. Les broches par défaut sont un exemple.
2. **`SENS`** : mettez -1 pour chaque servo qui tourne à l'envers de la simulation.
3. **`CENTRE_DEG`** : l'angle de la simulation qui correspond au milieu du servo. Il vaut 0 pour les jambes et les écarts de bras, et **-45 pour le tangage des épaules**. Ainsi le servo couvre de « bras levé au-dessus de la tête » à « bras vers l'arrière ». Montez le palonnier pour que le servo à 90° place le bras à 45° vers l'avant-bas.
4. **`DECALAGE_DEG`** : la correction fine. Avec la commande `z`, les jambes doivent être droites et les bras le long du corps.
5. **`IMU_AXE` / `IMU_SIGNE`** : l'orientation de l'IMU, comme à l'étape 2.

Faites les premiers essais robot suspendu, en commençant par les danses les plus douces.

## Différences avec l'étape 2

| | Étape 2 | Étape 3 |
|---|---|---|
| Servos | 12 | **16** (+ 2 par bras) |
| Masse | 1,75 kg | 2,1 kg |
| Entrées du réseau | 23 | **27** (+ position des bras) |
| Bras pendant l'entraînement | — | Mouvements aléatoires (équilibre robuste) |
| Physique | 50 itérations | 10 itérations (~2x plus rapide, stable) |
| Danses | — | **8 chorégraphies**, touches a à h |
| Visualisation | `voir_robot.py` | `jouer.py` (marche + danses), `danser.py` (danses seules) |
| ESP32 | `bipede_esp32/` | `robot_esp32/`, fonctionne même sans cerveau |

Les modèles de l'étape 2 ne sont **pas compatibles** avec l'étape 3 : il faut refaire l'entraînement.
