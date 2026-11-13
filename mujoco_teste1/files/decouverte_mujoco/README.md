# Découverte de MuJoCo

Petit projet pour voir à quoi ressemble **MuJoCo**, le simulateur open source de Google DeepMind, avec **le même bipède à 12 servos que l'étape 2**. Pas d'entraînement ici : on regarde le robot, on le pilote, on lit ses capteurs, et on compare la vitesse avec PyBullet.

## Contenu

```
decouverte_mujoco/
├── lancer.sh              Menu
├── README.md              Ce fichier
├── bipede.xml             Le robot au format MuJoCo (MJCF) : corps, moteurs, capteurs
├── bipede_12servos.urdf   Le même robot au format URDF (pour la comparaison avec PyBullet)
├── voir.py                Visualiseur interactif avec un curseur par servo
├── demo.py                Chorégraphie en boucle + lecture des capteurs dans le terminal
├── choregraphie.py        Les poses de la démonstration
└── vitesse.py             Course de vitesse MuJoCo contre PyBullet
```

## Utilisation

```bash
source ~/anaconda/bin/activate
cd decouverte_mujoco
chmod +x lancer.sh
./lancer.sh
```

1. **Option 1** : installer MuJoCo (`pip install mujoco`, quelques secondes).
2. **Option 2** : voir le robot. Dans le panneau de droite, la section **Control** contient un curseur par servo, sans aucune ligne de code.
3. **Option 3** : la démonstration. Le robot fait des flexions, se balance, puis **tient sur un pied**, pendant que le terminal affiche l'inclinaison, la force sous chaque pied et le servo le plus chargé.
4. **Option 4** : la course de vitesse contre PyBullet.

Dans la fenêtre MuJoCo :

- souris pour tourner la caméra, molette pour zoomer ;
- **double-clic sur une pièce, puis Ctrl + clic droit glissé** pour pousser le robot ;
- touches **C** (points de contact), **F** (forces de contact), **T** (transparence) ;
- Espace pour la pause, Retour arrière pour réinitialiser. La réinitialisation met les jambes droites ; pour revenir à la pose de repos, chargez la pose enregistrée (« Key ») depuis la section Simulation du panneau de gauche, ou relancez simplement le script.

## Ce qui change par rapport à PyBullet

**1. Le robot décrit tout dans un seul fichier.** Dans `bipede.xml`, on trouve le corps (comme dans l'URDF), mais aussi :

- **les moteurs** (`<actuator>`) : chaque MG996R est un actionneur asservi en position, avec son couple limité à 1 N·m. Dans PyBullet, ces réglages étaient dans le code Python ;
- **les capteurs** (`<sensor>`) : un accéléromètre et un gyroscope (la vraie IMU), l'orientation, et des **capteurs de contact sous les pieds**. Dans PyBullet, on reconstituait tout ça par calcul ;
- **les poses enregistrées** (`<keyframe>`) : la pose de repos est définie dans le fichier ;
- **les classes** (`<default>`) : les réglages communs sont écrits une seule fois (« tous les servos ont ce couple »).

**2. Les capteurs se lisent directement**, comme sur un vrai robot :

```python
acc = d.sensor("accelero").data            # comme le MPU6050
force = d.sensor("contact_pied_g").data    # force sous le pied gauche, en newtons
```

**3. Le pas de simulation est explicite** : on choisit soi-même quand avancer (`mujoco.mj_step`), et le visualiseur suit.

**4. C'est plus rapide.** Sur le PC de test, avec un seul cœur :

| | Décisions par seconde | Par rapport au temps réel |
|---|---|---|
| MuJoCo | ~10 900 | ~217 x plus vite |
| PyBullet | ~1 800 | ~36 x plus vite |

Soit **environ 6 x plus rapide** pour la physique. Lancez l'option 4 pour mesurer sur votre PC. Pour un entraînement, la physique n'est pas tout (le réseau doit aussi apprendre), mais le temps total baisserait nettement, avec le même processeur.

## Ce que montre la démonstration

Le passage « **Sur un pied** » est instructif. Le robot tient, mais le terminal signale que le **servo de roulis de la hanche droite est saturé** : il travaille à 100 % de son couple pendant tout ce temps. Sur le vrai robot, un MG996R dans cette situation chaufferait et risquerait de céder. C'est exactement le genre de chose qu'on veut découvrir en simulation **avant** de construire.

## Et après ?

On pourrait refaire l'entraînement de la marche (étape 2) avec MuJoCo. La logique (observations, consignes, récompense, export ESP32) resterait la même ; seule la partie « physique » de `env_bipede.py` serait réécrite. MuJoCo a aussi une version sur carte graphique (**MJX**), pour faire tourner des milliers de robots en parallèle le jour où vous aurez une carte NVIDIA adaptée.

MuJoCo sait aussi ouvrir directement certains fichiers URDF. Pour un export Fusion 360, on peut donc partir de l'URDF puis le convertir en MJCF pour ajouter moteurs et capteurs.
