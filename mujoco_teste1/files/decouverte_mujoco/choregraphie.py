"""
Petite chorégraphie de démonstration, partagée par demo.py.
Mêmes conventions que les projets PyBullet : angles en radians autour de la pose de repos.
"""
import numpy as np

NOMS = ["hanche_lacet", "hanche_roulis", "hanche_tangage", "genou", "cheville_tangage", "cheville_roulis"]


def pose(flexion=0.0, balance=0.0, lever_g=0.0):
    """flexion : 0 repos -> 1 accroupi | balance : bassin vers la droite (+) |
    lever_g : 0 -> 1 lève le pied gauche (à combiner avec balance > 0.2)"""
    angles = []
    for cote in "gd":
        hanche, genou, cheville = -0.3 - 0.3 * flexion, 0.6 + 0.6 * flexion, -0.3 - 0.3 * flexion
        if cote == "g":
            hanche -= 0.4 * lever_g
            genou += 0.8 * lever_g
            cheville -= 0.4 * lever_g
        angles += [0.0, balance, hanche, genou, cheville, -balance]
    return np.array(angles)


# (temps en secondes, pose, nom de l'étape)
SEQUENCE = [
    (0.0, pose(), "Repos"),
    (1.0, pose(), "Repos"),
    (2.0, pose(flexion=0.8), "Flexion"),
    (3.0, pose(), "Flexion"),
    (4.0, pose(flexion=0.8), "Flexion"),
    (5.0, pose(), "Flexion"),
    (6.0, pose(balance=0.12), "Balancement"),
    (7.0, pose(), "Balancement"),
    (8.0, pose(balance=-0.12), "Balancement"),
    (9.0, pose(), "Balancement"),
    (10.0, pose(balance=0.22), "Bascule sur le pied droit"),
    (11.0, pose(balance=0.22, lever_g=1.0), "Sur un pied !"),
    (13.0, pose(balance=0.22, lever_g=1.0), "Sur un pied !"),
    (14.0, pose(balance=0.22), "Retour"),
    (15.0, pose(), "Retour"),
    (16.0, pose(), "Repos"),
]
DUREE = SEQUENCE[-1][0]


def consigne(t):
    """Angles à l'instant t (la séquence tourne en boucle), avec transitions en S."""
    t = t % DUREE
    for (t0, a0, nom), (t1, a1, _) in zip(SEQUENCE, SEQUENCE[1:]):
        if t <= t1:
            u = (t - t0) / (t1 - t0)
            s = u * u * (3 - 2 * u)
            return a0 + (a1 - a0) * s, nom
    return SEQUENCE[-1][1], SEQUENCE[-1][2]
