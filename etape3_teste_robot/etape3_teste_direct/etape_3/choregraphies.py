"""
Bibliothèque des danses et mouvements (chorégraphies programmées).

Chaque chorégraphie = une liste d'images clés (temps en battements, pose).
Entre deux images clés, le mouvement est interpolé en douceur (courbe en S).
Le MÊME fichier sert à la simulation et, via exporter_esp32.py, à l'ESP32.

Pour créer votre propre danse : copiez un bloc de DANSES, changez la touche,
le nom et les images clés, puis testez avec  python danser.py
"""
import numpy as np

from robot_commun import BRAS, JAMBES, ORDRE_SERVOS, POSE_REPOS

DUREE_ENTREE = 0.6   # s : transition depuis la pose actuelle vers la 1re image
DUREE_SORTIE = 0.6   # s : retour à la pose de repos à la fin


# ----------------------------------------------------------------------
#  Outils pour écrire des poses lisibles
# ----------------------------------------------------------------------
def pose(flexion=0.0, balance=0.0, penche=0.0, g=(0.0, 0.0), d=(0.0, 0.0)):
    """
    flexion : 0 = repos, 1 = accroupi (plie hanches, genoux, chevilles ensemble)
    balance : déplace le bassin sur le côté (rad, + = vers la droite du robot)
    penche  : penche le buste vers l'avant (rad), pieds à plat
    g, d    : bras gauche / droit = (tangage, écart)
              tangage : 0 = bras le long du corps, -1.57 = tendu devant, -2.3 = levé haut
                        valeurs positives = vers l'arrière
              écart   : 0 = contre le corps, 1.5 = tendu sur le côté

    Remarque : pas de rotation du bassin (hanche_lacet) quand les deux pieds sont au sol :
    les jambes se contrediraient et les servos forceraient en permanence (vérifié en simulation).
    """
    a = dict(zip(ORDRE_SERVOS, POSE_REPOS))
    for c in ("g", "d"):
        a[f"hanche_tangage_{c}"] = -0.3 - 0.3 * flexion - penche
        a[f"genou_{c}"] = 0.6 + 0.6 * flexion
        a[f"cheville_tangage_{c}"] = -0.3 - 0.3 * flexion
        a[f"hanche_roulis_{c}"] = balance
        a[f"cheville_roulis_{c}"] = -balance
    a["epaule_tangage_g"], a["epaule_roulis_g"] = g[0], g[1]
    a["epaule_tangage_d"], a["epaule_roulis_d"] = d[0], -d[1]      # bras droit en miroir
    return np.array([a[n] for n in ORDRE_SERVOS])


REPOS = pose()

# ----------------------------------------------------------------------
#  Les chorégraphies  (touche : clavier de la simulation ET Moniteur série ESP32)
# ----------------------------------------------------------------------
DANSES = {
    "a": {
        "nom": "Salut", "type": "mouvement", "tempo": 120, "boucles": 1,
        "images": [
            (0.0, REPOS),
            (1.5, pose(d=(-2.0, 0.3))),
            (2.0, pose(d=(-2.0, 0.9))),
            (2.5, pose(d=(-2.0, 0.3))),
            (3.0, pose(d=(-2.0, 0.9))),
            (3.5, pose(d=(-2.0, 0.3))),
            (4.0, pose(d=(-2.0, 0.9))),
            (5.5, REPOS),
        ],
    },
    "b": {
        "nom": "Victoire", "type": "mouvement", "tempo": 100, "boucles": 1,
        "images": [
            (0.0, REPOS),
            (1.0, pose(flexion=0.5, g=(-0.6, 0.0), d=(-0.6, 0.0))),
            (2.0, pose(flexion=0.0, g=(-2.2, 0.3), d=(-2.2, 0.3))),
            (3.5, pose(flexion=0.0, g=(-2.2, 0.3), d=(-2.2, 0.3))),
            (5.0, REPOS),
        ],
    },
    "c": {
        "nom": "Révérence", "type": "mouvement", "tempo": 90, "boucles": 1,
        "images": [
            (0.0, REPOS),
            (1.5, pose(penche=0.4, flexion=0.2, g=(0.5, 0.1), d=(0.5, 0.1))),
            (3.0, pose(penche=0.4, flexion=0.2, g=(0.5, 0.1), d=(0.5, 0.1))),
            (4.5, REPOS),
        ],
    },
    "d": {
        "nom": "Flexions", "type": "mouvement", "tempo": 90, "boucles": 3,
        "images": [
            (0.0, REPOS),
            (1.0, pose(flexion=0.8, g=(-1.5, 0.0), d=(-1.5, 0.0))),
            (2.0, REPOS),
        ],
    },
    "e": {
        "nom": "Balancement", "type": "danse", "tempo": 100, "boucles": 3,
        "images": [
            (0.0, pose(flexion=0.2)),
            (1.0, pose(flexion=0.2, balance=0.12, g=(-0.3, 0.7), d=(0.0, 0.1))),
            (2.0, pose(flexion=0.2)),
            (3.0, pose(flexion=0.2, balance=-0.12, g=(0.0, 0.1), d=(-0.3, 0.7))),
            (4.0, pose(flexion=0.2)),
        ],
    },
    "f": {
        "nom": "Vague", "type": "danse", "tempo": 120, "boucles": 2,
        "images": [   # une vague qui traverse les bras de gauche à droite
            (0.0, pose(flexion=0.2, g=(0.0, 0.2), d=(0.0, 0.2))),
            (1.0, pose(flexion=0.4, g=(-0.3, 1.5), d=(0.0, 0.2))),
            (2.0, pose(flexion=0.2, g=(-1.2, 1.0), d=(-1.2, 1.0))),
            (3.0, pose(flexion=0.4, g=(0.0, 0.2), d=(-0.3, 1.5))),
            (4.0, pose(flexion=0.2, g=(0.0, 0.2), d=(0.0, 0.2))),
        ],
    },
    "g": {
        "nom": "Robot", "type": "danse", "tempo": 110, "boucles": 2,
        "images": [   # mouvements secs : transitions très courtes puis pauses
            (0.0, pose(flexion=0.2)),
            (0.2, pose(flexion=0.2, g=(-1.57, 0.0), d=(0.0, 1.4))),
            (1.0, pose(flexion=0.2, g=(-1.57, 0.0), d=(0.0, 1.4))),
            (1.2, pose(flexion=0.2, g=(0.0, 1.4), d=(-1.57, 0.0))),
            (2.0, pose(flexion=0.2, g=(0.0, 1.4), d=(-1.57, 0.0))),
            (2.2, pose(flexion=0.4, g=(-1.57, 0.0), d=(-1.57, 0.0))),
            (3.0, pose(flexion=0.4, g=(-1.57, 0.0), d=(-1.57, 0.0))),
            (3.2, pose(flexion=0.2)),
            (4.0, pose(flexion=0.2)),
        ],
    },
    "h": {
        "nom": "Disco", "type": "danse", "tempo": 110, "boucles": 2,
        "images": [
            (0.0, pose(flexion=0.3)),
            (1.0, pose(flexion=0.0, d=(-2.2, 0.5), g=(0.2, 0.1))),
            (2.0, pose(flexion=0.4, d=(-0.5, 0.0), g=(0.2, 0.1))),
            (3.0, pose(flexion=0.0, d=(-2.2, 0.5), g=(0.2, 0.1))),
            (4.0, pose(flexion=0.3)),
            (5.0, pose(flexion=0.0, g=(-2.2, 0.5), d=(0.2, 0.1))),
            (6.0, pose(flexion=0.4, g=(-0.5, 0.0), d=(0.2, 0.1))),
            (7.0, pose(flexion=0.0, g=(-2.2, 0.5), d=(0.2, 0.1))),
            (8.0, pose(flexion=0.3)),
        ],
    },
}


# ----------------------------------------------------------------------
#  Calcul des images clés en secondes (boucles déroulées)
# ----------------------------------------------------------------------
def images_en_secondes(danse):
    """Liste [(temps_s, angles16)] du cycle complet, en partant de t = 0 à la 1re image."""
    sec = 60.0 / danse["tempo"]
    cycle = danse["images"][-1][0]
    res = []
    for k in range(danse["boucles"]):
        for b, angles in danse["images"]:
            t = (k * cycle + b) * sec
            if res and t <= res[-1][0] + 1e-9:
                continue
            res.append((t, np.asarray(angles, dtype=float)))
    return res


def interpoler(images, t):
    """Position à l'instant t, avec une transition en S entre les images clés."""
    if t <= images[0][0]:
        return images[0][1]
    for (t0, a0), (t1, a1) in zip(images, images[1:]):
        if t <= t1:
            u = (t - t0) / (t1 - t0)
            s = u * u * (3 - 2 * u)
            return a0 + (a1 - a0) * s
    return images[-1][1]


class Lecteur:
    """Joue une chorégraphie : entrée depuis la pose actuelle, la danse, puis retour au repos."""

    def __init__(self, touche, pose_actuelle, bas, haut):
        d = DANSES[touche]
        self.nom = d["nom"]
        corps = images_en_secondes(d)
        self.images = ([(0.0, np.asarray(pose_actuelle, dtype=float))]
                       + [(t + DUREE_ENTREE, a) for t, a in corps])
        fin = self.images[-1][0]
        self.images.append((fin + DUREE_SORTIE, REPOS))
        self.duree = self.images[-1][0]
        self.bas, self.haut = bas, haut
        self.t = 0.0

    @classmethod
    def retour_repos(cls, pose_actuelle, bas, haut):
        """Interrompt une danse : simple retour en douceur à la pose de repos."""
        l = cls.__new__(cls)
        l.nom, l.bas, l.haut, l.t = "retour au repos", bas, haut, 0.0
        l.images = [(0.0, np.asarray(pose_actuelle, dtype=float)), (DUREE_SORTIE, REPOS)]
        l.duree = DUREE_SORTIE
        return l

    def suivant(self, dt):
        """Avance de dt secondes et renvoie les 16 angles à envoyer."""
        self.t += dt
        return np.clip(interpoler(self.images, self.t), self.bas, self.haut)

    @property
    def fini(self):
        return self.t >= self.duree


def liste_touches():
    return "\n".join(f"  {k} : {v['nom']} ({v['type']})" for k, v in DANSES.items())
