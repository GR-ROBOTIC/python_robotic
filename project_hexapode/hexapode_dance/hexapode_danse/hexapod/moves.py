"""Bibliothèque de pas de danse.

Chaque pas est une fonction f(t, B, a, P) :
  t : temps écoulé dans le pas, en temps musicaux (beats)
  B : durée totale du pas, en beats
  a : intensité (1.0 = normal)
  P : la posture à modifier (P.body, P.feet, P.cmd)

  P.body = [x, y, z, roulis, tangage, lacet]  décalage et inclinaison du corps
  P.feet = tableau (6, 3)                     décalage de chaque pied (LF, LM, LR, RF, RM, RR)
  P.cmd  = [vx, vy, rotation] dans [-1, 1]    marche pendant le pas

Chaque pas commence et finit en pose neutre (grâce aux enveloppes), donc on peut
les enchaîner dans n'importe quel ordre sans à-coup.

Ajouter un pas : écrire une fonction décorée par @move(...). Il apparaîtra
automatiquement dans la télécommande et sera utilisable dans les chorégraphies.
"""
import math

import numpy as np

from .config import LEG_INDEX as L

PI = math.pi
MOVES = {}


def move(name, label, beats=8, legs=False):
    """legs=True : le pas lève des pattes (la marche est coupée pendant ce pas)."""
    def deco(fn):
        MOVES[name] = {"fn": fn, "label": label, "beats": beats, "legs": legs}
        return fn
    return deco


class Pose:
    def __init__(self):
        self.body = np.zeros(6)
        self.feet = np.zeros((6, 3))
        self.cmd = np.zeros(3)


# ------------------------------------------------------------ enveloppes
def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def env(t, B, ramp=0.75):
    """0 au début et à la fin du pas, 1 au milieu : transitions douces."""
    return smooth(t / ramp) * smooth((B - t) / ramp)


def window(t, start, end, ramp=0.5):
    """1 entre start et end (en beats), avec montée et descente douces."""
    return smooth((t - start) / ramp) * smooth((end - t) / ramp)


def bump(x):
    """Une bosse de 0 à 1 à 0 quand x va de 0 à 1."""
    return math.sin(PI * min(max(x, 0.0), 1.0))


# ============================================================ corps seul (6 pieds au sol)
@move("balancement", "↔ Balancement", 8)
def sway(t, B, a, P):
    P.body[1] += env(t, B) * a * 0.035 * math.sin(PI * t)


@move("rebond", "⇕ Rebond", 8)
def bounce(t, B, a, P):
    P.body[2] -= env(t, B, 0.5) * a * 0.025 * (0.5 - 0.5 * math.cos(2 * PI * t))


@move("twist", "↺↻ Twist", 8)
def twist(t, B, a, P):
    P.body[5] += env(t, B) * a * 0.32 * math.sin(PI * t)


@move("hula", "◎ Hula-hoop", 8)
def hula(t, B, a, P):
    e, ang = env(t, B, 1.0) * a, 2 * PI * t / 4
    P.body[0] += e * 0.025 * math.cos(ang)
    P.body[1] += e * 0.03 * math.sin(ang)
    P.body[3] += e * 0.10 * math.sin(ang)
    P.body[4] -= e * 0.10 * math.cos(ang)


@move("roulis", "⤺ Roulis", 8)
def rock(t, B, a, P):
    P.body[3] += env(t, B) * a * 0.22 * math.sin(PI * t)


@move("hochement", "⤵ Hochement", 8)
def nod(t, B, a, P):
    P.body[4] += env(t, B) * a * 0.18 * math.sin(PI * t)


@move("pompes", "⇓ Pompes", 8)
def pushups(t, B, a, P):
    e = env(t, B, 0.5) * a
    P.body[2] -= e * 0.045 * (0.5 - 0.5 * math.cos(PI * t))
    P.body[4] += e * 0.06 * (0.5 - 0.5 * math.cos(PI * t))


@move("grandir", "⇑ Grandir", 4)
def rise(t, B, a, P):
    P.body[2] += window(t, 0, B, 1.0) * a * 0.03


@move("revérence", "🙇 Révérence", 4)
def bow(t, B, a, P):
    e = window(t, 0, B, 1.2) * a
    P.body[4] += e * 0.32
    P.body[2] -= e * 0.02
    P.body[0] -= e * 0.01


@move("shimmy", "≋ Shimmy", 8)
def shimmy(t, B, a, P):
    e = env(t, B) * a
    P.body[5] += e * 0.12 * math.sin(4 * PI * t)          # 2 secousses par beat
    P.body[1] += e * 0.02 * math.sin(PI * t / 2)
    P.body[2] -= e * 0.01 * (0.5 - 0.5 * math.cos(2 * PI * t))


@move("huit", "∞ Huit", 8)
def figure_eight(t, B, a, P):
    e = env(t, B, 1.0) * a
    P.body[0] += e * 0.03 * math.sin(2 * PI * t / 4)
    P.body[1] += e * 0.035 * math.sin(4 * PI * t / 4)
    P.body[5] += e * 0.12 * math.sin(2 * PI * t / 4)


@move("vague", "〰 Vague", 8)
def body_wave(t, B, a, P):
    """Ondulation d'avant en arrière, comme un serpent."""
    e = env(t, B, 1.0) * a
    P.body[4] += e * 0.15 * math.sin(PI * t)
    P.body[2] -= e * 0.02 * (0.5 - 0.5 * math.cos(2 * PI * t))
    P.body[0] += e * 0.02 * math.cos(PI * t)


# ============================================================ jeux de pattes
@move("ola", "🌊 Ola", 12, legs=True)
def ola(t, B, a, P):
    """Les pattes se lèvent une par une en faisant le tour du corps."""
    e = env(t, B, 1.0) * a
    order = [L["LF"], L["LM"], L["LR"], L["RR"], L["RM"], L["RF"]]
    slot = (t * 2) % 6                                      # une patte par demi-beat
    i = order[int(slot)]
    P.feet[i, 2] += e * 0.06 * bump(slot - int(slot))
    P.body[5] += e * 0.08 * math.sin(2 * PI * t / 3)       # le corps suit la vague


@move("marche_sur_place", "♪ Marche sur place", 8, legs=True)
def stomp(t, B, a, P):
    e = env(t, B, 0.5) * a
    group = [L["LF"], L["LR"], L["RM"]] if int(t) % 2 == 0 else [L["LM"], L["RF"], L["RR"]]
    lift = 0.045 * bump(t - int(t))
    for i in group:
        P.feet[i, 2] += e * lift
    P.body[2] -= e * 0.008 * bump(t - int(t))


@move("coucou", "👋 Coucou", 8, legs=True)
def hello(t, B, a, P):
    """Le corps se décale, puis la patte avant gauche se lève et salue."""
    shift = window(t, 0, B, 1.0)
    P.body[0] -= shift * 0.035
    P.body[1] -= shift * 0.03
    up = window(t, 1.0, B - 1.0, 0.6) * a
    f = P.feet[L["LF"]]
    f[2] += up * 0.11
    f[0] += up * 0.03
    f[1] += up * 0.045 * math.sin(2 * PI * t)


@move("boxe", "🥊 Boxe", 8, legs=True)
def boxing(t, B, a, P):
    """Les deux pattes avant en l'air, coups de poing alternés."""
    shift = window(t, 0, B, 1.0)
    P.body[0] -= shift * 0.05
    P.body[4] -= shift * 0.08                              # buste relevé
    up = window(t, 1.0, B - 1.0, 0.6) * a
    for name, sign in (("LF", 1), ("RF", -1)):
        f = P.feet[L[name]]
        punch = max(0.0, sign * math.sin(2 * PI * t))
        f[2] += up * (0.09 + 0.02 * punch)
        f[0] += up * 0.05 * punch
    P.body[5] += up * 0.10 * math.sin(2 * PI * t)          # rotation des épaules


@move("tambour", "🥁 Tambour", 8, legs=True)
def drum(t, B, a, P):
    """Les pattes avant tapent le sol à tour de rôle, deux fois par beat."""
    shift = window(t, 0, B, 1.0)
    P.body[0] -= shift * 0.035
    act = window(t, 1.0, B - 0.5, 0.5) * a
    half = (t * 2) % 2
    leg = L["LF"] if half < 1 else L["RF"]
    P.feet[leg, 2] += act * 0.05 * bump(half % 1)
    P.body[3] += act * 0.05 * (1 if half < 1 else -1) * bump(half % 1)


@move("menace", "🕷 Menace", 8, legs=True)
def threat(t, B, a, P):
    """Posture d'intimidation : buste cabré, pattes avant levées et écartées qui tremblent."""
    shift = window(t, 0, B, 1.2)
    P.body[0] -= shift * 0.055
    P.body[2] += shift * 0.015
    P.body[4] -= shift * 0.18
    up = window(t, 1.5, B - 1.0, 0.8) * a
    for name, side in (("LF", 1), ("RF", -1)):
        f = P.feet[L[name]]
        f[2] += up * 0.12
        f[1] += up * side * 0.04
        f[0] += up * (0.02 + 0.006 * math.sin(8 * PI * t))   # tremblement


@move("araignee_folle", "🤪 Pattes folles", 8, legs=True)
def crazy_legs(t, B, a, P):
    """Chaque patte du trépied levé dessine un petit cercle, en alternance."""
    e = env(t, B, 0.75) * a
    group = [L["LF"], L["LR"], L["RM"]] if int(t) % 2 == 0 else [L["LM"], L["RF"], L["RR"]]
    s = t - int(t)
    for i in group:
        P.feet[i, 2] += e * 0.05 * bump(s)
        P.feet[i, 0] += e * 0.025 * math.sin(2 * PI * s)
        P.feet[i, 1] += e * 0.025 * math.cos(2 * PI * s) * (1 if i < 3 else -1) * bump(s)
    P.body[5] += e * 0.1 * math.sin(PI * t)


# ============================================================ déplacements dansés
@move("toupie", "🌀 Toupie", 8)
def spin(t, B, a, P):
    e = env(t, B, 1.0)
    P.cmd[2] += e * min(a, 1.0)
    P.body[2] -= e * a * 0.01 * (0.5 - 0.5 * math.cos(2 * PI * t))


@move("moonwalk", "🌙 Moonwalk", 8)
def moonwalk(t, B, a, P):
    e = env(t, B, 1.0)
    P.cmd[0] -= e * min(a, 1.0)
    P.body[4] -= e * 0.06
    P.body[1] += e * a * 0.015 * math.sin(PI * t)


@move("crabe", "🦀 Crabe", 8)
def crab(t, B, a, P):
    """4 beats à gauche, 4 beats à droite."""
    e = env(t, B, 1.0)
    P.cmd[1] += e * min(a, 1.0) * math.sin(PI * t / 4)
    P.body[3] -= e * 0.06 * math.sin(PI * t / 4)


@move("cercle", "◯ Grand cercle", 8)
def circle(t, B, a, P):
    e = env(t, B, 1.0)
    P.cmd[0] += e * 0.8 * min(a, 1.0)
    P.cmd[2] += e * 0.8 * min(a, 1.0)
    P.body[3] -= e * 0.05


@move("avance_rebond", "⇡ Avance en rebondissant", 8)
def bouncy_walk(t, B, a, P):
    e = env(t, B, 1.0)
    P.cmd[0] += e * min(a, 1.0)
    P.body[2] -= e * a * 0.012 * (0.5 - 0.5 * math.cos(2 * PI * t))
    P.body[5] += e * a * 0.08 * math.sin(PI * t)


@move("pause", "⏸ Pause", 4)
def pause(t, B, a, P):
    pass
