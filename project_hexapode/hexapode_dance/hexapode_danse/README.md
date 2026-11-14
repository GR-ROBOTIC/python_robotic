# 🕷 L'araignée danseuse 🪩

Hexapode 18 servos qui danse dans PyBullet, en physique réelle : chaque mouvement est joué par
les servos simulés, avec gravité et frottement. Les pieds restent posés pendant que le corps
ondule, tourne, se baisse ou se cabre, et certaines figures lèvent une ou plusieurs pattes.

## Démarrage

```bash
conda activate pybullet
cd hexapode_danse
chmod +x run.sh
./run.sh
```

Le menu propose : voir le modèle, lister les danses, jouer une danse, spectacle complet,
télécommande, vidéo MP4, créer sa danse, vérifier la stabilité.
Seuls pybullet et numpy sont nécessaires (pas d'entraînement ici). Pour la vidéo, il faut
ffmpeg : `sudo apt install ffmpeg`.

## Les 25 pas de danse

| Corps (6 pieds au sol) | Jeux de pattes | Déplacements dansés |
|---|---|---|
| Balancement, Rebond, Twist, Hula-hoop, Roulis, Hochement, Pompes, Grandir, Révérence, Shimmy, Huit, Vague | Ola (vague de pattes), Marche sur place, Coucou (salut d'une patte), Boxe, Tambour, Menace (cabrée, pattes levées qui tremblent), Pattes folles | Toupie, Moonwalk, Crabe, Grand cercle, Avance en rebondissant |

Tous les mouvements sont calés sur le tempo (BPM). La marche fait un pas par beat, et la piste
disco change de couleur à chaque temps.

## Les 5 chorégraphies

Échauffement (100 BPM), Disco Spider (118), Kung-fu Araignée (126), Moonwalker (112),
Grand Final (122). Elles ont toutes été vérifiées en simulation : aucune chute.

## Télécommande (`./run.sh gui`)

- **Chorégraphies** : lance une danse complète.
- **Pas de danse** : chaque clic ajoute le pas à la file. C'est le mode improvisation.
- **Déplacement** : l'araignée marche *pendant* les pas qui gardent les 6 pieds au sol, par
  exemple un twist en avançant ou un hula-hoop en tournant. Les jeux de pattes arrêtent la marche.
- **Réglages** : tempo, intensité (amplitude des mouvements), piste disco, caméra qui suit.
- **Clavier** : flèches pour marcher, A / E pour tourner, Espace stop, Échap vide la file.

## Créer sa danse

`./run.sh new` crée un fichier dans `dances/` à partir de ce modèle :

```json
{
  "name": "Ma danse",
  "bpm": 110,
  "steps": [
    ["balancement", 8],
    ["rebond", 4, {"amp": 1.2}],
    ["coucou", 8],
    ["revérence", 4]
  ]
}
```

Chaque pas s'écrit `[nom, durée en beats]`, avec une option `{"amp": x}` pour l'amplitude.
Une amplitude négative inverse le sens : par exemple, une toupie à `-1` tourne vers la droite.
Les noms des pas sont donnés par `./run.sh list`.

Avant de la regarder, vérifie qu'elle tient debout avec `./run.sh check ma_danse`. Le test
affiche l'inclinaison maximale du corps et signale une chute éventuelle. Fais-le aussi avant
toute danse sur le vrai robot.

## Inventer un nouveau pas

Dans `hexapod/moves.py`, écris une fonction décorée par `@move("nom", "Libellé", beats)`.
Elle modifie la posture à l'instant `t` (en beats) :

- `P.body` : décalage x, y, z et inclinaison roulis, tangage, lacet du corps ;
- `P.feet` : décalage de chaque pied ;
- `P.cmd` : marche.

Le pas apparaît alors automatiquement dans la télécommande et devient utilisable dans les
chorégraphies. Si ton pas lève des pattes, décale d'abord le corps à l'opposé : le centre
de gravité doit rester au-dessus des pieds encore au sol (voir `coucou` ou `boxe`).

## Pose debout et vrai robot

Pose debout = tous les servos à 90° : cuisses horizontales, tibias verticaux.
Toute la danse sort d'une seule fonction de cinématique (`hexapod/kinematics.py`), facile à
porter sur l'ESP32 avec la même formule de conversion vers les servos que dans le projet de
marche.
