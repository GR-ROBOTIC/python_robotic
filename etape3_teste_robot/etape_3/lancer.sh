#!/usr/bin/env bash
# =============================================================
#  Lanceur — ÉTAPE 3 : robot 16 servos (bras + jambes), marche + danses
#
#  Première utilisation :   chmod +x lancer.sh
#  Menu interactif :        ./lancer.sh
#  Commande directe :       ./lancer.sh installer | robot | danses | env | entrainer | reprendre
#                                       courbes | jouer | jouer-dernier | exporter | exporter-danses | comparer

ENV_CONDA="pybullet"
cd "$(dirname "$(readlink -f "$0")")" || exit 1

VERT="\e[32m"; ROUGE="\e[31m"; JAUNE="\e[33m"; GRAS="\e[1m"; FIN="\e[0m"

# ---------- Activation de l'environnement conda ----------
activer_conda() {
    # Si l'environnement actif contient déjà PyBullet, on l'utilise tel quel
    if python -c "import pybullet" >/dev/null 2>&1; then
        echo -e "${VERT}Environnement utilisé : ${CONDA_DEFAULT_ENV:-python système} ($(command -v python))${FIN}"
        return
    fi
    if ! command -v conda >/dev/null 2>&1; then
        for d in "$HOME/anaconda" "$HOME/anaconda3" "$HOME/miniconda3" "$HOME/miniforge3"; do
            [ -f "$d/etc/profile.d/conda.sh" ] && source "$d/etc/profile.d/conda.sh" && break
        done
    else
        source "$(conda info --base)/etc/profile.d/conda.sh"
    fi
    if ! command -v conda >/dev/null 2>&1; then
        echo -e "${ROUGE}conda introuvable. Vérifiez l'installation d'Anaconda.${FIN}"; exit 1
    fi
    if ! conda activate "$ENV_CONDA" 2>/dev/null; then
        echo -e "${ROUGE}L'environnement conda '$ENV_CONDA' n'existe pas.${FIN}"
        echo "Créez-le avec :"
        echo "  conda create -n $ENV_CONDA python=3.11 -y"
        echo "  conda activate $ENV_CONDA && conda install -c conda-forge pybullet -y"
        exit 1
    fi
}

verifier_fichier() {
    if [ ! -f "$1" ]; then
        echo -e "${ROUGE}Fichier manquant : $1 (il doit être dans $(pwd))${FIN}"; return 1
    fi
}

# Dernière sauvegarde intermédiaire -> variables MODELE et STATS
derniere_sauvegarde() {
    MODELE=$(ls modeles/bipede_*_steps.zip 2>/dev/null | sort -V | tail -n 1)
    if [ -z "$MODELE" ]; then return 1; fi
    PAS=$(basename "$MODELE" | sed -E 's/bipede_([0-9]+)_steps\.zip/\1/')
    STATS="modeles/bipede_vecnormalize_${PAS}_steps.pkl"
    [ -f "$STATS" ]
}

# ---------- Actions ----------
installer() {
    echo -e "${GRAS}Installation des bibliothèques d'entraînement...${FIN}"
    pip install stable-baselines3 gymnasium tensorboard
    python -c "import pybullet, gymnasium, stable_baselines3, torch; print('Tout est installé.')"
}

tester_robot()  { verifier_fichier test_robot.py && python test_robot.py; }
tester_danses() {
    echo "Cliquez dans la fenêtre PyBullet, puis touches a à h pour les danses, Espace pour arrêter."
    verifier_fichier danser.py && python danser.py
}
tester_env()    { verifier_fichier env_bipede.py && python env_bipede.py; }

entrainer() {
    verifier_fichier entrainer.py || return
    read -rp "Nombre de pas [15000000] : " PAS_TOTAL; PAS_TOTAL=${PAS_TOTAL:-15000000}
    read -rp "Simulations en parallèle [$(nproc)] : " NB_ENVS; NB_ENVS=${NB_ENVS:-$(nproc)}
    echo -e "${VERT}Entraînement : $PAS_TOTAL pas, $NB_ENVS simulations. Ctrl+C pour arrêter et sauvegarder.${FIN}"
    python entrainer.py --pas "$PAS_TOTAL" --envs "$NB_ENVS"
}

reprendre() {
    if ! derniere_sauvegarde; then echo -e "${JAUNE}Aucune sauvegarde dans modeles/.${FIN}"; return; fi
    echo -e "Reprise depuis ${GRAS}$MODELE${FIN}"
    read -rp "Pas supplémentaires [5000000] : " PAS_TOTAL; PAS_TOTAL=${PAS_TOTAL:-5000000}
    read -rp "Simulations en parallèle [$(nproc)] : " NB_ENVS; NB_ENVS=${NB_ENVS:-$(nproc)}
    python entrainer.py --reprendre "$MODELE" --stats "$STATS" --pas "$PAS_TOTAL" --envs "$NB_ENVS"
}

courbes() {
    echo -e "${VERT}TensorBoard sur http://localhost:6006 (Ctrl+C pour fermer)${FIN}"
    (sleep 3 && xdg-open http://localhost:6006 >/dev/null 2>&1) &
    tensorboard --logdir logs
}

rappel_touches() {
    echo "Cliquez dans la fenêtre PyBullet. Flèches / Page préc.-suiv. : marcher, Espace : s'arrêter,"
    echo "a à h : danses, Entrée : relever le robot."
}

jouer() {
    if [ ! -f modeles/bipede_final.zip ]; then
        echo -e "${JAUNE}Pas encore de modèle final. Essayez « jouer avec la dernière sauvegarde ».${FIN}"; return
    fi
    rappel_touches; python jouer.py
}

jouer_dernier() {
    if ! derniere_sauvegarde; then echo -e "${JAUNE}Aucune sauvegarde intermédiaire.${FIN}"; return; fi
    echo -e "Modèle : ${GRAS}$MODELE${FIN}"
    rappel_touches; python jouer.py --modele "$MODELE" --stats "$STATS"
}

exporter() {
    verifier_fichier exporter_esp32.py || return
    if [ -f modeles/bipede_final.zip ] && [ -f modeles/vecnormalize_final.pkl ]; then
        read -rp "Exporter le modèle final ? [O/n] : " REP
        if [[ ! "$REP" =~ ^[nN] ]]; then python exporter_esp32.py; return; fi
    fi
    if derniere_sauvegarde; then
        echo -e "Export de ${GRAS}$MODELE${FIN}"
        python exporter_esp32.py --modele "$MODELE" --stats "$STATS"
    else
        python exporter_esp32.py      # exportera seulement les danses
    fi
}

exporter_danses() { python exporter_esp32.py --danses; }
comparer()        { verifier_fichier comparer_pc.py && python comparer_pc.py; }

# ---------- Programme principal ----------
activer_conda

case "$1" in
    installer)       installer; exit ;;
    robot)           tester_robot; exit ;;
    danses)          tester_danses; exit ;;
    env)             tester_env; exit ;;
    entrainer)       entrainer; exit ;;
    reprendre)       reprendre; exit ;;
    courbes)         courbes; exit ;;
    jouer)           jouer; exit ;;
    jouer-dernier)   jouer_dernier; exit ;;
    exporter)        exporter; exit ;;
    exporter-danses) exporter_danses; exit ;;
    comparer)        comparer; exit ;;
esac

while true; do
    echo
    echo -e "${GRAS}====== Étape 3 — bras, marche et danses ======${FIN}"
    echo "   1) Installer les bibliothèques d'entraînement"
    echo "   2) Tester le robot (16 curseurs)"
    echo "   3) Tester les danses (sans entraînement)"
    echo "   4) Tester l'environnement d'entraînement"
    echo "   5) Lancer un entraînement de la marche"
    echo "   6) Reprendre le dernier entraînement"
    echo "   7) Voir les courbes (TensorBoard)"
    echo "   8) Jouer : marcher + danser (modèle final)"
    echo "   9) Jouer avec la dernière sauvegarde"
    echo "  10) Exporter vers l'ESP32 (danses + cerveau)"
    echo "  11) Exporter seulement les danses"
    echo "  12) Mesurer la vitesse de ce PC"
    echo "   0) Quitter"
    read -rp "Votre choix : " CHOIX
    case "$CHOIX" in
        1) installer ;;      2) tester_robot ;;   3) tester_danses ;;
        4) tester_env ;;     5) entrainer ;;      6) reprendre ;;
        7) courbes ;;        8) jouer ;;          9) jouer_dernier ;;
        10) exporter ;;      11) exporter_danses ;; 12) comparer ;;
        0) exit 0 ;;
        *) echo -e "${ROUGE}Choix invalide.${FIN}" ;;
    esac
done
