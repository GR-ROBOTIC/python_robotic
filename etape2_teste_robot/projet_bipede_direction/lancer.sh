#!/usr/bin/env bash
# =============================================================
#  Lanceur du projet bipède — VERSION DIRECTION (avancer, reculer, tourner)
#
#  Première utilisation :   chmod +x lancer.sh
#  Menu interactif :        ./lancer.sh
#  Commande directe :       ./lancer.sh installer | robot | env | entrainer
#                                       reprendre | courbes | voir | voir-dernier | exporter
# =============================================================

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

tester_robot() { verifier_fichier test_bipede.py && verifier_fichier bipede_12servos.urdf && python test_bipede.py; }

tester_env()   { verifier_fichier env_bipede.py && verifier_fichier bipede_12servos.urdf && python env_bipede.py; }

entrainer() {
    verifier_fichier entrainer.py || return
    COEURS=$(nproc)
    read -rp "Nombre de pas [15000000] : " PAS_TOTAL; PAS_TOTAL=${PAS_TOTAL:-15000000}
    read -rp "Simulations en parallèle [$COEURS] : " NB_ENVS; NB_ENVS=${NB_ENVS:-$COEURS}
    echo -e "${VERT}Entraînement : $PAS_TOTAL pas, $NB_ENVS simulations. Ctrl+C pour arrêter et sauvegarder.${FIN}"
    python entrainer.py --pas "$PAS_TOTAL" --envs "$NB_ENVS"
}

reprendre() {
    if ! derniere_sauvegarde; then
        echo -e "${JAUNE}Aucune sauvegarde intermédiaire trouvée dans modeles/.${FIN}"; return
    fi
    echo -e "Reprise depuis ${GRAS}$MODELE${FIN}"
    read -rp "Pas supplémentaires [3000000] : " PAS_TOTAL; PAS_TOTAL=${PAS_TOTAL:-3000000}
    python entrainer.py --reprendre "$MODELE" --stats "$STATS" --pas "$PAS_TOTAL" --envs "$(nproc)"
}

courbes() {
    echo -e "${VERT}TensorBoard sur http://localhost:6006 (Ctrl+C pour fermer)${FIN}"
    (sleep 3 && xdg-open http://localhost:6006 >/dev/null 2>&1) &
    tensorboard --logdir logs
}

voir() {
    echo "Cliquez dans la fenêtre PyBullet, puis : flèches = avancer/reculer/tourner,"
    echo "Page préc./suiv. = pas chassé, Espace = s'arrêter."
    if [ ! -f modeles/bipede_final.zip ]; then
        echo -e "${JAUNE}Pas encore de modèle final. Essayez « voir la dernière sauvegarde ».${FIN}"; return
    fi
    python voir_robot.py
}

voir_dernier() {
    if ! derniere_sauvegarde; then
        echo -e "${JAUNE}Aucune sauvegarde intermédiaire trouvée.${FIN}"; return
    fi
    echo -e "Modèle : ${GRAS}$MODELE${FIN}"
    echo "Cliquez dans la fenêtre PyBullet, puis : flèches = avancer/reculer/tourner,"
    echo "Page préc./suiv. = pas chassé, Espace = s'arrêter."
    python voir_robot.py --modele "$MODELE" --stats "$STATS"
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
        echo -e "${JAUNE}Aucun modèle trouvé dans modeles/.${FIN}"
    fi
}

# ---------- Programme principal ----------
activer_conda

case "$1" in
    installer)    installer; exit ;;
    robot)        tester_robot; exit ;;
    env)          tester_env; exit ;;
    entrainer)    entrainer; exit ;;
    reprendre)    reprendre; exit ;;
    courbes)      courbes; exit ;;
    voir)         voir; exit ;;
    voir-dernier) voir_dernier; exit ;;
    exporter)     exporter; exit ;;
esac

while true; do
    echo
    echo -e "${GRAS}====== Projet bipède — direction ======${FIN}"
    echo "  1) Installer les bibliothèques d'entraînement"
    echo "  2) Tester le robot (curseurs)"
    echo "  3) Tester l'environnement d'entraînement"
    echo "  4) Lancer un entraînement"
    echo "  5) Reprendre le dernier entraînement"
    echo "  6) Voir les courbes (TensorBoard)"
    echo "  7) Piloter le robot entraîné au clavier (modèle final)"
    echo "  8) Piloter la dernière sauvegarde intermédiaire"
    echo "  9) Exporter le modèle vers l'ESP32"
    echo "  0) Quitter"
    read -rp "Votre choix : " CHOIX
    case "$CHOIX" in
        1) installer ;;
        2) tester_robot ;;
        3) tester_env ;;
        4) entrainer ;;
        5) reprendre ;;
        6) courbes ;;
        7) voir ;;
        8) voir_dernier ;;
        9) exporter ;;
        0) exit 0 ;;
        *) echo -e "${ROUGE}Choix invalide.${FIN}" ;;
    esac
done
