#!/usr/bin/env bash
# =============================================================
#  Lanceur — ÉTAPE 2 avec MuJoCo : apprendre à marcher (avec directions)
#
#  Première utilisation :   chmod +x lancer.sh
#  Menu interactif :        ./lancer.sh
#  Commande directe :       ./lancer.sh installer | robot | env | entrainer | reprendre
#                                       courbes | jouer | jouer-dernier | exporter | comparer
# =============================================================

cd "$(dirname "$(readlink -f "$0")")" || exit 1
VERT="\e[32m"; ROUGE="\e[31m"; JAUNE="\e[33m"; GRAS="\e[1m"; FIN="\e[0m"

# ---------- Python à utiliser : l'environnement déjà activé ----------
if ! command -v python >/dev/null 2>&1; then
    for d in "$HOME/anaconda" "$HOME/anaconda3" "$HOME/miniconda3"; do
        [ -f "$d/bin/activate" ] && source "$d/bin/activate" && break
    done
fi
echo -e "${VERT}Python utilisé : $(command -v python)${FIN}"
if ! python -c "import mujoco" >/dev/null 2>&1; then
    echo -e "${JAUNE}MuJoCo n'est pas installé dans cet environnement : choisissez d'abord l'option 1.${FIN}"
fi

verifier_fichier() {
    [ -f "$1" ] || { echo -e "${ROUGE}Fichier manquant : $1${FIN}"; return 1; }
}

derniere_sauvegarde() {
    MODELE=$(ls modeles/bipede_*_steps.zip 2>/dev/null | sort -V | tail -n 1)
    [ -z "$MODELE" ] && return 1
    PAS=$(basename "$MODELE" | sed -E 's/bipede_([0-9]+)_steps\.zip/\1/')
    STATS="modeles/bipede_vecnormalize_${PAS}_steps.pkl"
    [ -f "$STATS" ]
}

# ---------- Actions ----------
installer() {
    echo -e "${GRAS}Installation de MuJoCo et des bibliothèques d'entraînement...${FIN}"
    pip install mujoco stable-baselines3 gymnasium tensorboard
    python -c "import mujoco, gymnasium, stable_baselines3, torch; print('Tout est installé. MuJoCo', mujoco.__version__)"
}

tester_robot() { verifier_fichier voir.py && python voir.py; }
tester_env()   { verifier_fichier env_bipede.py && python env_bipede.py; }

entrainer() {
    verifier_fichier entrainer.py || return
    read -rp "Nombre de pas [15000000] : " PAS_TOTAL; PAS_TOTAL=${PAS_TOTAL:-15000000}
    read -rp "Simulations en parallèle [$(nproc)] : " NB_ENVS; NB_ENVS=${NB_ENVS:-$(nproc)}
    echo -e "${VERT}Entraînement MuJoCo : $PAS_TOTAL pas, $NB_ENVS simulations. Ctrl+C pour arrêter et sauvegarder.${FIN}"
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

rappel() { echo "Flèches : avancer/reculer/tourner, Page préc./suiv. : pas chassé, Espace : stop, Entrée : relever."; }

jouer() {
    [ -f modeles/bipede_final.zip ] || { echo -e "${JAUNE}Pas encore de modèle final : essayez l'option 7.${FIN}"; return; }
    rappel; python jouer.py
}

jouer_dernier() {
    derniere_sauvegarde || { echo -e "${JAUNE}Aucune sauvegarde intermédiaire.${FIN}"; return; }
    echo -e "Modèle : ${GRAS}$MODELE${FIN}"; rappel
    python jouer.py --modele "$MODELE" --stats "$STATS"
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

comparer() { verifier_fichier comparer_pc.py && python comparer_pc.py; }

# ---------- Programme principal ----------
case "$1" in
    installer)     installer; exit ;;
    robot)         tester_robot; exit ;;
    env)           tester_env; exit ;;
    entrainer)     entrainer; exit ;;
    reprendre)     reprendre; exit ;;
    courbes)       courbes; exit ;;
    jouer)         jouer; exit ;;
    jouer-dernier) jouer_dernier; exit ;;
    exporter)      exporter; exit ;;
    comparer)      comparer; exit ;;
esac

while true; do
    echo
    echo -e "${GRAS}====== Étape 2 — MuJoCo : apprendre à marcher ======${FIN}"
    echo "  1) Installer MuJoCo et les bibliothèques d'entraînement"
    echo "  2) Voir le robot (curseurs)"
    echo "  3) Tester l'environnement d'entraînement"
    echo "  4) Lancer un entraînement"
    echo "  5) Reprendre le dernier entraînement"
    echo "  6) Voir les courbes (TensorBoard)"
    echo "  7) Piloter la dernière sauvegarde intermédiaire"
    echo "  8) Piloter le robot entraîné (modèle final)"
    echo "  9) Exporter le modèle vers l'ESP32"
    echo " 10) Mesurer la vitesse de ce PC"
    echo "  0) Quitter"
    read -rp "Votre choix : " CHOIX
    case "$CHOIX" in
        1) installer ;; 2) tester_robot ;; 3) tester_env ;; 4) entrainer ;; 5) reprendre ;;
        6) courbes ;; 7) jouer_dernier ;; 8) jouer ;; 9) exporter ;; 10) comparer ;;
        0) exit 0 ;;
        *) echo -e "${ROUGE}Choix invalide.${FIN}" ;;
    esac
done
