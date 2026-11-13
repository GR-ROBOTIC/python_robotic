#!/usr/bin/env bash
# =============================================================
#  Lanceur — Découverte de MuJoCo
#  Première utilisation : chmod +x lancer.sh      Puis : ./lancer.sh

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


# ---------- Actions ----------
installer() {
    echo -e "${GRAS}Installation de MuJoCo...${FIN}"
    pip install mujoco
    python -c "import mujoco; print('MuJoCo', mujoco.__version__, 'est installé.')"
}
voir()    { verifier_fichier voir.py && python voir.py; }
demo()    { verifier_fichier demo.py && python demo.py; }
vitesse() { verifier_fichier vitesse.py && python vitesse.py; }
fichier() { verifier_fichier bipede.xml && python -m mujoco.viewer --mjcf=bipede.xml; }

activer_conda
if ! python -c "import mujoco" >/dev/null 2>&1; then
    echo -e "${JAUNE}MuJoCo n'est pas encore installé : choisissez l'option 1.${FIN}"
fi

case "$1" in
    installer) installer; exit ;;
    voir)      voir; exit ;;
    demo)      demo; exit ;;
    vitesse)   vitesse; exit ;;
esac

while true; do
    echo
    echo -e "${GRAS}====== Découverte de MuJoCo ======${FIN}"
    echo "  1) Installer MuJoCo"
    echo "  2) Voir le robot et le piloter avec les curseurs"
    echo "  3) Démonstration : chorégraphie + capteurs"
    echo "  4) Comparer la vitesse MuJoCo / PyBullet"
    echo "  5) Ouvrir bipede.xml dans le visualiseur seul (sans code)"
    echo "  0) Quitter"
    read -rp "Votre choix : " CHOIX
    case "$CHOIX" in
        1) installer ;; 2) voir ;; 3) demo ;; 4) vitesse ;; 5) fichier ;;
        0) exit 0 ;;
        *) echo -e "${ROUGE}Choix invalide.${FIN}" ;;
    esac
done
