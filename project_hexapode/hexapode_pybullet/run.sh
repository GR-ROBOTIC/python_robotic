#!/usr/bin/env bash
# Lanceur du projet hexapode PyBullet.
#   ./run.sh              → menu interactif
#   ./run.sh <commande>   → direct (./run.sh help pour la liste)
set -e
cd "$(dirname "$0")"

VENV=".venv"
ENV_CONDA="pybullet"   # nom de ton environnement conda

pick_python() {
  for py in python3.11 python3.10 python3.12 python3; do
    if command -v "$py" >/dev/null 2>&1; then echo "$py"; return; fi
  done
  echo "python3"
}

# Ordre de priorité :
#   1) l'environnement déjà actif s'il contient pybullet (ton conda activé)
#   2) l'environnement conda "pybullet"
#   3) le .venv local créé par ./run.sh install
activate() {
  if python -c "import pybullet" >/dev/null 2>&1; then return; fi
  if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook 2>/dev/null)" || true
    if conda env list 2>/dev/null | awk '{print $1}' | grep -qx "$ENV_CONDA"; then
      conda activate "$ENV_CONDA" && return
    fi
  fi
  if [ -f "$VENV/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$VENV/bin/activate"
  fi
}

check_deps() {
  activate
  if ! python -c "import pybullet, stable_baselines3, gymnasium" 2>/dev/null; then
    echo "⚠  Dépendances manquantes dans $(command -v python). Lance : ./run.sh install"
    exit 1
  fi
  echo "Python utilisé : ${CONDA_DEFAULT_ENV:-$(command -v python)}"
  [ -f urdf/hexapod.urdf ] || python tools/generate_urdf.py
}

do_install() {
  activate
  if python -c "import pybullet" >/dev/null 2>&1; then
    echo "→ pybullet déjà présent dans ${CONDA_DEFAULT_ENV:-$(command -v python)} : j'ajoute le reste."
    python -m pip install "numpy>=1.24" "gymnasium>=0.29" "stable-baselines3>=2.2" "tensorboard>=2.12"
  elif command -v conda >/dev/null 2>&1; then
    echo "→ Création de l'environnement conda '$ENV_CONDA' (pybullet précompilé via conda-forge)"
    eval "$(conda shell.bash hook)"
    conda create -n "$ENV_CONDA" python=3.11 -y
    conda activate "$ENV_CONDA"
    conda install -c conda-forge pybullet -y
    python -m pip install "numpy>=1.24" "gymnasium>=0.29" "stable-baselines3>=2.2" "tensorboard>=2.12"
  else
    PY=$(pick_python)
    echo "→ Pas de conda : création de .venv avec $PY"
    $PY -m venv "$VENV"
    # shellcheck disable=SC1091
    source "$VENV/bin/activate"
    pip install --upgrade pip wheel
    pip install -r requirements.txt
  fi
  python tools/generate_urdf.py
  echo "✓ Installation terminée."
}

do_train() {
  check_deps
  python train.py "$@"
}

do_custom_train() {
  read -rp "Nombre de pas [1000000] : " steps;  steps=${steps:-1000000}
  read -rp "Simulations en parallèle [8] : " envs; envs=${envs:-8}
  read -rp "Nom de l'entraînement (optionnel) : " name
  args=(--steps "$steps" --envs "$envs")
  [ -n "$name" ] && args+=(--name "$name")
  do_train "${args[@]}"
}

do_tensorboard() {
  check_deps
  echo "→ Ouvre http://localhost:6006 dans ton navigateur (Ctrl+C pour arrêter)"
  tensorboard --logdir runs
}

do_clean() {
  read -rp "Supprimer TOUS les entraînements dans runs/ ? (oui/non) : " ok
  if [ "$ok" = "oui" ]; then
    find runs -mindepth 1 ! -name '.gitkeep' -exec rm -rf {} + 2>/dev/null || true
    echo "✓ runs/ vidé."
  else
    echo "Annulé."
  fi
}

do_export() {
  echo "Export vers l'ESP32 : à venir."
  echo "(Idée : exporter la démarche + les corrections du réseau en tableau C pour le firmware.)"
}

usage() {
  cat <<'EOF'
Commandes :
  install              créer .venv et installer les dépendances
  urdf                 régénérer urdf/hexapod.urdf depuis hexapod/config.py
  model [--gait]       voir le modèle (curseurs par servo, ou démarche animée)
  gait                 démarche tripode au sol, sans IA
  quick                entraînement rapide (300 000 pas)
  long                 entraînement long (3 000 000 pas)
  train [options]      entraînement personnalisé (python train.py --help)
  resume [pas]         reprendre le dernier entraînement
  play [options]       voir le dernier entraînement (python play.py --help)
  control [options]    piloter avec des boutons (--ui pybullet sur macOS si besoin)
  tb                   TensorBoard (courbes d'apprentissage)
  list                 lister les entraînements
  clean                supprimer les entraînements
  export               export ESP32 (à venir)
EOF
}

menu() {
  while true; do
    cat <<'EOF'

 ╔══════════════════════════════════════════════╗
 ║        🕷  HEXAPODE  –  PyBullet + PPO         ║
 ╠══════════════════════════════════════════════╣
 ║  1) Installer les dépendances                ║
 ║  2) Voir le modèle (curseurs par servo)      ║
 ║  3) Tester la démarche sans IA               ║
 ║  4) Entraînement rapide   (300k pas)         ║
 ║  5) Entraînement long     (3M pas)           ║
 ║  6) Entraînement personnalisé                ║
 ║  7) Reprendre le dernier entraînement        ║
 ║  8) Voir le dernier entraînement             ║
 ║  9) Piloter l'araignée (boutons)             ║
 ║ 10) TensorBoard (courbes)                    ║
 ║ 11) Lister les entraînements                 ║
 ║ 12) Exporter vers ESP32 (à venir)            ║
 ║ 13) Supprimer les entraînements              ║
 ║  0) Quitter                                  ║
 ╚══════════════════════════════════════════════╝
EOF
    read -rp " Choix : " c
    case "$c" in
      1) do_install ;;
      2) check_deps; python view_model.py ;;
      3) check_deps; python play.py --no-policy ;;
      4) do_train --steps 300000 --name rapide ;;
      5) do_train --steps 3000000 --name long ;;
      6) do_custom_train ;;
      7) read -rp "Pas supplémentaires [1000000] : " s; do_train --resume latest --steps "${s:-1000000}" ;;
      8) check_deps; python play.py ;;
      9) check_deps; python control_gui.py ;;
      10) do_tensorboard ;;
      11) check_deps; python play.py --list ;;
      12) do_export ;;
      13) do_clean ;;
      0|q) exit 0 ;;
      *) echo "Choix inconnu." ;;
    esac
  done
}

cmd="${1:-menu}"; shift || true
case "$cmd" in
  menu) menu ;;
  install) do_install ;;
  urdf) activate; python tools/generate_urdf.py ;;
  model) check_deps; python view_model.py "$@" ;;
  gait) check_deps; python play.py --no-policy "$@" ;;
  quick) do_train --steps 300000 --name rapide "$@" ;;
  long) do_train --steps 3000000 --name long "$@" ;;
  train) do_train "$@" ;;
  resume) do_train --resume latest --steps "${1:-1000000}" ;;
  play) check_deps; python play.py "$@" ;;
  control) check_deps; python control_gui.py "$@" ;;
  tb) do_tensorboard ;;
  list) check_deps; python play.py --list ;;
  clean) do_clean ;;
  export) do_export ;;
  help|-h|--help) usage ;;
  *) echo "Commande inconnue : $cmd"; usage; exit 1 ;;
esac
