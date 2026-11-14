#!/usr/bin/env bash
# Lanceur de l'araignée danseuse.
#   ./run.sh              → menu
#   ./run.sh <commande>   → direct (./run.sh help)
set -e
cd "$(dirname "$0")"

ENV_CONDA="pybullet"   # nom de ton environnement conda
VENV=".venv"

# 1) environnement déjà actif avec pybullet  2) conda "pybullet"  3) .venv local
activate() {
  if python -c "import pybullet" >/dev/null 2>&1; then return; fi
  if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook 2>/dev/null)" || true
    if conda env list 2>/dev/null | awk '{print $1}' | grep -qx "$ENV_CONDA"; then
      conda activate "$ENV_CONDA" && return
    fi
  fi
  # shellcheck disable=SC1091
  [ -f "$VENV/bin/activate" ] && source "$VENV/bin/activate"
}

check() {
  activate
  if ! python -c "import pybullet, numpy" 2>/dev/null; then
    echo "⚠  pybullet introuvable. Lance : ./run.sh install"
    exit 1
  fi
  [ -f urdf/hexapod.urdf ] || python tools/generate_urdf.py
}

do_install() {
  activate
  if python -c "import pybullet" >/dev/null 2>&1; then
    python -m pip install "numpy>=1.24"
  elif command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
    conda create -n "$ENV_CONDA" python=3.11 -y
    conda activate "$ENV_CONDA"
    conda install -c conda-forge pybullet numpy -y
  else
    python3 -m venv "$VENV"
    # shellcheck disable=SC1091
    source "$VENV/bin/activate"
    pip install --upgrade pip && pip install -r requirements.txt
  fi
  python tools/generate_urdf.py
  echo "✓ Prêt. Pour la vidéo : sudo apt install ffmpeg"
}

pick_dance() {   # la liste et la question vont sur stderr, le choix sur stdout
  python play.py --list >&2
  echo >&2
  read -rp "Nom de la danse (ou 'all') : " d
  echo "${d:-all}"
}

new_dance() {
  read -rp "Nom du fichier (sans espace, ex. ma_danse) : " n
  [ -z "$n" ] && { echo "Annulé."; return; }
  f="dances/$n.json"
  if [ -e "$f" ]; then echo "$f existe déjà."; return; fi
  sed "s/\"Ma danse\"/\"$n\"/" dances/_modele.json > "$f"
  echo "✓ Créé : $f"
  echo "  Modifie-le, puis : ./run.sh check $n   (vérifier qu'elle tient debout)"
  echo "                     ./run.sh play $n    (la regarder)"
  ${EDITOR:-nano} "$f" || true
}

usage() {
  cat <<'EOF'
Commandes :
  install            installer
  model              voir le modèle (curseurs, pose debout = servos à 90°)
  list               lister danses et pas
  play [danse]       jouer une danse (défaut : toutes) ; options : --bpm, --loop, --intensity
  move <pas>         jouer un seul pas
  show               spectacle complet en boucle
  gui                télécommande de danse
  video [danse]      enregistrer une danse en MP4 (videos/)
  new                créer une nouvelle danse
  check [danse]      vérifier qu'une danse (ou toutes) tient debout
  checkmoves         vérifier chaque pas
  urdf               régénérer le modèle
EOF
}

menu() {
  while true; do
    cat <<'EOF'

 ╔═════════════════════════════════════════════╗
 ║       🕷   L'ARAIGNÉE DANSEUSE   🪩           ║
 ╠═════════════════════════════════════════════╣
 ║  1) Installer                               ║
 ║  2) Voir le modèle (curseurs)               ║
 ║  3) Lister les danses et les pas            ║
 ║  4) Jouer une danse                         ║
 ║  5) Spectacle complet (en boucle)           ║
 ║  6) Télécommande de danse 🎛                 ║
 ║  7) Enregistrer une vidéo                   ║
 ║  8) Créer ma propre danse                   ║
 ║  9) Vérifier la stabilité des danses        ║
 ║ 10) Vérifier chaque pas                     ║
 ║  0) Quitter                                 ║
 ╚═════════════════════════════════════════════╝
EOF
    read -rp " Choix : " c
    case "$c" in
      1) do_install ;;
      2) check; python view_model.py ;;
      3) check; python play.py --list ;;
      4) check; d=$(pick_dance); python play.py "$d" ;;
      5) check; python play.py all --loop ;;
      6) check; python dance_gui.py ;;
      7) check; d=$(pick_dance); python play.py "$d" --video ;;
      8) new_dance ;;
      9) check; python check_dances.py || true ;;
      10) check; python check_dances.py --moves || true ;;
      0|q) exit 0 ;;
      *) echo "Choix inconnu." ;;
    esac
  done
}

cmd="${1:-menu}"; shift || true
case "$cmd" in
  menu) menu ;;
  install) do_install ;;
  model) check; python view_model.py ;;
  list) check; python play.py --list ;;
  play) check; python play.py "${@:-all}" ;;
  move) check; python play.py --move "$@" ;;
  show) check; python play.py all --loop ;;
  gui) check; python dance_gui.py ;;
  video) check; python play.py "${1:-all}" --video ;;
  new) new_dance ;;
  check) check; python check_dances.py "$@" ;;
  checkmoves) check; python check_dances.py --moves ;;
  urdf) activate; python tools/generate_urdf.py ;;
  help|-h|--help) usage ;;
  *) echo "Commande inconnue : $cmd"; usage; exit 1 ;;
esac
