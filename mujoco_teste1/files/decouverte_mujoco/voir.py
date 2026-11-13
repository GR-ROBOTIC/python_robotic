"""
Ouvre le robot dans le visualiseur interactif de MuJoCo, en pose de repos.

    python voir.py

À essayer dans la fenêtre :
  - panneau de droite, section "Control" : un curseur par servo (comme test_bipede.py, sans code) ;
  - Espace : pause / reprise ; Retour arrière : réinitialiser ;
  - panneau de gauche, "Rendering" : afficher les contacts, les forces, le centre de masse ;
  - double-clic sur une pièce puis Ctrl + clic droit glissé : pousser le robot.
"""
import os

import mujoco
import mujoco.viewer

DOSSIER = os.path.dirname(os.path.abspath(__file__))

m = mujoco.MjModel.from_xml_path(os.path.join(DOSSIER, "bipede.xml"))
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)
print(__doc__)
mujoco.viewer.launch(m, d)
