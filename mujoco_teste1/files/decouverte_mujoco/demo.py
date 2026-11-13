"""
Démonstration MuJoCo : le bipède fait une petite chorégraphie en boucle
(flexions, balancement, tenir sur un pied) et le terminal affiche ses CAPTEURS.

    python demo.py

Dans la fenêtre :  souris = tourner la caméra, molette = zoom,
                   double-clic sur une pièce puis Ctrl + clic droit glissé = la pousser !
                   touche C = points de contact, F = forces de contact, T = transparence
"""
import os
import time

import mujoco
import mujoco.viewer
import numpy as np

from choregraphie import consigne

DOSSIER = os.path.dirname(os.path.abspath(__file__))
CONTROLE = 0.02           # le "cerveau" décide toutes les 20 ms (50 Hz), comme sur l'ESP32
COUPLE_MAX = 1.0          # N·m, MG996R


def main():
    m = mujoco.MjModel.from_xml_path(os.path.join(DOSSIER, "bipede.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)          # pose "repos" définie dans bipede.xml
    sous_pas = round(CONTROLE / m.opt.timestep)
    noms = [m.actuator(i).name.replace("servo_", "") for i in range(m.nu)]
    bassin = m.body("bassin").id
    print(__doc__)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING   # la caméra suit le robot
            viewer.cam.trackbodyid = bassin
            viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 1.2, -15, 135
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

        prochain_affichage = 0.0
        while viewer.is_running():
            debut = time.time()
            angles, etape = consigne(d.time)
            d.ctrl[:] = angles
            for _ in range(sous_pas):
                mujoco.mj_step(m, d)
            viewer.sync()

            # --- Lecture des capteurs, comme le ferait un vrai robot ---
            if d.time >= prochain_affichage:
                prochain_affichage += 0.5
                acc = d.sensor("accelero").data
                inclinaison = np.degrees(np.arccos(np.clip(acc[2] / np.linalg.norm(acc), -1, 1)))
                pied_g = d.sensor("contact_pied_g").data[0]
                pied_d = d.sensor("contact_pied_d").data[0]
                forces = np.abs(d.actuator_force)
                i = int(np.argmax(forces))
                charge = 100 * forces[i] / COUPLE_MAX
                alerte = "  <- SATURÉ" if charge > 95 else ""
                print(f"t={d.time:5.1f}s  {etape:26s} inclinaison {inclinaison:4.1f}°  "
                      f"pieds G {pied_g:5.1f} N / D {pied_d:5.1f} N  "
                      f"servo le plus chargé : {noms[i]} {charge:3.0f} %{alerte}")
                if d.qpos[2] < 0.15:
                    print("Le robot est tombé : remise en position.")
                    mujoco.mj_resetDataKeyframe(m, d, 0)
                    prochain_affichage = 0.0

            reste = CONTROLE - (time.time() - debut)       # temps réel
            if reste > 0:
                time.sleep(reste)


if __name__ == "__main__":
    main()
