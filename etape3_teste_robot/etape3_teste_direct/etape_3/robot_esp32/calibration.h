// ============================================================
//  CALIBRATION DU ROBOT RÉEL (16 servos)  —  à adapter à VOTRE montage
// ============================================================
#pragma once
#include <stdint.h>

// ---------- Ordre des servos (identique à la simulation) ----------
//   0 hanche_lacet_g     1 hanche_roulis_g     2 hanche_tangage_g
//   3 genou_g            4 cheville_tangage_g  5 cheville_roulis_g
//   6 hanche_lacet_d     7 hanche_roulis_d     8 hanche_tangage_d
//   9 genou_d           10 cheville_tangage_d 11 cheville_roulis_d
//  12 epaule_tangage_g  13 epaule_roulis_g
//  14 epaule_tangage_d  15 epaule_roulis_d

// ---------- Broches ----------
// L'ESP32 pilote au maximum 16 servos en direct : on les utilise tous.
// -> Remplacez par les broches de VOTRE shield.
// À éviter : 6 à 11 (mémoire flash), 34 à 39 (entrées seulement), 21 et 22 (IMU),
// 1 et 3 (port série USB), 0 et 12 (démarrage). Sur un ESP32-WROVER : pas de 16 ni 17.
static const uint8_t BROCHE[16] = {
  13, 14, 16, 17, 18, 19,     // jambe gauche
  23, 25, 26, 27, 32, 33,     // jambe droite
   4,  5,                     // bras gauche
  15,  2                      // bras droit
};

// ---------- Sens de rotation : +1 ou -1 ----------
// Si un servo tourne à l'envers de la simulation, inversez-le.
static const int8_t SENS[16] = { 1, 1, 1, 1, 1, 1,   1, 1, 1, 1, 1, 1,   1, 1,   1, 1 };

// ---------- Centre de chaque servo (degrés, convention simulation) ----------
// Angle de la simulation qui correspond au milieu du servo (90°).
// Jambes : 0 (le servo couvre -90°..+90°).
// Épaules (tangage) : -45 -> le servo couvre de -135° (bras levé haut devant)
// à +45° (bras vers l'arrière). Montez le palonnier en conséquence.
static const float CENTRE_DEG[16] = { 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   -45, 0,   -45, 0 };

// ---------- Correction fine (degrés) ----------
// Pour que la commande "z" (tous les angles à 0) donne : jambes droites, pieds à plat,
// bras le long du corps. Ajustez servo par servo.
static const float DECALAGE_DEG[16] = { 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0,   0, 0 };

// Largeur d'impulsion du MG996R pour 0° et 180°
static const int IMPULSION_MIN_US = 500;
static const int IMPULSION_MAX_US = 2500;

// ---------- IMU (MPU6050) ----------
// Repère du robot : X vers l'avant, Y vers la gauche, Z vers le haut.
// Pour chaque axe du robot : quel axe de l'IMU (0=X, 1=Y, 2=Z) et quel signe.
static const uint8_t IMU_AXE[3]   = { 0, 1, 2 };
static const int8_t  IMU_SIGNE[3] = { 1, 1, 1 };
static const int PIN_I2C_SDA = 21;
static const int PIN_I2C_SCL = 22;

// ---------- Sécurité ----------
static const float INCLINAISON_MAX = 0.7f;   // rad (~40°) : au-delà, servos coupés
