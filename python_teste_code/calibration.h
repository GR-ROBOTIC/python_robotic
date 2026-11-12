// ============================================================
//  CALIBRATION DU ROBOT RÉEL  —  à adapter à VOTRE montage
// ============================================================
#pragma once
#include <stdint.h>

// ---------- Servos ----------
// Ordre des 12 servos (identique à la simulation, voir politique.h) :
//   0 hanche_lacet_g     1 hanche_roulis_g     2 hanche_tangage_g
//   3 genou_g            4 cheville_tangage_g  5 cheville_roulis_g
//   6 hanche_lacet_d     7 hanche_roulis_d     8 hanche_tangage_d
//   9 genou_d           10 cheville_tangage_d 11 cheville_roulis_d

// Canal de la carte PCA9685 (0 à 15) branché sur chaque servo
static const uint8_t CANAL[12] = { 0, 1, 2, 3, 4, 5,   8, 9, 10, 11, 12, 13 };

// Sens de rotation : +1 ou -1. Si un servo tourne à l'envers de la simulation, inversez-le.
// (Côté droit, les servos sont souvent montés en miroir : plusieurs seront à -1.)
static const int8_t SENS[12] = { 1, 1, 1, 1, 1, 1,   1, 1, 1, 1, 1, 1 };

// Correction en degrés pour que la commande "z" (tous les angles à 0) donne
// des jambes parfaitement droites, pieds à plat. Ajustez servo par servo.
static const float DECALAGE_DEG[12] = { 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0 };

// Largeur d'impulsion du MG996R pour 0° et 180° (à affiner si besoin)
static const float IMPULSION_MIN_US = 500.0f;
static const float IMPULSION_MAX_US = 2500.0f;

// Fréquence de l'oscillateur de la PCA9685 (25-27 MHz selon les cartes).
// Si les angles sont décalés partout, ajustez cette valeur.
static const uint32_t PCA_OSCILLATEUR = 27000000;

// ---------- IMU (MPU6050) ----------
// Repère du robot (comme la simulation) : X vers l'avant, Y vers la gauche, Z vers le haut.
// Pour chaque axe du robot : quel axe de l'IMU (0=X, 1=Y, 2=Z) et quel signe.
// Exemple : IMU montée à plat, flèche X vers l'avant -> {0,1,2} et {1,1,1}.
static const uint8_t IMU_AXE[3]   = { 0, 1, 2 };
static const int8_t  IMU_SIGNE[3] = { 1, 1, 1 };

// ---------- Sécurité ----------
static const float INCLINAISON_MAX = 0.7f;   // rad (~40°) : au-delà, on coupe les servos
static const int   PIN_I2C_SDA = 21;
static const int   PIN_I2C_SCL = 22;
