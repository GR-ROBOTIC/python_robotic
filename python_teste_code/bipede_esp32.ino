// ============================================================
//  Bipède 12 servos MG996R — politique apprise en simulation
//  Matériel : ESP32 + PCA9685 (servos) + MPU6050 (IMU), tous deux en I2C
//  Bibliothèque requise : "Adafruit PWM Servo Driver Library"
//
//  Commandes (Moniteur série, 115200 bauds) :
//    l : libre    (servos coupés, robot mou)          <- état au démarrage
//    r : repos    (pose de départ, genoux fléchis)
//    m : marche   (le réseau de neurones pilote)      <- seulement depuis "repos"
//    z : zéro     (tous les angles à 0 : pour calibrer)
//    i : afficher / masquer l'IMU
//    t : refaire l'autotest du réseau
// ============================================================
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "reseau.h"
#include "calibration.h"

Adafruit_PWMServoDriver pca(0x40);
const uint8_t MPU = 0x68;

enum Mode { LIBRE, REPOS, MARCHE, ZERO };
Mode mode = LIBRE;

float g_est[3] = { 0, 0, -1 };     // direction de la gravité vue par le robot
float gyro[3] = { 0, 0, 0 };       // rad/s, repère robot
float gyro_biais[3] = { 0, 0, 0 };
float commande[N_SORTIE];          // angles envoyés (rad, convention simulation)
float action_prec[N_SORTIE] = { 0 };
float phase = 0;
bool afficher_imu = false;

const uint32_t PERIODE_IMU_US = 5000;   // 200 Hz
uint32_t t_imu = 0;
int compteur_imu = 0;
int pas_par_controle = 4;                // 4 x 5 ms = 20 ms = 50 Hz (recalculé au setup)

// ------------------------------------------------------------
//  IMU MPU6050
// ------------------------------------------------------------
void mpu_ecrire(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

void mpu_init() {
  mpu_ecrire(0x6B, 0x00);  // réveil
  mpu_ecrire(0x1A, 0x03);  // filtre passe-bas ~44 Hz
  mpu_ecrire(0x1B, 0x08);  // gyro ±500 °/s
  mpu_ecrire(0x1C, 0x08);  // accéléro ±4 g
}

// Lecture brute, convertie dans le repère du robot
bool mpu_lire(float acc[3], float gyr[3]) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU, 14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) r[i] = (Wire.read() << 8) | Wire.read();
  float a_imu[3] = { r[0] / 8192.0f, r[1] / 8192.0f, r[2] / 8192.0f };           // g
  float g_imu[3] = { r[4] / 65.5f, r[5] / 65.5f, r[6] / 65.5f };                 // °/s
  for (int k = 0; k < 3; k++) {
    acc[k] = IMU_SIGNE[k] * a_imu[IMU_AXE[k]];
    gyr[k] = IMU_SIGNE[k] * g_imu[IMU_AXE[k]] * (float)DEG_TO_RAD;
  }
  return true;
}

void normaliser(float v[3]) {
  float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n > 1e-6f) for (int k = 0; k < 3; k++) v[k] /= n;
}

// Filtre complémentaire : gyroscope (rapide) + accéléromètre (stable)
void imu_mettre_a_jour(float dt) {
  float acc[3], gyr[3];
  if (!mpu_lire(acc, gyr)) return;
  for (int k = 0; k < 3; k++) gyro[k] = gyr[k] - gyro_biais[k];
  // Un vecteur fixe dans le monde tourne à l'inverse du robot : dg/dt = g x w
  float d[3] = { g_est[1] * gyro[2] - g_est[2] * gyro[1],
                 g_est[2] * gyro[0] - g_est[0] * gyro[2],
                 g_est[0] * gyro[1] - g_est[1] * gyro[0] };
  float g_acc[3] = { -acc[0], -acc[1], -acc[2] };   // l'accéléro mesure "vers le haut"
  normaliser(g_acc);
  const float ALPHA = 0.02f;
  for (int k = 0; k < 3; k++)
    g_est[k] = (1 - ALPHA) * (g_est[k] + d[k] * dt) + ALPHA * g_acc[k];
  normaliser(g_est);
}

void imu_calibrer() {
  Serial.println("Calibration du gyroscope : ne pas bouger le robot (2 s)...");
  float somme[3] = { 0 }, acc[3], gyr[3], acc_moy[3] = { 0 };
  int n = 0;
  for (int i = 0; i < 400; i++) {
    if (mpu_lire(acc, gyr)) {
      for (int k = 0; k < 3; k++) { somme[k] += gyr[k]; acc_moy[k] += acc[k]; }
      n++;
    }
    delay(5);
  }
  if (n == 0) { Serial.println("ERREUR : MPU6050 introuvable (vérifiez le câblage I2C)."); return; }
  for (int k = 0; k < 3; k++) { gyro_biais[k] = somme[k] / n; g_est[k] = -acc_moy[k] / n; }
  normaliser(g_est);
  Serial.printf("Gravité mesurée : (%.2f, %.2f, %.2f)  -> doit être proche de (0, 0, -1)\n",
                g_est[0], g_est[1], g_est[2]);
}

float inclinaison() { return acosf(borner(-g_est[2], -1.0f, 1.0f)); }

// ------------------------------------------------------------
//  Servos
// ------------------------------------------------------------
void servo_ecrire(int i, float angle_rad) {
  float deg = 90.0f + SENS[i] * angle_rad * (float)RAD_TO_DEG + DECALAGE_DEG[i];
  deg = borner(deg, 0.0f, 180.0f);
  float us = IMPULSION_MIN_US + (IMPULSION_MAX_US - IMPULSION_MIN_US) * deg / 180.0f;
  pca.writeMicroseconds(CANAL[i], (uint16_t)us);
}

void servos_couper() {
  for (int i = 0; i < N_SORTIE; i++) pca.setPWM(CANAL[i], 0, 4096);
}

// Déplacement progressif vers une pose (vitesse limitée à 1 rad/s)
void aller_vers(const float cible[N_SORTIE], float dt) {
  const float pas_max = 1.0f * dt;
  for (int i = 0; i < N_SORTIE; i++) {
    float e = borner(cible[i] - commande[i], -pas_max, pas_max);
    commande[i] += e;
    servo_ecrire(i, commande[i]);
  }
}

// ------------------------------------------------------------
//  Un pas de contrôle (50 Hz)
// ------------------------------------------------------------
void controle() {
  const float dt = DT_CONTROLE;

  if (mode != LIBRE && inclinaison() > INCLINAISON_MAX) {
    servos_couper();
    mode = LIBRE;
    Serial.println("CHUTE détectée : servos coupés. Relevez le robot puis tapez 'r'.");
    return;
  }

  if (mode == REPOS) {
    aller_vers(POSE_REPOS, dt);
  } else if (mode == ZERO) {
    static const float zeros[N_SORTIE] = { 0 };
    aller_vers(zeros, dt);
  } else if (mode == MARCHE) {
    // Observation : exactement le même ordre que dans env_bipede.py
    float obs[N_ENTREE];
    for (int k = 0; k < 3; k++) { obs[k] = g_est[k]; obs[3 + k] = gyro[k]; }
    for (int i = 0; i < N_SORTIE; i++) obs[6 + i] = action_prec[i];
    obs[18] = sinf(2 * PI * phase);
    obs[19] = cosf(2 * PI * phase);

    float action[N_SORTIE];
    politique_calculer(obs, action);
    action_vers_angles(action, commande);
    for (int i = 0; i < N_SORTIE; i++) {
      servo_ecrire(i, commande[i]);
      action_prec[i] = action[i];
    }
    phase += dt / PERIODE_PAS;
    if (phase >= 1.0f) phase -= 1.0f;
  }

  if (afficher_imu) {
    Serial.printf("g=(%+.2f %+.2f %+.2f)  gyro=(%+.2f %+.2f %+.2f)  incl=%.0f°\n",
                  g_est[0], g_est[1], g_est[2], gyro[0], gyro[1], gyro[2],
                  inclinaison() * RAD_TO_DEG);
  }
}

// ------------------------------------------------------------
//  Commandes série
// ------------------------------------------------------------
void lire_commandes() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'l':
        servos_couper(); mode = LIBRE; Serial.println("Mode LIBRE (servos coupés)"); break;
      case 'r':
        if (mode == LIBRE) for (int i = 0; i < N_SORTIE; i++) commande[i] = POSE_REPOS[i];
        mode = REPOS; Serial.println("Mode REPOS"); break;
      case 'z':
        if (mode == LIBRE) for (int i = 0; i < N_SORTIE; i++) commande[i] = 0;
        mode = ZERO; Serial.println("Mode ZÉRO (calibration : jambes droites, pieds à plat)"); break;
      case 'm':
        if (mode != REPOS) { Serial.println("Passez d'abord en REPOS ('r') et attendez 2 s."); break; }
        for (int i = 0; i < N_SORTIE; i++) action_prec[i] = 0;
        phase = 0;
        mode = MARCHE; Serial.println("Mode MARCHE — 'r' pour arrêter"); break;
      case 'i':
        afficher_imu = !afficher_imu; break;
      case 't':
        Serial.printf("Autotest réseau : écart max = %.2e (OK si < 1e-4)\n", politique_autotest()); break;
    }
  }
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Bipède ESP32 ===");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  pca.begin();
  pca.setOscillatorFrequency(PCA_OSCILLATEUR);
  pca.setPWMFreq(50);
  servos_couper();

  mpu_init();
  imu_calibrer();

  float e = politique_autotest();
  Serial.printf("Autotest réseau : écart max = %.2e  %s\n", e, e < 1e-4f ? "OK" : "PROBLÈME !");
  uint32_t t0 = micros();
  float tmp[N_SORTIE];
  politique_calculer(TEST_OBS, tmp);
  Serial.printf("Temps de calcul du réseau : %lu µs (budget : %d µs)\n",
                (unsigned long)(micros() - t0), (int)(DT_CONTROLE * 1e6f));

  pas_par_controle = (int)roundf(DT_CONTROLE * 1e6f / PERIODE_IMU_US);
  for (int i = 0; i < N_SORTIE; i++) commande[i] = POSE_REPOS[i];
  Serial.println("Prêt. Mode LIBRE. Tapez 'r' (repos) puis 'm' (marche).");
  t_imu = micros();
}

void loop() {
  lire_commandes();
  if ((uint32_t)(micros() - t_imu) >= PERIODE_IMU_US) {
    t_imu += PERIODE_IMU_US;
    imu_mettre_a_jour(PERIODE_IMU_US * 1e-6f);
    if (++compteur_imu >= pas_par_controle) {
      compteur_imu = 0;
      controle();
    }
  }
}
