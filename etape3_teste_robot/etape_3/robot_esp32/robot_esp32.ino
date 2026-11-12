// ============================================================
//  Robot humanoïde 16 servos MG996R — ÉTAPE 3 : marche + danses
//  Matériel : ESP32 (+ shield servos) + MPU6050 (IMU) en I2C
//  Bibliothèque : "ESP32Servo" (Kevin Harrington)
//
//  Fichiers du dossier :
//    calibration.h  vos réglages (broches, sens, centres, IMU)
//    danses.h       servos + chorégraphies (généré, fourni)
//    politique.h    cerveau de marche (généré APRÈS l'entraînement, facultatif)
//    reseau.h       calcul du réseau (ne pas modifier)
//  Sans politique.h, le robot sait déjà danser ; la marche est désactivée.
//
//  Commandes (Moniteur série, 115200 bauds) :
//    l libre (servos coupés)   r repos   m marche   z zéro (calibration)
//    i afficher l'IMU          t autotest du réseau
//    Danses : a Salut  b Victoire  c Révérence  d Flexions
//             e Balancement  f Vague  g Robot  h Disco     x : arrêter la danse
//    Direction pendant la marche (pavé numérique) :
//             7 pas chassé G   8 avancer    9 pas chassé D
//             4 tourner G      5 sur place  6 tourner D
//                              2 reculer
// ============================================================
#include <Wire.h>
#include <ESP32Servo.h>
#include "calibration.h"
#include "danses.h"

#if __has_include("politique.h")
  #define AVEC_MARCHE 1
  #include "reseau.h"
  #if N_ENTREE != 27
    #error "politique.h ne vient pas de l'étape 3 : entraînez puis exportez avec le projet etape_3."
  #endif
#else
  static inline float borner(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
#endif

Servo servos[N_SERVOS];
const uint8_t MPU = 0x68;

enum Mode { LIBRE, REPOS, MARCHE, ZERO, DANSE };
Mode mode = LIBRE;
const char* NOMS_MODES[] = { "LIBRE", "REPOS", "MARCHE", "ZÉRO", "DANSE" };

float commande[N_SERVOS];          // angles envoyés (rad, convention simulation)
float g_est[3] = { 0, 0, -1 };
float gyro[3] = { 0, 0, 0 };
float gyro_biais[3] = { 0, 0, 0 };
bool afficher_imu = false;

// Marche
float action_prec[12] = { 0 };
float phase = 0;
float consigne[3] = { 0, 0, 0 };

// Danse
const Danse* danse = nullptr;      // nullptr + mode DANSE = simple retour au repos
float danse_t = 0;
float danse_depart[N_SERVOS];
float danse_duree = 0;
Mode mode_apres_danse = REPOS;

const uint32_t PERIODE_IMU_US = 5000;   // 200 Hz
uint32_t t_imu = 0;
int compteur_imu = 0;
int pas_par_controle = 4;

// ------------------------------------------------------------
//  IMU MPU6050
// ------------------------------------------------------------
void mpu_ecrire(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

void mpu_init() {
  mpu_ecrire(0x6B, 0x00);  // réveil
  mpu_ecrire(0x1A, 0x03);  // filtre ~44 Hz
  mpu_ecrire(0x1B, 0x08);  // gyro ±500 °/s
  mpu_ecrire(0x1C, 0x08);  // accéléro ±4 g
}

bool mpu_lire(float acc[3], float gyr[3]) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU, 14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) r[i] = (Wire.read() << 8) | Wire.read();
  float a_imu[3] = { r[0] / 8192.0f, r[1] / 8192.0f, r[2] / 8192.0f };
  float g_imu[3] = { r[4] / 65.5f, r[5] / 65.5f, r[6] / 65.5f };
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

void imu_mettre_a_jour(float dt) {
  float acc[3], gyr[3];
  if (!mpu_lire(acc, gyr)) return;
  for (int k = 0; k < 3; k++) gyro[k] = gyr[k] - gyro_biais[k];
  float d[3] = { g_est[1] * gyro[2] - g_est[2] * gyro[1],
                 g_est[2] * gyro[0] - g_est[0] * gyro[2],
                 g_est[0] * gyro[1] - g_est[1] * gyro[0] };
  float g_acc[3] = { -acc[0], -acc[1], -acc[2] };
  normaliser(g_acc);
  const float ALPHA = 0.02f;
  for (int k = 0; k < 3; k++) g_est[k] = (1 - ALPHA) * (g_est[k] + d[k] * dt) + ALPHA * g_acc[k];
  normaliser(g_est);
}

void imu_calibrer() {
  Serial.println("Calibration du gyroscope : ne pas bouger le robot (2 s)...");
  float somme[3] = { 0 }, acc_moy[3] = { 0 }, acc[3], gyr[3];
  int n = 0;
  for (int i = 0; i < 400; i++) {
    if (mpu_lire(acc, gyr)) {
      for (int k = 0; k < 3; k++) { somme[k] += gyr[k]; acc_moy[k] += acc[k]; }
      n++;
    }
    delay(5);
  }
  if (n == 0) { Serial.println("ERREUR : MPU6050 introuvable (câblage I2C ?)."); return; }
  for (int k = 0; k < 3; k++) { gyro_biais[k] = somme[k] / n; g_est[k] = -acc_moy[k] / n; }
  normaliser(g_est);
  Serial.printf("Gravité mesurée : (%.2f, %.2f, %.2f) -> doit être proche de (0, 0, -1)\n",
                g_est[0], g_est[1], g_est[2]);
}

float inclinaison() { return acosf(borner(-g_est[2], -1.0f, 1.0f)); }

// ------------------------------------------------------------
//  Servos
// ------------------------------------------------------------
void servo_ecrire(int i, float angle_rad) {
  float deg = 90.0f + SENS[i] * (angle_rad * (float)RAD_TO_DEG - CENTRE_DEG[i]) + DECALAGE_DEG[i];
  deg = borner(deg, 0.0f, 180.0f);
  int us = IMPULSION_MIN_US + (int)((IMPULSION_MAX_US - IMPULSION_MIN_US) * deg / 180.0f);
  if (!servos[i].attached()) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(BROCHE[i], IMPULSION_MIN_US, IMPULSION_MAX_US);
  }
  servos[i].writeMicroseconds(us);
}

void servos_ecrire_tout() {
  for (int i = 0; i < N_SERVOS; i++) servo_ecrire(i, commande[i]);
}

void servos_couper() {
  for (int i = 0; i < N_SERVOS; i++)
    if (servos[i].attached()) servos[i].detach();
}

void aller_vers(const float cible[N_SERVOS], float dt) {
  const float pas_max = 1.0f * dt;     // 1 rad/s
  for (int i = 0; i < N_SERVOS; i++)
    commande[i] += borner(cible[i] - commande[i], -pas_max, pas_max);
  servos_ecrire_tout();
}

// ------------------------------------------------------------
//  Danses (même calcul que choregraphies.py)
// ------------------------------------------------------------
static inline float courbe_s(float u) { return u * u * (3 - 2 * u); }

void melanger(const float* a, const float* b, float u, float* sortie) {
  float s = courbe_s(borner(u, 0, 1));
  for (int i = 0; i < N_SERVOS; i++)
    sortie[i] = borner(a[i] + (b[i] - a[i]) * s, LIM_BAS[i], LIM_HAUT[i]);
}

// Angles de la danse à l'instant t : entrée -> images clés -> retour au repos
void danse_angles(float t, float* sortie) {
  if (danse == nullptr) {                                  // simple retour au repos
    melanger(danse_depart, POSE_REPOS, t / DUREE_SORTIE, sortie);
    return;
  }
  const float* A = danse->angles;
  const float* T = danse->temps;
  int n = danse->n_images;
  if (t < DUREE_ENTREE) { melanger(danse_depart, A, t / DUREE_ENTREE, sortie); return; }
  float td = t - DUREE_ENTREE;
  for (int k = 0; k < n - 1; k++) {
    if (td <= T[k + 1]) {
      melanger(A + k * N_SERVOS, A + (k + 1) * N_SERVOS, (td - T[k]) / (T[k + 1] - T[k]), sortie);
      return;
    }
  }
  melanger(A + (n - 1) * N_SERVOS, POSE_REPOS, (td - T[n - 1]) / DUREE_SORTIE, sortie);
}

void demarrer_marche() {
#ifdef AVEC_MARCHE
  for (int i = 0; i < 12; i++) action_prec[i] = 0;
  phase = 0;
  mode = MARCHE;
#else
  mode = REPOS;
#endif
}

void lancer_danse(char touche) {
  if (mode != REPOS && mode != MARCHE && mode != DANSE) {
    Serial.println("Passez d'abord en REPOS ('r').");
    return;
  }
  for (int k = 0; k < N_DANSES; k++) {
    if (DANSES[k].touche == touche) {
      if (mode != DANSE) mode_apres_danse = mode;
      danse = &DANSES[k];
      danse_t = 0;
      for (int i = 0; i < N_SERVOS; i++) danse_depart[i] = commande[i];
      danse_duree = DUREE_ENTREE + danse->temps[danse->n_images - 1] + DUREE_SORTIE;
      mode = DANSE;
      Serial.printf("Danse : %s (%.1f s)\n", danse->nom, danse_duree);
      return;
    }
  }
}

void arreter_danse() {
  if (mode != DANSE) return;
  danse = nullptr;
  danse_t = 0;
  for (int i = 0; i < N_SERVOS; i++) danse_depart[i] = commande[i];
  danse_duree = DUREE_SORTIE;
  Serial.println("Arrêt de la danse, retour au repos.");
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
    static const float zeros[N_SERVOS] = { 0 };
    aller_vers(zeros, dt);
  } else if (mode == DANSE) {
    danse_t += dt;
    danse_angles(danse_t, commande);
    servos_ecrire_tout();
    if (danse_t >= danse_duree) {
      Serial.println("Fin de la danse.");
      if (mode_apres_danse == MARCHE) demarrer_marche(); else mode = REPOS;
    }
  }
#ifdef AVEC_MARCHE
  else if (mode == MARCHE) {
    float obs[N_ENTREE];
    for (int k = 0; k < 3; k++) { obs[k] = g_est[k]; obs[3 + k] = gyro[k]; }
    for (int i = 0; i < 12; i++) obs[6 + i] = action_prec[i];
    obs[18] = sinf(2 * PI * phase);
    obs[19] = cosf(2 * PI * phase);
    for (int k = 0; k < 3; k++) obs[20 + k] = consigne[k];
    for (int k = 0; k < 4; k++) obs[23 + k] = 0.0f;      // bras le long du corps pendant la marche

    float action[N_SORTIE];
    politique_calculer(obs, action);
    action_vers_angles(action, commande);                // 12 jambes
    for (int k = 12; k < N_SERVOS; k++) commande[k] = 0;  // 4 bras
    servos_ecrire_tout();
    for (int i = 0; i < 12; i++) action_prec[i] = action[i];
    phase += dt / PERIODE_PAS;
    if (phase >= 1.0f) phase -= 1.0f;
  }
#endif

  if (afficher_imu)
    Serial.printf("g=(%+.2f %+.2f %+.2f) gyro=(%+.2f %+.2f %+.2f) incl=%.0f° mode=%s\n",
                  g_est[0], g_est[1], g_est[2], gyro[0], gyro[1], gyro[2],
                  inclinaison() * RAD_TO_DEG, NOMS_MODES[mode]);
}

// ------------------------------------------------------------
//  Commandes série
// ------------------------------------------------------------
void choisir_consigne(float avant, float lateral, float rotation) {
#ifdef AVEC_MARCHE
  float c[3] = { avant, lateral, rotation };
  for (int k = 0; k < 3; k++) consigne[k] = borner(c[k], CONSIGNE_MIN[k], CONSIGNE_MAX[k]);
  Serial.printf("Consigne : avant %+.2f m/s, latéral %+.2f m/s, rotation %+.2f rad/s\n",
                consigne[0], consigne[1], consigne[2]);
#endif
}

void lire_commandes() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= 'a' && c <= 'h') { lancer_danse(c); continue; }
    switch (c) {
      case 'l': servos_couper(); mode = LIBRE; Serial.println("Mode LIBRE"); break;
      case 'r':
        if (mode == LIBRE) for (int i = 0; i < N_SERVOS; i++) commande[i] = POSE_REPOS[i];
        mode = REPOS; Serial.println("Mode REPOS"); break;
      case 'z':
        if (mode == LIBRE) for (int i = 0; i < N_SERVOS; i++) commande[i] = 0;
        mode = ZERO; Serial.println("Mode ZÉRO (jambes droites, bras le long du corps)"); break;
      case 'm':
#ifdef AVEC_MARCHE
        if (mode != REPOS) { Serial.println("Passez d'abord en REPOS ('r') et attendez 2 s."); break; }
        choisir_consigne(0, 0, 0);
        demarrer_marche();
        Serial.println("Mode MARCHE");
#else
        Serial.println("Pas de cerveau de marche : entraînez puis exportez (politique.h).");
#endif
        break;
      case 'x': case ' ': arreter_danse(); break;
      case '8': choisir_consigne( 0.15f,  0.0f,   0.0f); break;
      case '2': choisir_consigne(-0.08f,  0.0f,   0.0f); break;
      case '4': choisir_consigne( 0.0f,   0.0f,   0.4f); break;
      case '6': choisir_consigne( 0.0f,   0.0f,  -0.4f); break;
      case '7': choisir_consigne( 0.0f,   0.05f,  0.0f); break;
      case '9': choisir_consigne( 0.0f,  -0.05f,  0.0f); break;
      case '5': choisir_consigne( 0.0f,   0.0f,   0.0f); break;
      case 'i': afficher_imu = !afficher_imu; break;
      case 't':
#ifdef AVEC_MARCHE
        Serial.printf("Autotest réseau : écart max = %.2e (OK si < 1e-4)\n", politique_autotest());
#endif
        break;
    }
  }
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Robot 16 servos — étape 3 ===");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servos_couper();

  mpu_init();
  imu_calibrer();

#ifdef AVEC_MARCHE
  float e = politique_autotest();
  Serial.printf("Cerveau de marche : présent, autotest %.2e %s\n", e, e < 1e-4f ? "OK" : "PROBLÈME !");
#else
  Serial.println("Cerveau de marche : absent (danses seulement).");
#endif
  Serial.printf("%d chorégraphies chargées :", N_DANSES);
  for (int k = 0; k < N_DANSES; k++) Serial.printf(" %c=%s", DANSES[k].touche, DANSES[k].nom);
  Serial.println();

  pas_par_controle = (int)roundf(DT_CONTROLE * 1e6f / PERIODE_IMU_US);
  for (int i = 0; i < N_SERVOS; i++) commande[i] = POSE_REPOS[i];
  Serial.println("Prêt. Mode LIBRE. Tapez 'r' (repos), puis une danse (a-h) ou 'm' (marche).");
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
