// ============================================================
//  Calcul du réseau de neurones de MARCHE (identique au Python)
//  obs brute (27) -> normalisation -> 64 tanh -> 64 tanh -> 12 -> [-1, 1]
//  Ne pas modifier.
// ============================================================
#pragma once
#include <math.h>
#include "danses.h"      // POSE_REPOS, LIM_BAS, LIM_HAUT
#include "politique.h"   // poids du réseau

static inline float borner(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static void couche_dense(const float* W, const float* B, const float* x, float* y,
                         int n_entree, int n_sortie, bool activer_tanh) {
  for (int o = 0; o < n_sortie; o++) {
    const float* ligne = W + o * n_entree;
    float s = B[o];
    for (int i = 0; i < n_entree; i++) s += ligne[i] * x[i];
    y[o] = activer_tanh ? tanhf(s) : s;
  }
}

static void politique_calculer(const float obs[N_ENTREE], float action[N_SORTIE]) {
  float x[N_ENTREE], h1[N_C1], h2[N_C2];
  for (int i = 0; i < N_ENTREE; i++)
    x[i] = borner((obs[i] - OBS_MOYENNE[i]) / OBS_ECART[i], -OBS_CLIP, OBS_CLIP);
  couche_dense(W1, B1, x, h1, N_ENTREE, N_C1, true);
  couche_dense(W2, B2, h1, h2, N_C1, N_C2, true);
  couche_dense(W3, B3, h2, action, N_C2, N_SORTIE, false);
  for (int i = 0; i < N_SORTIE; i++) action[i] = borner(action[i], -1.0f, 1.0f);
}

// Action [-1, 1] -> angles des 12 servos de jambes (radians, convention simulation)
static void action_vers_angles(const float action[N_SORTIE], float angles[N_SORTIE]) {
  for (int i = 0; i < N_SORTIE; i++)
    angles[i] = borner(POSE_REPOS[i] + ECHELLE_ACTION * action[i], LIM_BAS[i], LIM_HAUT[i]);
}

static float politique_autotest() {
  float a[N_SORTIE];
  politique_calculer(TEST_OBS, a);
  float e = 0;
  for (int i = 0; i < N_SORTIE; i++) e = fmaxf(e, fabsf(a[i] - TEST_ACTION[i]));
  return e;
}
