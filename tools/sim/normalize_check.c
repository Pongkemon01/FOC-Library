#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "foc_lib.c" /* white-box: the sims reach internal functions and constants */
static double ref(float a){ double t = 2*M_PI, r = fmod((double)a, t); if (r < 0) r += t; return r; }   /* exact w.r.t. TRUE 2pi */
static double aerr(double x, double r){ double e = fabs(x - r); return fmin(e, 2*M_PI - e); }       /* angular distance */
int main(void){ srand(9);
  double R[][2] = {{4*M_PI, 160}, {-1000, -4*M_PI}, {1e3, 1e5}, {-1e7, -1e5}, {1e7, 9.9e7}};
  const char *nm[] = {"[4pi, 160] (calibration)", "[-1000, -4pi]", "[1e3, 1e5]", "[-1e7, -1e5]", "[1e7, 1e8)"};
  for (int r = 0; r < 5; r++) { double emax = 0; int bad = 0;
    for (int i = 0; i < 500000; i++) { float a = (float)(R[r][0] + (R[r][1]-R[r][0]) * (rand()/(double)RAND_MAX)), b = a;
      Normalize_Angle(&b); if (!(b >= 0.0f && b < TWO_PI)) bad++; double e = aerr(b, ref(a)); if (e > emax) emax = e; }
    printf("  %-26s max error vs true 2pi: %.2e rad, out of [0,2pi): %d\n", nm[r], emax, bad); }
  /* exact multiples of TWO_PI and their float neighbours: turn boundaries */
  double eb = 0; int badb = 0;
  for (int k = -20000; k <= 20000; k++) { float m = (float)k * TWO_PI;
    for (int d = -3; d <= 3; d++) { float a = m; for (int j = 0; j < abs(d); j++) a = nextafterf(a, d > 0 ? INFINITY : -INFINITY);
      float b = a; Normalize_Angle(&b); if (!(b >= 0.0f && b < TWO_PI)) badb++; double e = aerr(b, ref(a)); if (e > eb) eb = e; } }
  printf("  %-26s max error vs true 2pi: %.2e rad, out of [0,2pi): %d\n", "turn boundaries +-3 ulp", eb, badb);
  float ed[] = {NAN, INFINITY, -INFINITY, 1e30f, -1e30f, 1e8f, -1e8f}; printf("  edges:");
  for (unsigned i = 0; i < sizeof ed/sizeof *ed; i++) { float b = ed[i]; Normalize_Angle(&b); printf(" %g->%.6g", ed[i], b); } printf("\n");
  return 0; }
