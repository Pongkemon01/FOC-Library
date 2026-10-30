#include <stdio.h>
#include <math.h>
#include "foc_lib.c" /* white-box: the sims reach internal functions and constants */
int main(void){ const double vdc = 12.0; double gmin = 1e9, gmax = 0, emax = 0;
  for (int k = 0; k < 3600; k++) { double ang = k * 2 * M_PI / 3600; for (double mag = 0.5; mag <= 6.9; mag += 0.8) {
    uint32_t a, b, c; SVPWM_Core((float)(mag * cos(ang)), (float)(mag * sin(ang)), (float)vdc, PWM_RES, &a, &b, &c);
    double va = (double)a / PWM_RES * vdc, vb = (double)b / PWM_RES * vdc, vc = (double)c / PWM_RES * vdc;
    double al = (2 * va - vb - vc) / 3, be = (vb - vc) / sqrt(3.0), g = hypot(al, be) / mag, e = fabs(remainder(atan2(be, al) - ang, 2 * M_PI));
    if (g < gmin) gmin = g; if (g > gmax) gmax = g; if (e > emax) emax = e; } }
  printf("applied/commanded voltage magnitude: %.4f .. %.4f, max angle error %.3f deg\n", gmin, gmax, emax * 180 / M_PI); return 0; }
