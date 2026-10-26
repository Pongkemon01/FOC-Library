/* Accuracy + edge-case verification of foc_lib math helpers against double libm. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <string.h>
#include "foc_lib.h"

static float rnd_float_any(void){ uint32_t u = ((uint32_t)rand() << 16) ^ (uint32_t)rand(); float f; memcpy(&f, &u, 4); return f; }
static double urand(double lo, double hi){ return lo + (hi - lo) * (rand() / (double)RAND_MAX); }

int main(void)
{
    Init_Trigonometric_Tables(); srand(12345);

    /* ---- Normalize_Angle ---- */
    { double maxerr = 0; int badrange = 0, n = 0;
      for (int i = 0; i < 2000000; i++) {
        float a = (i & 1) ? (float)urand(-1e4, 1e4) : (float)urand(-20, 20);
        float b = a; Normalize_Angle(&b); n++;
        if (!(b >= 0.0f && b <= TWO_PI)) badrange++;
        double ref = fmod((double)a, (double)TWO_PI); if (ref < 0) ref += (double)TWO_PI;   /* exact wrt float TWO_PI */
        double e = fabs(b - ref); if (e > (double)TWO_PI / 2) e = fabs(e - (double)TWO_PI); if (e > maxerr) maxerr = e; }
      float ed[] = {0.0f, -0.0f, TWO_PI, nextafterf(TWO_PI,0), -1e-30f, -TWO_PI, 1e30f, -1e30f, NAN, INFINITY, -INFINITY};
      printf("Normalize_Angle: max err vs exact fmod %.3g rad, out-of-[0,2pi] %d/%d | edges:", maxerr, badrange, n);
      for (unsigned k = 0; k < sizeof ed/sizeof *ed; k++) { float b = ed[k]; Normalize_Angle(&b); printf(" %g->%.7g", ed[k], b); } printf("\n"); }

    /* ---- sin / cos error by range ---- */
    { double R[] = {2*M_PI, 20, 200, 1000};
      printf("sin/cos max abs err by |angle| range:");
      for (int r = 0; r < 4; r++) { double e = 0;
        for (int i = 0; i < 1000000; i++) { float a = (float)urand(-R[r], R[r]); float s, c; Fast_Sin_Cos(a, &s, &c);
          e = fmax(e, fmax(fabs(s - sin((double)a)), fabs(c - cos((double)a))));
          e = fmax(e, fmax(fabs(Fast_Sin(a) - sin((double)a)), fabs(Fast_Cos(a) - cos((double)a)))); }
        printf("  <=%g: %.2e", R[r], e); }
      printf("\n"); }
    /* ---- sin / cos ---- */
    { double es = 0, ec = 0, esc = 0, ew = 0;
      for (int i = 0; i < 4000000; i++) {
        float a = (i % 4 == 0) ? (float)urand(-1000, 1000) : (float)urand(-7, 14);
        float s, c; Fast_Sin_Cos(a, &s, &c);
        double rs = sin((double)a), rc = cos((double)a);
        es = fmax(es, fabs(Fast_Sin(a) - rs)); ec = fmax(ec, fabs(Fast_Cos(a) - rc));
        esc = fmax(esc, fmax(fabs(s - rs), fabs(c - rc)));
        float w = fmodf(fabsf(a), TWO_PI);
        if (a >= 0.0f && a < TWO_PI) { float s2, c2; Fast_Sin_Cos(a, &s2, &c2); ew = fmax(ew, fmax(fabs(s2 - rs), fabs(c2 - rc))); }
        (void)w; }
      printf("sin/cos: max abs err Fast_Sin %.3g, Fast_Cos %.3g, Fast_Sin_Cos %.3g (|a|<=1000)\n", es, ec, esc);
      float ed[] = {0.0f, -0.0f, HALF_PI, PI, TWO_PI, nextafterf(TWO_PI,0), 1e-30f, -1e-30f, 1e30f, NAN, INFINITY};
      printf("  edges (a: sin,cos):"); for (unsigned k = 0; k < sizeof ed/sizeof *ed; k++) { float s, c; Fast_Sin_Cos(ed[k], &s, &c); printf(" %g:(%.3g,%.3g)", ed[k], s, c); } printf("\n"); }

    /* ---- Fast_Atan2 ---- */
    { double e = 0, ew = 0; float wy = 0, wx = 0;
      for (int i = 0; i < 4000000; i++) {
        float y = (float)urand(-10, 10), x = (float)urand(-10, 10);
        if (i % 7 == 0) { y *= 1e-6f; }
        if (i % 11 == 0) { x *= 1e-6f; }
        double d = fabs((double)Fast_Atan2(y, x) - atan2((double)y, (double)x)); if (d > M_PI) d = fabs(d - 2*M_PI);
        if (d > e) { e = d; wy = y; wx = x; } }
      for (int k = 0; k <= 100000; k++) { float a = (float)k / 100000.0f;          /* dense scan of the reduced argument */
        double d = fabs((double)Fast_Atan2(a, 1.0f) - atan((double)a)); if (d > ew) ew = d; }
      printf("Fast_Atan2: max abs err %.3g rad (%.2g deg) at (y=%g, x=%g); reduced-arg scan max %.3g rad\n", e, e*180/M_PI, wy, wx, ew);
      struct { float y, x; } ed[] = {{0,0},{-0.0f,0},{0,-0.0f},{0,-1},{-0.0f,-1},{1,0},{-1,0},{0,1},{1,1},{-1,-1},{INFINITY,1},{1,INFINITY},{INFINITY,INFINITY},{NAN,1},{1e-38f,1e-38f},{1e38f,1e-38f}};
      printf("  edges (y,x -> got | atan2):"); for (unsigned k = 0; k < sizeof ed/sizeof *ed; k++) printf(" (%g,%g->%.5g|%.5g)", ed[k].y, ed[k].x, Fast_Atan2(ed[k].y, ed[k].x), atan2f(ed[k].y, ed[k].x)); printf("\n"); }

    /* ---- Fast_Sqrt ---- */
    { double worst = 0; float wx = 0;
      for (int i = 0; i < 2000000; i++) { float x = (float)pow(10.0, urand(-12, 12));
        double r = fabs(Fast_Sqrt(x) - sqrt((double)x)) / sqrt((double)x); if (r > worst) { worst = r; wx = x; } }
      double band = 0; for (int i = 0; i < 200000; i++) { float x = (float)urand(0.01, 400); double r = fabs(Fast_Sqrt(x) - sqrt((double)x))/sqrt((double)x); if (r > band) band = r; }
      printf("Fast_Sqrt: max rel err %.3g at x=%g over 1e-12..1e12; %.3g over 0.01..400\n", worst, wx, band);
      float ed[] = {0.0f, -0.0f, -1.0f, 1e-38f, 1.0f, 4.0f, 100.0f, 1e4f, 1e38f, INFINITY, NAN};
      printf("  edges:"); for (unsigned k = 0; k < sizeof ed/sizeof *ed; k++) printf(" %g->%.6g", ed[k], Fast_Sqrt(ed[k])); printf("\n");
      int bad = 0; for (int i = 0; i < 1000000; i++) { float x = rnd_float_any(); if (x > 0 && isfinite(x)) { double r = fabs(Fast_Sqrt(x) - sqrt((double)x))/sqrt((double)x); if (r > 1e-6) bad++; } }
      printf("  random positive floats with rel err > 1e-6: %d / ~500000\n", bad); }
    return 0;
}
