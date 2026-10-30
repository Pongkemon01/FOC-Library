#include <stdio.h>
#include <math.h>
#include "foc_lib.c" /* white-box: the sims reach internal functions and constants */
/* iq loop on RL plant; back-EMF exceeds the voltage limit for 50 ms (e.g. accelerating
 * past base speed), then drops. Compare original Kb = Kp/Ki with Kb = Ki/Kp. */
static void run(const char *name, float kb)
{
    PI_Controller_t pi; PI_Init(&pi, KP_I, KI_I, TS, 6.93f);
    if (kb >= 0) pi.Kb = kb;
    const double R = RS, L = LS, F = exp(-R*TS/L), G = (1-F)/R;
    double i = 0, peak = 0, integ50 = 0; int settle = -1;
    for (int k = 0; k < 2000; k++) {                 /* 200 ms */
        double t = k*TS, E = (t < 0.05) ? 7.5 : 5.0, iref = 3.0;
        float v = PI_Step(&pi, (float)(iref - i));
        i = F*i + G*(v - E);
        if (k == 499) integ50 = pi.integral;
        if (t > 0.05) { if (i > peak) peak = i; if (settle < 0 && fabs(i-iref) < 0.05 && t > 0.0505) {
            int ok = 1; /* stays settled? checked loosely */ settle = k; (void)ok; } }
    }
    printf("%-26s integral@50ms = %9.1f V | after EMF drop: peak %8.1f A, settled %s%.1f ms\n",
           name, integ50, peak, settle < 0 ? ">" : "", settle < 0 ? 150.0 : (settle*TS - 0.05)*1e3);
}
int main(void){ run("patched PI_Step (library)", -1); return 0; }
