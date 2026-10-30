/* Host-side test of SMO_Update (eSMO + PLL) against a PMSM model.
 * Build: see tools/sim/README.md */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "foc_lib.c" /* white-box: the sims reach internal functions and constants */
#ifdef _WIN32
#define NULL_DEVICE "NUL"
#else
#define NULL_DEVICE "/dev/null"
#endif

static double wrapd(double x){ x = fmod(x + M_PI, 2*M_PI); if (x < 0) x += 2*M_PI; return x - M_PI; }
static double lsb_noise(void){ return ((rand() % 5) - 2) * 0.0156; }  /* +/-2 ADC LSB (15.6 mA/LSB) */

typedef struct {
    const char *name;
    int   dir;         /* +1 / -1 */
    double meas_alpha; /* measurement LPF alpha (1.0 = none, 0.5 = current foc_lib default) */
    double r_err, l_err; /* observer parameter error factors */
    double Lq_over_Ld;
} Case;

static void run(const Case *c)
{
    const double Ts = TS, Rs = RS, Ld = LS, Lq = LS * c->Lq_over_Ld, psi = FLUX_LINKAGE;
    const int SUB = 40; const double h = Ts / SUB;
    const double IqRef = 3.0, vbus = 16.0;

    BLDC_Specs_t b = { POLE_PAIRS, I_RATED, Rs * c->r_err, Ld * c->l_err, Lq * c->l_err, Ld * c->l_err };
    SMO_t smo = {0};
    SMO_Init(&smo, &b, (float)Ts);

    double th = 1.0, id = 0, iq = 0, ia_f = 0, ib_f = 0;
    double va_prev = 0, vb_prev = 0;
    double sumE[2] = {0}, maxE[2] = {0}, sumW[2] = {0}; int n[2] = {0}, lockTime = -1, drops = 0;
    bool prevLock = false;

    for (int k = 0; k < (int)(1.2 / Ts); k++) {
        double t = k * Ts;
        double w = (t < 0.2) ? 300.0 * t / 0.2 : (t < 0.5) ? 300.0 :
                   (t < 0.8) ? 300.0 + 1500.0 * (t - 0.5) / 0.3 : 1800.0;
        w *= c->dir;

        /* sampled currents (quantised noise + optional foc_lib measurement LPF) */
        double ca = cos(th), sa = sin(th);
        double ia = id * ca - iq * sa + lsb_noise(), ib = id * sa + iq * ca + lsb_noise();
        ia_f = c->meas_alpha * ia + (1 - c->meas_alpha) * ia_f;
        ib_f = c->meas_alpha * ib + (1 - c->meas_alpha) * ib_f;

        /* observer gets the voltage applied during the period that just ended */
        bool ok = SMO_Update(&smo, (float)vbus, (float)va_prev, (float)vb_prev, (float)ia_f, (float)ib_f);
        if (ok && !prevLock && lockTime < 0) lockTime = k;
        if (!ok && prevLock && t > 0.25) drops++;
        prevLock = ok;

        int win = (t > 0.35 && t < 0.5) ? 0 : (t > 1.05) ? 1 : -1;
        if (win >= 0) {
            double es = wrapd(smo.theta_est - th) * 180 / M_PI; double e = fabs(es); 
            sumE[win] += e; if (e > maxE[win]) maxE[win] = e;
            sumW[win] += fabs(smo.omega_est - w); n[win]++;
        }

        /* feed-forward voltage for this period (applied over [k, k+1]) */
        double thm = th + 0.5 * w * Ts;
        double vd = Rs * 0 - w * Lq * (c->dir * IqRef), vq = Rs * c->dir * IqRef + w * psi;
        double va = vd * cos(thm) - vq * sin(thm), vb = vd * sin(thm) + vq * cos(thm);

        for (int j = 0; j < SUB; j++) {   /* plant in dq, voltage fixed in alpha-beta */
            double cc = cos(th), ss = sin(th);
            double ud = va * cc + vb * ss, uq = -va * ss + vb * cc;
            double did = (ud - Rs * id + w * Lq * iq) / Ld;
            double diq = (uq - Rs * iq - w * Ld * id - w * psi) / Lq;
            id += h * did; iq += h * diq; th = wrapd(th + h * w);
        }
        va_prev = va; vb_prev = vb;
    }
    fprintf(stderr, "%-34s lock@%5.1fms drops=%d | 300 rad/s: mean %5.2f max %5.2f deg dw %5.1f | 1800 rad/s: mean %5.2f max %5.2f deg dw %5.1f\n",
            c->name, lockTime < 0 ? -1.0 : lockTime * Ts * 1e3, drops,
            sumE[0] / n[0], maxE[0], sumW[0] / n[0], sumE[1] / n[1], maxE[1], sumW[1] / n[1]);
}

int main(void)
{
    Init_Trigonometric_Tables();
    if (!freopen(NULL_DEVICE, "w", stdout)) { /* keep going: library prints just stay visible */ }   /* silence printf in foc_lib */
    srand(1);
    const Case cases[] = {
        {"eSMO forward",                   +1, 1.0, 1.0, 1.0, 1.0},
        {"eSMO reverse",                   -1, 1.0, 1.0, 1.0, 1.0},
        {"eSMO + meas LPF a=0.5 (default)",+1, 0.5, 1.0, 1.0, 1.0},
        {"eSMO Rs +30%",                   +1, 1.0, 1.3, 1.0, 1.0},
        {"eSMO Ls +30%",                   +1, 1.0, 1.0, 1.3, 1.0},
        {"eSMO IPM Lq=2Ld",                +1, 1.0, 1.0, 1.0, 2.0},
    };
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++) run(&cases[i]);
    return 0;
}
