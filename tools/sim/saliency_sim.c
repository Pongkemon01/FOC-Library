/* Saliency sweep on a blocked salient rotor: Ld-type HF injection at SWEEP_POINTS angles,
 * FOC_BLDC_Saliency_Point per angle, then the same harmonic fit as Src/freertos.c.
 * Args: Lq/Ld ratio, rotor d-axis angle (deg el). Env VDT / I0: dead-time error as in calib_sim. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "foc_lib.h"
#define SWEEP_POINTS 24
int main(int argc, char **argv)
{
    const double Rs = RS, Ld = LS, Lq = LS * (argc > 1 ? atof(argv[1]) : 1.0), vbus = 12.0, Ts = TS;
    const double th = (argc > 2 ? atof(argv[2]) : 0.0) * M_PI / 180.0;
    const int SUB = 40; const double h = Ts / SUB;
    const double A_PER_LSB = ADC_2_VOLT * V_2_CURRENT; srand(5);
    const double vdt = getenv("VDT") ? atof(getenv("VDT")) : 0, i0 = getenv("I0") ? atof(getenv("I0")) : 0.05;
    /* alpha-beta inductance matrix at rotor angle th, and its inverse */
    const double L0 = 0.5 * (Ld + Lq), L1 = 0.5 * (Ld - Lq);
    const double Laa = L0 + L1 * cos(2 * th), Lbb = L0 - L1 * cos(2 * th), Lab = L1 * sin(2 * th);
    const double det = Laa * Lbb - Lab * Lab;
    static FOC_t foc; memset(&foc, 0, sizeof foc); FOC_Init(&foc, PWM_RES, ADC_OFFSET);
    double L[SWEEP_POINTS], xr[SWEEP_POINTS];
    for (int k = 0; k < SWEEP_POINTS; k++) {
        double ia = 0, ib = 0; uint32_t pa = PWM_RES / 2, pb = PWM_RES / 2, pc = PWM_RES / 2;
        foc.inj_angle = (float)k * (TWO_PI / SWEEP_POINTS);
        foc.foc_mode = FOC_CALIBRATION; foc.motor_mode = MOTOR_ESTIMATION_LD; foc.inj_n = 0; foc.inj_start_flag = true;
        for (int n = 0; n < 20000 && foc.inj_start_flag; n++) {
            double pib = -0.5 * ia + 0.8660254037844386 * ib; /* phase b current */
            int aa = (int)lround(ADC_OFFSET + ia / A_PER_LSB + (rand() % 5 - 2)), ab = (int)lround(ADC_OFFSET + pib / A_PER_LSB + (rand() % 5 - 2));
            foc.adc_v_a = (uint16_t)aa; foc.adc_v_b = (uint16_t)ab; foc.adc_v_bus = (uint16_t)lround(vbus / ADC_2_VBUS);
            FOC_Update(&foc);
            double va = (double)pa / PWM_RES * vbus, vb = (double)pb / PWM_RES * vbus, vc = (double)pc / PWM_RES * vbus;
            double pic = -ia - pib;
            va -= vdt * tanh(ia / i0); vb -= vdt * tanh(pib / i0); vc -= vdt * tanh(pic / i0);
            double ua = (2 * va - vb - vc) / 3.0, ub = (vb - vc) / sqrt(3.0);
            for (int j = 0; j < SUB; j++) {
                double ea = ua - Rs * ia, eb = ub - Rs * ib;
                ia += h * (Lbb * ea - Lab * eb) / det;
                ib += h * (-Lab * ea + Laa * eb) / det;
            }
            pa = foc.pwm_a; pb = foc.pwm_b; pc = foc.pwm_c;
        }
        float fL, fr, fi;
        if (foc.inj_fault || !FOC_BLDC_Saliency_Point(&foc, &fL, &fr, &fi)) { fprintf(stderr, "point %d failed\n", k); return 1; }
        L[k] = fL; xr[k] = fr;
        printf("%3d deg  L %7.0f nH  cross %+.4f %+.4f\n", k * 360 / SWEEP_POINTS, fL * 1e9, fr, fi);
    }
    double s0 = 0, c1 = 0, s1 = 0, c2 = 0, s2 = 0, xc2 = 0, xs2 = 0, n = SWEEP_POINTS;
    for (int k = 0; k < SWEEP_POINTS; k++) {
        double p = k * 2 * M_PI / n;
        s0 += L[k]; c1 += L[k] * cos(p); s1 += L[k] * sin(p); c2 += L[k] * cos(2 * p); s2 += L[k] * sin(2 * p);
        xc2 += xr[k] * cos(2 * p); xs2 += xr[k] * sin(2 * p);
    }
    double A0 = s0 / n, A1 = 2 * hypot(c1, s1) / n, A2 = 2 * hypot(c2, s2) / n, p2 = 0.5 * atan2(s2, c2);
    double ax = fmod((p2 + 0.5 * M_PI) * 180 / M_PI + 360, 180);
    fprintf(stderr, "true Ld %.2f Lq %.2f uH, d-axis %.1f deg | fit L0 %.2f A2 %.3f uH -> Ld %.2f Lq %.2f, min-L axis %.1f deg, A1 %.3f uH, cross2 %.4f\n",
            Ld * 1e6, Lq * 1e6, fmod(th * 180 / M_PI + 360, 180), A0 * 1e6, A2 * 1e6, (A0 - A2) * 1e6, (A0 + A2) * 1e6, ax, A1 * 1e6, 2 * hypot(xc2, xs2) / n);
    return 0;
}
