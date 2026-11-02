/* Parameter calibration on a blocked rotor (theta = 0): ADC -> FOC_Update -> PWM -> RL plant */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "foc_lib.c" /* white-box: the sims reach internal functions and constants */
int main(int argc, char **argv)
{
    /* Plant: the library defaults, or env PRS / PLS (test a motor unlike foc_lib.h's RS / LS) */
    const double Rs = getenv("PRS") ? atof(getenv("PRS")) : RS, Ld = getenv("PLS") ? atof(getenv("PLS")) : LS;
    const double Lq = Ld * atof(argv[1]), vbus = 12.0, Ts = TS; const int SUB = 40; const double h = Ts / SUB;
    const double A_PER_LSB = ADC_2_VOLT * V_2_CURRENT; srand(5);
    static FOC_t foc; memset(&foc, 0, sizeof foc); FOC_Init(&foc, PWM_RES);
    double id = 0, iq = 0; uint32_t pa = PWM_RES / 2, pb = PWM_RES / 2, pc = PWM_RES / 2;
    double rs_v = 0; int modes[3] = {MOTOR_ESTIMATION_RS, MOTOR_ESTIMATION_LD, MOTOR_ESTIMATION_LQ};
    for (int m = 0; m < 3; m++) {
        FOC_Start_Measurement(&foc, modes[m], 0.0f);
        for (int k = 0; k < 100000 && foc.inj_start_flag; k++) {
            /* rotor blocked at theta = 0: alpha = d, beta = q */
            double ia = id, ib = -0.5 * id + 0.8660254037844386 * iq;
            int aa = (int)lround(ADC_OFFSET + ia / A_PER_LSB + (rand() % 5 - 2)), ab = (int)lround(ADC_OFFSET + ib / A_PER_LSB + (rand() % 5 - 2));
            foc.adc_v_a = (uint16_t)aa; foc.adc_v_b = (uint16_t)ab; foc.adc_v_bus = (uint16_t)lround(vbus / ADC_2_VBUS);
            FOC_Update(&foc);
            double va = (double)pa / PWM_RES * vbus, vb = (double)pb / PWM_RES * vbus, vc = (double)pc / PWM_RES * vbus;
            { /* optional inverter dead-time error per phase (env VDT volts, soft knee I0 amps) */
                double vdt = getenv("VDT") ? atof(getenv("VDT")) : 0, i0 = getenv("I0") ? atof(getenv("I0")) : 0.05;
                double pia = id, pib = -0.5 * id + 0.8660254037844386 * iq, pic = -pia - pib;
                va -= vdt * tanh(pia / i0); vb -= vdt * tanh(pib / i0); vc -= vdt * tanh(pic / i0);
            }
            double ud = (2 * va - vb - vc) / 3.0, uq = (vb - vc) / sqrt(3.0);
            for (int j = 0; j < SUB; j++) { id += h * (ud - Rs * id) / Ld; iq += h * (uq - Rs * iq) / Lq; }
            pa = foc.pwm_a; pb = foc.pwm_b; pc = foc.pwm_c;
        }
        if (modes[m] == MOTOR_ESTIMATION_RS) { FOC_BLDC_Parameter_Calculation_Resistance(&foc); rs_v = foc.inj_v; }
        id = iq = 0; /* let the winding settle between injections */
    }
    FOC_BLDC_Parameter_Calculation_Inductance(&foc); FOC_Recalc_Params(&foc);
    fprintf(stderr, "Rs = %.4f ohm (true %.4f), Ld = %.2f uH (true %.2f), Lq = %.2f uH (true %.2f), fault %d\n",
            foc.bldc.Rs, Rs, foc.bldc.Ld * 1e6, Ld * 1e6, foc.bldc.Lq * 1e6, Lq * 1e6, foc.inj_fault, rs_v, foc.hfi_v);
    return 0;
}
