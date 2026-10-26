/*
 * hil.c - closed-loop drive simulation:
 *   ADC (quantised, noisy) -> FOC_Update (library as compiled) -> SVPWM duties
 *   -> applied one PWM period later (timer shadow registers) -> PMSM dq model
 *   with inertia, friction and fan load.
 * Usage: ./startup_sim <Tc_Nm> <kfan_Nm_per_(rad/s)^2> <Lq/Ld> <coast_time_s or -1> <t_end_s>
 *                      [dir: 1 | -1] [plant_flux_Wb] [plant_pole_pairs]
 * The optional plant flux / pole pairs let the simulated motor differ from the
 * library's FLUX_LINKAGE / POLE_PAIRS (i.e. test a wrong constant in foc_lib.h).
 * Example (heavy load, forward):  ./startup_sim 0.03 1e-5 1 -1 2.5
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "foc_lib.h"

static double wrap(double x) { x = fmod(x + M_PI, 2 * M_PI); if (x < 0) x += 2 * M_PI; return x - M_PI; }
static double lsb_noise(void) { return (double)((rand() % 5) - 2); }

static const char *mode_name(int m)
{
    switch (m) {
    case MOTOR_IDLE: return "IDLE";
    case MOTOR_STARTUP_ALIGN: return "ALIGN";
    case MOTOR_STARTUP_RAMP: return "RAMP";
#if 1 /* MOTOR_STARTUP_TRANSITION exists in this foc_lib */
    case MOTOR_STARTUP_TRANSITION: return "TRANSITION";
#endif
    case MOTOR_RUNNING: return "RUNNING";
    default: return "?";
    }
}

int main(int argc, char **argv)
{
    const double Tc = atof(argv[1]), kfan = atof(argv[2]), lq_ratio = atof(argv[3]);
    const double t_coast = atof(argv[4]), t_end = atof(argv[5]);
    const double Ts = TS, Rs = RS, Ld = LS, Lq = LS * lq_ratio;
    const double psi = (argc > 7) ? atof(argv[7]) : FLUX_LINKAGE;   /* plant (real motor) flux */
    const double p = (argc > 8) ? atof(argv[8]) : POLE_PAIRS;       /* plant (real motor) pole pairs */
    const double J = 2e-5, B = 1e-6, vbus = 12.0;
    const int SUB = 40; const double h = Ts / SUB;
    const double A_PER_LSB = ADC_2_VOLT * V_2_CURRENT;
    srand(3);

    static FOC_t foc;
    memset(&foc, 0, sizeof foc);
    FOC_Init(&foc, PWM_RES, ADC_OFFSET);
    foc.bldc.Ld = (float)Ld; foc.bldc.Lq = (float)Lq; FOC_Recalc_Params(&foc);
    foc.foc_mode = FOC_TORQUE_CONTROL;
    foc.motor_mode = MOTOR_IDLE;
    foc.is_ref = 3.0f; /* torque command in RUN = startup current: no step */
    if (argc > 6 && atoi(argv[6]) < 0) { foc.rotor_dir = REVERSE_DIRECTION; foc.is_ref = -3.0f; }

    double id = 0, iq = 0, th = 2.0 /* unknown initial rotor angle */, wm = 0;
    uint32_t pa = PWM_RES / 2, pb = PWM_RES / 2, pc = PWM_RES / 2; int bridge_on = 0;
    int last_mode = -1;
    double i_peak = 0, i_peak_after = 0, t_handover = -1, t_run = -1;
    double te_before = 0, wm_at_handover = 0, te_min_after = 1e9, te_max_after = -1e9, wm_min_after = 1e9;
    double cur_ang_prev = 0, max_cur_ang_jump = 0, err_sum = 0, id_sum = 0; int n_err = 0;
    double brake_peak = 0;

    for (long k = 0; k * Ts < t_end; k++) {
        double t = k * Ts;
        /* ---- sample currents (ADC) ---- */
        double c = cos(th), s = sin(th);
        double ial = id * c - iq * s, ibe = id * s + iq * c;
        double ia = ial, ib = -0.5 * ial + 0.8660254037844386 * ibe;
        int aa = (int)lround(ADC_OFFSET + ia / A_PER_LSB + lsb_noise()), ab = (int)lround(ADC_OFFSET + ib / A_PER_LSB + lsb_noise());
        foc.adc_v_a = (uint16_t)(aa < 0 ? 0 : aa > ADC_RES ? ADC_RES : aa);
        foc.adc_v_b = (uint16_t)(ab < 0 ? 0 : ab > ADC_RES ? ADC_RES : ab);
        foc.adc_v_bus = (uint16_t)lround(vbus / ADC_2_VBUS);

        if (t_coast > 0 && t >= t_coast && t - Ts < t_coast) FOC_Coast(&foc);

        FOC_Update(&foc);

        if (foc.motor_mode != last_mode) {
            printf("    t=%7.4f s  -> %-10s  speed %6.1f rad/s el   (observer |E| %.3f V, omega_est %.0f; flux model |forced w|*psi = %.3f V)\n",
                   t, mode_name(foc.motor_mode), wm * p, foc.smo.emf_magnitude, foc.smo.omega_est, fabs(foc.startup_omega) * FLUX_LINKAGE);
            if (last_mode == MOTOR_STARTUP_RAMP && t_handover < 0) { t_handover = t; te_before = 1.5 * p * (psi * iq + (Ld - Lq) * id * iq); wm_at_handover = wm; }
            if (foc.motor_mode == MOTOR_RUNNING) t_run = t;
            last_mode = foc.motor_mode;
        }

        /* ---- plant over [t, t+Ts] with the duties computed in the PREVIOUS period ---- */
        double da = (double)pa / PWM_RES, db = (double)pb / PWM_RES, dc = (double)pc / PWM_RES;
        double va = da * vbus, vb = db * vbus, vc = dc * vbus;
        double v_al = (2 * va - vb - vc) / 3.0, v_be = (vb - vc) / sqrt(3.0);
        for (int j = 0; j < SUB; j++) {
            double cc = cos(th), ss = sin(th), we = p * wm;
            if (bridge_on) {
                double ud = v_al * cc + v_be * ss, uq = -v_al * ss + v_be * cc;
                double did = (ud - Rs * id + we * Lq * iq) / Ld, diq = (uq - Rs * iq - we * Ld * id - we * psi) / Lq;
                id += h * did; iq += h * diq;
            } else {
                id = iq = 0; /* bridge off: no conduction while line back-EMF < Vbus */
            }
            double te = 1.5 * p * (psi * iq + (Ld - Lq) * id * iq);
            double tl = Tc * tanh(wm / 0.5) + kfan * wm * fabs(wm) + B * wm;
            wm += h * (te - tl) / J;
            th = wrap(th + h * p * wm);
        }
        /* duties computed now take effect next period */
        pa = foc.pwm_a; pb = foc.pwm_b; pc = foc.pwm_c;
#if 1 /* MOTOR_STARTUP_TRANSITION exists in this foc_lib */
        bridge_on = foc.outputs_enabled;
#else
        bridge_on = (foc.foc_mode != FOC_IDLE) || 1; /* v1: the modulator always runs */
#endif

        /* ---- metrics ---- */
        double imag = sqrt(id * id + iq * iq), te = 1.5 * p * (psi * iq + (Ld - Lq) * id * iq);
        if (imag > i_peak) i_peak = imag;
        if (t_coast > 0 && t > t_coast && imag > brake_peak) brake_peak = imag;
        double cur_ang = atan2(iq, id);
        if (k > 0 && t_handover > 0 && t < t_handover + 0.3) { double jmp = fabs(wrap(cur_ang - cur_ang_prev)); if (jmp > max_cur_ang_jump) max_cur_ang_jump = jmp; }
        cur_ang_prev = cur_ang;
        if (t_handover > 0 && t < t_handover + 0.3) {
            if (te < te_min_after) te_min_after = te;
            if (te > te_max_after) te_max_after = te;
            if (fabs(wm) < fabs(wm_min_after)) wm_min_after = wm;
            if (imag > i_peak_after) i_peak_after = imag;
        }
        if (t_run > 0 && t > t_run + 0.2 && foc.motor_mode == MOTOR_RUNNING && (t_coast < 0 || t < t_coast)) {
            err_sum += fabs(wrap(foc.smo.theta_est - th)); id_sum += fabs(id); n_err++;
        }
    }
    printf("  peak |i| overall %.1f A", i_peak);
    if (t_handover > 0)
        printf(" | handover at %.3f s: torque before %.3f Nm, min/max in next 0.3 s %.3f/%.3f Nm, speed dip %.1f%%, "
               "max current-vector jump per period %.1f deg, peak |i| %.1f A",
               t_handover, te_before, te_min_after, te_max_after,
               100 * (fabs(wm_at_handover) - fmin(fabs(wm_min_after), fabs(wm_at_handover))) / fmax(fabs(wm_at_handover), 1e-3), max_cur_ang_jump * 180 / M_PI, i_peak_after);
    else printf(" | NO HANDOVER (observer never locked)");
    if (n_err) printf("\n  RUNNING: mean |angle error| %.2f deg, mean |id| %.2f A, final speed %.0f rad/s el, mode %s",
                      err_sum / n_err * 180 / M_PI, id_sum / n_err, wm * p, mode_name(foc.motor_mode));
    if (t_coast > 0) printf("\n  after FOC_Coast at %.2f s (speed %.0f rad/s el): peak winding current %.1f A", t_coast, wm * p, brake_peak);
    printf("\n");
    return 0;
}
