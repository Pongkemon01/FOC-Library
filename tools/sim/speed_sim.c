/*
 * speed_sim.c - startup_sim in SPEED mode: the same drive model (ADC -> FOC_Update ->
 * duties applied one PWM period later -> PMSM dq model + inertia + loads), with the bench
 * test profile: speed_ref 1500 rad/s el, stepped to 2500 at t = 3 s. Prints a trace every
 * 1000 periods and the TRUE plant current ripple during the holds (1500: 2.0-3.0 s,
 * 2500: 4.0-5.5 s).
 * Usage: same arguments as startup_sim:
 *   ./speed_sim <Tc_Nm> <kfan> <Lq/Ld> <coast_time_s or -1> <t_end_s> [dir] [plant_flux_Wb] [plant_pp]
 * Environment:
 *   HD, HP   back-EMF 5th/7th harmonics as -6 / +6 sequence fractions of the fundamental
 *            (exercises FOC_Harmonic_Comp; build with -DHARM_COMP=0 to compare)
 *   LD_EST, RS_EST   controller's (calibrated) Ld / Rs differing from the plant
 *   XD       extra PWM periods of delay in the plant (unmodelled latency)
 *   J        rotor inertia (default 1.4e-5 kg m^2, from the bench acceleration)
 *   HFI      time (s) to set foc.hfi_enable (eSMO <-> HFI switching); prints each switch and,
 *            every 0.1 s while enabled, the TRUE rotor-minus-control angle (mean, max |.|)
 *   PROF     "low": speed_ref 1500, 150 from 2 s, 0 from 5 s, 1500 from 7 s (instead of 1500/2500);
 *            "start": 150 from standstill, 1500 from 3 s (with HSTART=1: HFI start)
 *   HSTART   1: foc.hfi_enable and foc.hfi_start (standstill start on HFI, N/S polarity pulses)
 *   IMAX     current cap (A) as TEST_I_MAX in freertos.c
 *   SAT      d-axis saturation: incremental Ld = Ld * (1 - SAT * id), clamped 0.5..1.5 (per A)
 *            "bench": the freertos.c TEST_PROFILE 2 schedule (150 from 2.5 s, 0 from 8.5 s, 1500 from 10 s)
 *   SHADOW   1: foc.hfi_takeover = false (tracker runs alongside the eSMO, never in control)
 *   SEED     offset (deg) added to the HFI tracker's start angle (sign / pull-in test)
 *   VDT, I0  inverter dead-time error per phase (V) with a soft knee (A), as in calib_sim
 *   VSAG     time (s) at which the bus falls from 12 V to 6 V over 50 ms (supply current limit)
 *   HALF     fraction of each period still driven by the previous command (0 = default
 *            one-period delay; 0.5 = new duty latched mid-period, 1.5-period latency)
 * Example: HD=0.04 HP=0.02 ./speed_sim 0.002 1e-7 1 -1 5.5 1 0.63e-3 7
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "foc_lib.h"

static double HD, HP; /* back-EMF -6 / +6 harmonic fractions (env HD, HP) */
static double rs_n[2], rs_d[2], rs_dd[2], rs_q[2], rs_qq[2];
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
    case MOTOR_STARTUP_HFI: return "HFI_START";
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
    const double J = getenv("J") ? atof(getenv("J")) : 1.4e-5, B = 1e-6; double vbus = 12.0;
    const int SUB = 40; const double h = Ts / SUB;
    const double A_PER_LSB = ADC_2_VOLT * V_2_CURRENT;
    srand(3); HD = getenv("HD") ? atof(getenv("HD")) : 0.0; HP = getenv("HP") ? atof(getenv("HP")) : 0.0;

    static FOC_t foc;
    memset(&foc, 0, sizeof foc);
    FOC_Init(&foc, PWM_RES, ADC_OFFSET);
    foc.bldc.Ld = (float)Ld; foc.bldc.Lq = (float)Lq;
    /* controller parameter mismatch: LD_EST / RS_EST are what calibration "measured" */
    if (getenv("LD_EST")) foc.bldc.Ld = foc.bldc.Lq = (float)atof(getenv("LD_EST"));
    if (getenv("RS_EST")) foc.bldc.Rs = (float)atof(getenv("RS_EST"));
    FOC_Recalc_Params(&foc);
    const int xdelay = getenv("XD") ? atoi(getenv("XD")) : 0; /* extra PWM periods of delay */
    uint32_t qa = PWM_RES / 2, qb = PWM_RES / 2, qc = PWM_RES / 2;
    foc.foc_mode = FOC_SPEED_CONTROL; foc.speed_ref = 1500.0f;
    foc.motor_mode = MOTOR_IDLE;
    foc.is_ref = 3.0f; /* torque command in RUN = startup current: no step */
    if (argc > 6 && atoi(argv[6]) < 0) FOC_Set_Direction(&foc, REVERSE_DIRECTION); /* is_ref and speed_ref are relative to the direction */

    double id = 0, iq = 0, th = 2.0 /* unknown initial rotor angle */, wm = 0;
    uint32_t pa = PWM_RES / 2, pb = PWM_RES / 2, pc = PWM_RES / 2; int bridge_on = 0;
    int last_mode = -1;
    double i_peak = 0, i_peak_after = 0, t_handover = -1, t_run = -1;
    double te_before = 0, wm_at_handover = 0, te_min_after = 1e9, te_max_after = -1e9, wm_min_after = 1e9;
    double cur_ang_prev = 0, max_cur_ang_jump = 0, err_sum = 0, id_sum = 0; int n_err = 0;
    double brake_peak = 0;
    const double t_hfi = getenv("HFI") ? atof(getenv("HFI")) : -1;
    const double t_sag = getenv("VSAG") ? atof(getenv("VSAG")) : -1;
    const double vdt = getenv("VDT") ? atof(getenv("VDT")) : 0, i0 = getenv("I0") ? atof(getenv("I0")) : 0.05;
    const double half = getenv("HALF") ? atof(getenv("HALF")) : 0; /* fraction of the period still on the old command */ uint32_t ha = PWM_RES / 2, hb = PWM_RES / 2, hc = PWM_RES / 2;
    const int prof_low = getenv("PROF") && !strcmp(getenv("PROF"), "low");
    const int prof_bench = getenv("PROF") && !strcmp(getenv("PROF"), "bench");
    if (getenv("SHADOW") && atoi(getenv("SHADOW"))) foc.hfi_takeover = false;
    const int prof_start = getenv("PROF") && !strcmp(getenv("PROF"), "start");
    if (getenv("HSTART") && atoi(getenv("HSTART"))) { foc.hfi_enable = true; foc.hfi_start = true; }
    const double sat = getenv("SAT") ? atof(getenv("SAT")) : 0.0;
if (getenv("IMAX")) { foc.bldc.max_current = (float)atof(getenv("IMAX")); PI_Set_Output_Limit(&foc.speed_ctrl, foc.bldc.max_current); } /* test current cap */
    const double seed = getenv("SEED") ? atof(getenv("SEED")) * M_PI / 180 : 0;
    double hfi_e_sum = 0, hfi_e_max = 0, hfi_q_sum = 0; long hfi_e_n = 0;
    int last_on = 0, last_ctrl = 0;

    for (long k = 0; k * Ts < t_end; k++) {
        double t = k * Ts;
        if (t_sag >= 0) vbus = (t < t_sag) ? 12.0 : fmax(6.0, 12.0 - (t - t_sag) * 120.0); /* VSAG: 12 -> 6 V in 50 ms */
        /* ---- sample currents (ADC) ---- */
        double c = cos(th), s = sin(th);
        double ial = id * c - iq * s, ibe = id * s + iq * c;
        double ia = ial, ib = -0.5 * ial + 0.8660254037844386 * ibe;
        int aa = (int)lround(ADC_OFFSET + ia / A_PER_LSB + lsb_noise()), ab = (int)lround(ADC_OFFSET + ib / A_PER_LSB + lsb_noise());
        foc.adc_v_a = (uint16_t)(aa < 0 ? 0 : aa > ADC_RES ? ADC_RES : aa);
        foc.adc_v_b = (uint16_t)(ab < 0 ? 0 : ab > ADC_RES ? ADC_RES : ab);
        foc.adc_v_bus = (uint16_t)lround(vbus / ADC_2_VBUS);

        if (t_coast > 0 && t >= t_coast && t - Ts < t_coast) FOC_Coast(&foc);

        if (prof_start) foc.speed_ref = (t >= 3.0) ? 1500.0f : 150.0f;
        else if (prof_bench) foc.speed_ref = (t >= 10.0) ? 1500.0f : (t >= 8.5) ? 0.0f : (t >= 2.5) ? 150.0f : 1500.0f;
        else if (prof_low) foc.speed_ref = (t >= 7.0) ? 1500.0f : (t >= 5.0) ? 0.0f : (t >= 2.0) ? 150.0f : 1500.0f;
        else if (t >= 3.0) foc.speed_ref = 2500.0f;
        if (t_hfi >= 0 && t >= t_hfi) foc.hfi_enable = true;
        FOC_Update(&foc);
        if (foc.hfi_on && !last_on && seed != 0) foc.hfi_theta = (float)fmod(foc.hfi_theta + seed + 2 * M_PI, 2 * M_PI);
        if (foc.hfi_on != last_on || foc.hfi_control != last_ctrl)
            printf("    t=%7.4f s  HFI tracker %s, control %s  speed %6.1f rad/s el (est %.0f, eSMO %.0f, HFI %.0f), rotor-control %+.2f deg, HFI-rotor %+.2f deg%s\n",
                   t, foc.hfi_on ? "ON " : "off", foc.hfi_control ? "HFI " : "eSMO", wm * p, foc.omega, foc.smo.omega_est, foc.hfi_omega,
                   wrap(th - foc.theta) * 180 / M_PI, wrap(foc.hfi_theta - th) * 180 / M_PI, foc.hfi_fault ? " FAULT" : "");
        last_on = foc.hfi_on; last_ctrl = foc.hfi_control;
        if (foc.hfi_enable && foc.motor_mode == MOTOR_RUNNING) {
            double e = wrap(th - foc.theta);
            hfi_e_sum += e; hfi_q_sum += iq; hfi_e_n++; if (fabs(e) > hfi_e_max) hfi_e_max = fabs(e);
            if (k % 2000 == 0) {
                printf("      HFI t=%.1f ref %5.0f true %6.1f est %6.1f [%s] rotor-control mean %+6.2f max %5.2f deg, HFI-rotor %+6.2f deg, |Zp| %.2f A, iq %.2f A\n",
                       t, foc.speed_ref_ramp, wm * p, foc.omega, foc.hfi_control ? "HFI " : foc.hfi_on ? "eSMO+t" : "eSMO",
                       hfi_e_sum / hfi_e_n * 180 / M_PI, hfi_e_max * 180 / M_PI, wrap(foc.hfi_theta - th) * 180 / M_PI, foc.hfi_zp, hfi_q_sum / hfi_e_n);
                hfi_e_sum = hfi_q_sum = hfi_e_max = 0; hfi_e_n = 0;
            }
        }
        if (k % 1000 == 0) printf("      t=%.1f mode=%s ref=%.0f est=%.0f true=%.0f iq_ref=%.2f iq=%.2f id=%.2f\n", t, mode_name(foc.motor_mode), foc.speed_ref_ramp, foc.smo.omega_est, wm * p, foc.iq_ref, iq, id);

        if (foc.motor_mode != last_mode) {
            printf("    t=%7.4f s  -> %-10s  speed %6.1f rad/s el   (observer |E| %.3f V, omega_est %.0f; flux model |forced w|*psi = %.3f V)\n",
                   t, mode_name(foc.motor_mode), wm * p, foc.smo.emf_magnitude, foc.smo.omega_est, fabs(foc.startup_omega) * FLUX_LINKAGE);
            if (last_mode == MOTOR_STARTUP_HFI)
                printf("      HFI start: |Zp| +pulse %.4f A, -pulse %.4f A -> pol %d; HFI-rotor %+.2f deg (rotor %.1f deg)\n",
                       foc.hfi_zp_pos, foc.hfi_zp_neg, foc.hfi_pol, wrap(foc.hfi_theta - th) * 180 / M_PI, wrap(th) * 180 / M_PI);
            if (last_mode == MOTOR_STARTUP_RAMP && t_handover < 0) { t_handover = t; te_before = 1.5 * p * (psi * iq + (Ld - Lq) * id * iq); wm_at_handover = wm; }
            if (foc.motor_mode == MOTOR_RUNNING) t_run = t;
            last_mode = foc.motor_mode;
        }

        /* ---- plant over [t, t+Ts] with the duties computed in the PREVIOUS period ---- */
        double da = (double)pa / PWM_RES, db = (double)pb / PWM_RES, dc = (double)pc / PWM_RES;
        double va = da * vbus, vb = db * vbus, vc = dc * vbus;
        /* HALF=f: duties change a fraction f into the period (f = 0.5: 1.5-period latency): the first part of [t, t+Ts] still
           carries the command from two steps back */
        double va0 = (double)ha / PWM_RES * vbus, vb0 = (double)hb / PWM_RES * vbus, vc0 = (double)hc / PWM_RES * vbus;
        if (!half) { va0 = va; vb0 = vb; vc0 = vc; }
        double v_al0 = (2 * va0 - vb0 - vc0) / 3.0, v_be0 = (vb0 - vc0) / sqrt(3.0);
        if (vdt > 0) { /* dead-time error from the phase currents at the period start */
            double cc = cos(th), ss = sin(th), pia = id * cc - iq * ss, pbe = id * ss + iq * cc;
            double pib = -0.5 * pia + 0.8660254037844386 * pbe, pic = -pia - pib;
            va -= vdt * tanh(pia / i0); vb -= vdt * tanh(pib / i0); vc -= vdt * tanh(pic / i0);
        }
        double v_al = (2 * va - vb - vc) / 3.0, v_be = (vb - vc) / sqrt(3.0);
        for (int j = 0; j < SUB; j++) {
            double cc = cos(th), ss = sin(th), we = p * wm;
            if (bridge_on) {
                double xal = (j < SUB * half) ? v_al0 : v_al, xbe = (j < SUB * half) ? v_be0 : v_be;
                double ud = xal * cc + xbe * ss, uq = -xal * ss + xbe * cc;
                double ldi = Ld * fmin(1.5, fmax(0.5, 1.0 - sat * id)); /* SAT: incremental Ld falls with id along the magnet */
                double did = (ud - Rs * id + we * Lq * iq) / ldi, diq = (uq - Rs * iq - we * Ld * id - we * psi) / Lq;
                /* 5th/7th back-EMF harmonics appear as a 6x disturbance in d/q (HD = fraction of fundamental) */
                /* D = we*psi*(HD*exp(-j6th) + HP*exp(+j6th)): -6 and +6 sequence */
                did -= we * psi * (HD + HP) * cos(6 * th) / Ld;
                diq -= we * psi * (HP - HD) * sin(6 * th) / Lq;
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
        ha = pa; hb = pb; hc = pc;
        if (xdelay) { pa = qa; pb = qb; pc = qc; qa = foc.pwm_a; qb = foc.pwm_b; qc = foc.pwm_c; }
        else { pa = foc.pwm_a; pb = foc.pwm_b; pc = foc.pwm_c; }
#if 1 /* MOTOR_STARTUP_TRANSITION exists in this foc_lib */
        bridge_on = foc.outputs_enabled;
#else
        bridge_on = (foc.foc_mode != FOC_IDLE) || 1; /* v1: the modulator always runs */
#endif

        /* ---- metrics ---- */
        { /* true-current ripple during the holds: [0] 1500 (2.0-3.0 s), [1] 2500 (4.0-5.5 s) */
            int b = (t >= 2.0 && t < 3.0) ? 0 : (t >= 4.0 && t < 5.5) ? 1 : -1;
            if (b >= 0) { rs_n[b]++; rs_d[b] += id; rs_dd[b] += id * id; rs_q[b] += iq; rs_qq[b] += iq * iq; }
        }
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
    for (int b = 0; b < 2; b++) if (rs_n[b]) {
        double md = rs_d[b] / rs_n[b], mq = rs_q[b] / rs_n[b];
        printf("\n  hold %d: true id mean %.2f sd %.2f A, iq mean %.2f sd %.2f A", b ? 2500 : 1500,
               md, sqrt(fmax(rs_dd[b] / rs_n[b] - md * md, 0)), mq, sqrt(fmax(rs_qq[b] / rs_n[b] - mq * mq, 0)));
    }
    if (t_coast > 0) printf("\n  after FOC_Coast at %.2f s (speed %.0f rad/s el): peak winding current %.1f A", t_coast, wm * p, brake_peak);
    printf("\n");
    return 0;
}
