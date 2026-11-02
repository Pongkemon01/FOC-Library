#include <stdio.h>
#include <math.h>
#include "foc_lib.h"
#include <string.h> // memcpy (bit test in Trig_In_Range)

// ============================================================================
// Internal constants (derived from foc_lib.h or fixed by the algorithms; not for the user)
// ============================================================================
#define ONE_OVER_SQRT3 0.57735026919f // 1 / sqrt(3)
#define TWO_OVER_SQRT3 1.15470053838f // 2 / sqrt(3)
#define TWO_PI 6.28318530718f         // 2 * pi
#define PI 3.14159265359f             // pi
#define HALF_PI 1.57079632679f        // pi / 2
#define SQRT3_OVER_2 0.86602540378f   // sqrt(3) / 2

#define CONSTRAIN(val, min, max) \
    ((val) <= (min) ? (min) : ((val) >= (max) ? (max) : (val)))

// Sampling and sensing
#define TS (1.0f / PWM_FREQ)                                        // Sampling period (s)
#ifndef PWM_RES
#define PWM_RES 28799 // Reference PWM period count (TIM1 at 72 MHz, 20 kHz, centre-aligned): host sims
#endif
#define ADC_2_VOLT (ADC_V_REF / (float)ADC_RES)                     // ADC counts to volts
#define ADC_OFFSET ((int)((V_OFFSET / (float)ADC_V_REF) * ADC_RES)) // ADC counts at zero current
#define V_2_CURRENT (CURRENT_SENSE_POLARITY / (V_GAIN * R_SHUNT))   // Sense voltage to current (A/V)
#define ADC_2_VBUS (ADC_2_VOLT / V_BUS_GAIN)                        // ADC counts to bus volts
#define ADC_2_CURRENT (ADC_2_VOLT * V_2_CURRENT)                    // ADC counts to amps
#define BETA_LPF_MEAS (1.0f - ALPHA_LPF_MEAS)                       // Current LPF: 1 - alpha

// Enhanced sliding-mode observer (eSMO) -> see SMO_Update()
#define KP_PLL 630.0f                 // PLL proportional gain
#define KI_PLL 100000.0f              // PLL integral gain
#define MIN_BEMF_AMPLITUDE 0.1f       // Minimum back-EMF amplitude for reliable estimation (V)
// Without HFI (start probe found no saliency, or FOC_HFI 0) the eSMO is the only angle source:
// speed mode stops the motor when speed_ref is below this x the speed where the back-EMF is
// MIN_BEMF_AMPLITUDE (FOC_Min_Speed; bench T200: estimate 50-82 at 75 rad/s el, steady at 150)
#ifndef ESMO_MIN_SPEED_RATIO
#define ESMO_MIN_SPEED_RATIO 1.5f
#endif
#define ESMO_K_SLIDE_MIN 1.0f         // Minimum sliding gain (V)
#define ESMO_K_SLIDE_MARGIN 1.5f      // Sliding gain = margin * |E_hat| (must be > 1)
#define ESMO_BOUNDARY_GAIN 0.0f       // lambda in (0, 1+F). 0 = auto (lambda = F, deadbeat current error)
#define ESMO_WC_MIN (TWO_PI * 30.0f)  // Minimum back-EMF LPF cutoff (rad/s)
#define ESMO_WC_RATIO 2.0f            // LPF cutoff = ratio * |omega| -> constant atan(1/ratio) lag, compensated
#define ESMO_WC_MAX_TS 0.7f           // Ceiling on wc*Ts: keeps the discrete back-EMF loop well damped
#define ESMO_LOCK_ERR 0.10f           // |sin(angle error)| below which the PLL counts as locked (~6 deg)
#define ESMO_UNLOCK_ERR 0.50f         // |sin(angle error)| above which lock is lost (~30 deg)
#define ESMO_PLL_ERR_ALPHA (TS / 0.005f) // Smoothing of the PLL error used for the lock detector (5 ms time constant)
#define ESMO_UNLOCK_COUNT ((uint16_t)(0.005f * PWM_FREQ)) // Consecutive bad samples before reporting loss of lock (5 ms)

// 6x harmonic compensation in closed loop (FOC_Harmonic_Comp). The 5th/7th back-EMF harmonics of
// the 12N14P outrunner appear in d/q as vectors rotating at -6 and +6 x omega (1.4-2.4 kHz at
// 1500-2500 rad/s el), beyond the PI current loop. Each is integrated in its own rotating frame,
// phase-compensated with the plant seen at that frequency (PWM delay, current LPF, RL, PI loop).
#define HARM_V_MAX 0.5f           // Magnitude limit of each harmonic voltage vector (V)
#define HARM_OMEGA_MIN 300.0f     // Active above this |omega| (electrical rad/s)
#define HARM_DELAY_TS 1.5f        // PWM delay seen by the harmonic, in periods (as IPARK_DELAY_COMP)

// Calibration. The Rs run ramps a DC voltage until the current reaches CAL_CURRENT (fault if it
// gets to CAL_V_MAX_RATIO * v_bus first), then measures at half that voltage and at the full one;
// Rs is the slope, which cancels the inverter's voltage error only where that error is constant.
// Before recording, the full level holds while the DC field pulls the unloaded rotor into
// alignment; its swing (back-EMF) biased Rs at 20 kHz when only INJECTION_ROUNDS were allowed, and
// it also stops the ramp early, so the level is scaled back to CAL_CURRENT after the hold.
#define RS_ALIGN_ROUNDS ((uint32_t)(0.3f * PWM_FREQ))
#define CAL_V_MAX_RATIO 0.25f // Ramp ceiling: fraction of the bus voltage
#define CAL_RAMP_TIME 1.0f    // s for the ramp to reach that ceiling
#define CAL_MIN_RATIO 0.2f    // ... where at least this fraction of CAL_CURRENT is still accepted
// Acceptance: broad physical limits only (the motor is unknown), plus a bounded saliency
#define CAL_RS_MIN 0.001f
#define CAL_RS_MAX 50.0f
#define CAL_L_MIN 0.5e-6f
#define CAL_L_MAX 0.1f
#define CAL_LQ_LD_MAX 4.0f // Lq / Ld (and Ld / Lq) above this is a bad measurement
#define MIN_SALIENCY_H 1e-7f                      // Below this Lq-Ld the motor is treated as non-salient (H)
#define INJECTION_FREQ 1000.0f                    // Injection frequency for parameter measurement (Hz)
#define INJECTION_OMEGA (TWO_PI * INJECTION_FREQ) // Injection angular frequency (rad/s)
#define CAL_LQ_HOLD 0.5f // Lq run: DC d-axis current holding the rotor at inj_angle (x CAL_CURRENT)

// Control loops
#define VD_MAX (V_RATED * ONE_OVER_SQRT3)                                  // Maximum D-axis voltage (V)
#define CURRENT_CONTROL_O_BANDWIDTH (TWO_PI * CURRENT_CONTROL_F_BANDWIDTH) // Current control bandwidth (rad/s)
#define KP_I (LS * CURRENT_CONTROL_O_BANDWIDTH)                            // Current controller proportional gain
#define KI_I (RS * CURRENT_CONTROL_O_BANDWIDTH)                            // Current controller integral gain
// Speed loop: runs inside FOC_Update (MOTOR_RUNNING, FOC_SPEED_CONTROL) every SPEED_CTRL_DIVIDER
// periods. Plant: d(omega_el)/dt = SPEED_PLANT_GAIN * iq. Motoring only (no regenerative braking):
// a bench supply cannot absorb the returned energy.
#define SPEED_CTRL_DIVIDER ((uint16_t)(PWM_FREQ / 1000.0f))  // Speed loop every N current-loop periods (1 kHz)
#define TS_SPEED (SPEED_CTRL_DIVIDER * TS)                   // Speed loop sampling period (s)
#define SPEED_CTRL_BANDWIDTH (TWO_PI * SPEED_CTRL_BW_HZ)     // Speed loop crossover (rad/s), well below the PLL
#define KP_SPEED (SPEED_CTRL_BANDWIDTH / SPEED_PLANT_GAIN)   // Speed controller proportional gain (A per rad/s el)
#define KI_SPEED (KP_SPEED * SPEED_CTRL_BANDWIDTH * 0.25f)   // Speed controller integral gain (PI zero at bandwidth/4)
#define V_BUS_UV_PERIODS 10u                                 // Consecutive low samples before the under-voltage trip

// Pass as output_limit to disable output clamping
#define PI_NO_OUTPUT_LIMIT (-1.0f)

// Flux-linkage estimate (Flux_Estimate) while the eSMO is in control
#define FLUX_EST_OMEGA_MIN (2.0f * OMEGA_FORCE) // Only above this |speed| (rad/s el): enough back-EMF
#define FLUX_EST_ALPHA (TS / 0.5f)              // Low-pass, 0.5 s time constant
#define FLUX_EST_RANGE 10.0f                    // Accepted psi: FLUX_LINKAGE / range .. * range
// The eSMO passes |E| with a gain g(wTs) = 0.8944 + 0.357 wTs + 0.73 (wTs)^2 (w = |omega|, wTs =
// angle per period): 1/sqrt(1 + 1/ESMO_WC_RATIO^2) at low speed (its back-EMF LPF at cutoff
// ESMO_WC_RATIO * |omega|), rising with wTs. Fitted to the eSMO alone on an ideal motor (sim,
// 2026-10-30, within 0.05 % for wTs 0.015..0.2); valid for ESMO_WC_RATIO 2.
#define FLUX_EST_G0 0.8944f
#define FLUX_EST_G1 0.357f
#define FLUX_EST_G2 0.73f

// Internal functions (defined below)
static void PI_Init(PI_Controller_t *controller, float Kp, float Ki, float sampling_time, float output_limit);
static void PI_Reset(PI_Controller_t *controller);
static void PI_Change_Params(PI_Controller_t *controller, float Kp, float Ki, float sampling_time);
static void PI_Set_Output_Limit(PI_Controller_t *controller, float output_limit);
static void FOC_BLDC_Parameter_Measurement_Update(FOC_t *foc);
static void Init_Trigonometric_Tables(void);
static void Normalize_Angle(float *angle);
static float Fast_Sin(float angle);
static float Fast_Cos(float angle);
static void Fast_Sin_Cos(float angle, float *sin_out, float *cos_out);
static float Fast_Atan2(float y, float x);
static float Fast_Sqrt(float x);
static void Clarke_Transform(float i_a, float i_b, float *i_alpha, float *i_beta);
static void Park_Transform(float i_alpha, float i_beta, float theta, float *i_d, float *i_q);
static void Clarke_Park_Transform(float i_a, float i_b, float theta, float *i_d, float *i_q);
static void Calculate_Phase_Currents(uint16_t adc_u, uint16_t adc_v, uint16_t offset_u, uint16_t offset_v, float *i_a, float *i_b);
static void SMO_Init(SMO_t *smo, BLDC_Specs_t *bldc, float sampling_time);
static void SMO_Reset(SMO_t *smo);
static void SMO_BLDC_Specs_Update(SMO_t *smo, BLDC_Specs_t *bldc, float sampling_time);
static bool SMO_Update(SMO_t *smo, float v_bus, float v_alpha, float v_beta, float i_alpha_meas, float i_beta_meas);
static void FOC_MTPA(FOC_t *foc, float Is, float *Id_ref, float *Iq_ref);
static void FOC_BLDC_Startup_Update(FOC_t *foc);

// ---------------------------------------------------------------------------
// HF-injection (HFI) tracker for standstill and low speed, and the eSMO <-> HFI switch
// (FOC_HFI). A +-hfi_v square wave (sized from Ls: HFI_STEP_CURRENT) at PWM_FREQ/4 (sign held 2 periods: + + - -) is
// injected along the tracker's own d-axis estimate hfi_theta.
// Not PWM_FREQ/2: TIM1 takes a new duty mid-period, so a sign flipped every period cancels
// within each sampling interval (bench 2026-10-27: HF step 0.13 A instead of ~1.2 A).
// Each period the current step (unfiltered currents) is projected on the axis applied HFI_DELAY
// periods earlier; sliding PWM_FREQ/4 phasors of the parallel (Zp) and perpendicular (Zq) parts
// over the last 4 periods give eps = Re(Zq conj(Zp)) / |Zp|^2 = HFI_K * sin(2 e), e = rotor d-axis
// (saliency minimum) minus hfi_theta, independent of where in the period the duty changes.
// A PLL drives e to 0.
// Switching (foc->omega is the active estimator's speed):
//   eSMO -> HFI: tracker on below HFI_OMEGA_ON (started at the eSMO angle, which fixes the N/S
//                ambiguity), in control below HFI_OMEGA_LOW once warmed up (HFI_WARMUP) and within
//                HFI_AGREE_ANGLE of the eSMO (else the tracker is re-seeded).
//   HFI -> eSMO: above HFI_OMEGA_UP, after the eSMO is locked and agrees with the tracker in
//                angle and speed for HFI_AGREE_PERIODS.
//   Both switches blend the control angle at ANGLE_BLEND_RATE (no current-vector step).
#define HFI_OMEGA_UP (1.3f * HFI_OMEGA_LOW) // HFI -> eSMO above this
#define HFI_OMEGA_ON (1.6f * HFI_OMEGA_LOW) // Tracker (injection) on below this while the eSMO is in control
#define HFI_PLL_BW (TWO_PI * 40.0f)                // Tracker PLL natural frequency (rad/s), critically damped
#define HFI_PLL_KP (2.0f * HFI_PLL_BW)             // rad/s per rad of angle error
#define HFI_PLL_KI (HFI_PLL_BW * HFI_PLL_BW)       // rad/s^2 per rad
#define HFI_WARMUP ((uint32_t)(0.02f * PWM_FREQ))  // Tracker running before it may take control (20 ms)
#define HFI_AGREE_ANGLE 0.2f                       // HFI -> eSMO: max angle difference (rad, ~11 deg)
#define HFI_AGREE_PERIODS ((uint32_t)(0.01f * PWM_FREQ)) // ... held this long (10 ms)
#define HFI_ZP_MIN 0.3f                            // Fault: HF response |Zp| below this (A; bench ~2.3, sim ~0.9;
                                                   // the step is held at HFI_STEP_CURRENT, so it does not scale with L)
#define HFI_V_MAX_RATIO 0.05f                      // Ceiling on hfi_v: fraction of V_RATED
#define HFI_ERR_MAX 0.35f                          // Fault: |e| above this (rad) ...
#define HFI_FAULT_PERIODS ((uint32_t)(0.02f * PWM_FREQ)) // ... or |Zp| low, for this long (20 ms) -> coast
// A tracker locked on the wrong pole (theta + pi) shows a normal |Zp| and e ~ 0, but drives the
// rotor backwards: in HFI control, rotation against rotor_dir faster than HFI_REVERSE_OMEGA for
// HFI_REVERSE_PERIODS latches hfi_fault and coasts (the drive is motoring only).
#define HFI_REVERSE_OMEGA 50.0f                    // Electrical rad/s
#define HFI_REVERSE_PERIODS ((uint32_t)(0.05f * PWM_FREQ)) // 50 ms
// A tracker that lost the rotor while it spins up reads ~0 speed while the current loop drives the
// voltage to the limit against the real back-EMF (T200 bench: runaway near no-load speed). In HFI
// control, |v_dq| above Rs|i| + |omega|(Ls|i| + flux) by more than HFI_VBAL_RATIO of the voltage
// limit for HFI_VBAL_PERIODS latches hfi_fault and coasts.
// The opposite failure (T200 bench): the tracker's PLL runs away to ~2000 rad/s el while the
// rotor stands. HFI only works below HFI_OMEGA_UP and hands back once the eSMO agrees, so
// |hfi_omega| above HFI_OVERSPEED * HFI_OMEGA_UP for HFI_VBAL_PERIODS also latches hfi_fault.
#define HFI_OVERSPEED 2.0f
#define HFI_VBAL_RATIO 0.3f
#define HFI_VBAL_PERIODS ((uint32_t)(0.02f * PWM_FREQ)) // 20 ms
// Start from standstill on HFI (MOTOR_STARTUP_HFI; Align -> I-f in the shadow test, hfi_takeover false):
//  1. Lock: zero current, tracker from theta = 0 for HFI_LOCK_TIME. It settles on the rotor's
//     d-axis or on its opposite end (sin 2e has both as stable points).
//  1b. Saliency probe: PLL frozen, injection HFI_PROBE_ANGLE off the tracker axis for
//     HFI_PROBE_TIME. With saliency the error reads ~ -sin(2 * 45 deg) / 2 = -0.5 rad (scaled by
//     the real K / HFI_K) on either end; without, ~0 and the tracker angle means nothing, so a
//     pulse would push the rotor (sim, Lq = Ld: kicked to -319 rad/s el). Above -HFI_PROBE_MIN:
//     no pulses, Align -> I-f.
//  2. Polarity: id = +HFI_POL_CURRENT then -HFI_POL_CURRENT along the tracker axis (no torque on
//     either end), HFI_POL_TIME each, |Zp| averaged over the second half of each. Current with the
//     magnet's flux saturates the iron: smaller incremental Ld, larger HF response. If the
//     negative pulse gives the larger |Zp| the tracker sits on the S end: add pi.
//  3. If the two differ by less than HFI_POL_MARGIN (relative) the result is not trusted: fall
//     back to the Align -> I-f start. Otherwise MOTOR_RUNNING with HFI in control at zero speed.
// The pulses are short and fast, and the current stays on the angle locked before them (it does not
// follow the tracker): a pulse on the wrong end is an unstable equilibrium, and with 30/80 ms
// pulses that followed the tracker the sim rotor turned 17-38 deg.
#define HFI_LOCK_TIME 0.1f           // s
#define HFI_PROBE_ANGLE (0.25f * PI) // rad
#define HFI_PROBE_TIME 0.01f         // s; the error is averaged over the last half
#define HFI_PROBE_MIN 0.2f           // rad: the probe must read below -HFI_PROBE_MIN (expected -0.5)
#define HFI_POL_SLEW 2000.0f         // id slew during the pulses (A/s): 3 ms to 6 A (current loop: 1 kHz)
#define HFI_POL_TIME 0.012f          // s per pulse; |Zp| averaged over the last half
#define HFI_POL_ZERO 0.006f          // s back at zero after each pulse

// Force-inline the small helpers on the ISR path (FOC_Update): an out-of-line call
// costs the call itself plus reloading every foc-> field it needs afterwards.
#if defined(__GNUC__)
#define FOC_INLINE static inline __attribute__((always_inline))
#else
#define FOC_INLINE static inline
#endif

// ============================================================================
// FUNCTION DEFINITIONS
// ============================================================================
// PI Controller functions
static void PI_Init(PI_Controller_t *controller, float Kp, float Ki, float sampling_time, float output_limit)
{
    if (controller == NULL)
    {
        return;
    }

    PI_Change_Params(controller, Kp, Ki, sampling_time);
    PI_Set_Output_Limit(controller, output_limit);
}

// ---------------------------------------------------------------------------
FOC_FAST_CODE static void PI_Reset(PI_Controller_t *controller)
{
    if (controller == NULL)
    {
        return;
    }

    controller->integral = 0.0f;
}

// ----------------------------------------------------------------------------
static void PI_Change_Params(PI_Controller_t *controller, float Kp, float Ki, float sampling_time)
{
    if (controller == NULL)
    {
        return;
    }

    controller->Kp = Kp;
    controller->Ki = Ki;
    controller->Ts = sampling_time;
    // Back-calculation tracking gain Kt = 1/Ti = Ki/Kp (units 1/s).
    // (Was Kp/Ki = Ti: inverted, which made the correction ~1e-8 per step in the
    // current loop, i.e. no anti-windup at all. Kp/Ki also gave inf/NaN for Ki = 0.)
    controller->Kb = (Kp > 0.0f) ? (Ki / Kp) : 0.0f;
    controller->integral = 0.0f;

    // Per-step products, so PI_Step needs no multiply by Ts. Kb*Ts is capped at 1
    // (full reset to the limit in one step); above 1 it over-corrects and can
    // oscillate (the speed loop has Ki/Kp*Ts = 2).
    controller->Ki_Ts = Ki * sampling_time;
    float kb_ts = controller->Kb * sampling_time;
    controller->Kb_Ts = (kb_ts < 1.0f) ? kb_ts : 1.0f;
}

// ----------------------------------------------------------------------------
static void PI_Set_Output_Limit(PI_Controller_t *controller, float output_limit)
{
    if (controller == NULL)
    {
        return;
    }

    controller->output_limit = output_limit;
}

// ----------------------------------------------------------------------------
// PI step without the NULL check, inlined into the ISR callers
FOC_INLINE float PI_Step(PI_Controller_t *controller, float error)
{
    // Proportional term
    float P_out = controller->Kp * error;

    // Integral increment (forward Euler: the output below uses the previous
    // integral state, and the state is advanced afterwards)
    float integral = controller->Ki_Ts * error;

    // Combine to the un-capped outputs
    float output = P_out + controller->integral;

    // Anti-windup for output (negative limit means unlimited; zero clamps to zero)
    if (controller->output_limit >= 0.0f)
    {
        float saturated_output = CONSTRAIN(output, -(controller->output_limit), controller->output_limit);
        // Back-calculation: bleed the integrator by Kt*Ts*(u_sat - u), Kt*Ts capped at 1
        integral += (saturated_output - output) * controller->Kb_Ts; // Anti-windup correction
        output = saturated_output;                                   // Clamp the output to the specified limits
    }

    // Update the integral state for the next iteration
    controller->integral += integral;

    return output;
}


//===========================================================================
// Trigonometric functions
//===========================================================================
// Table look-up with linear interpolation: the fastest option on Cortex-M4
// (SRAM loads are 1-2 cycles, each FPU multiply-add 3). Accuracy < 5e-6 abs,
// the interpolation error (2pi/N)^2/8 for N = 1024.
//
// ONE table holds sin over 1.25 turns plus two guard samples:
//   sin(x) = T[i], cos(x) = sin(x + pi/2) = T[i + N/4]
// so there is no separate cos table and T[i + 1] never needs wrapping.
// The second guard sample covers i = N, which an angle just below 2pi can
// round to.
//
// Every public function has a frameless fast path for angles in [0, 2pi).
// Other angles take a tail call into a noinline slow path (Normalize_Angle),
// so the rarely used fmodf code never adds a stack frame to the common case.
#define TRIG_TABLE_SIZE 1024 // Samples per turn (power of 2)
#define TRIG_TABLE_MASK (TRIG_TABLE_SIZE - 1)
#define TRIG_QUARTER (TRIG_TABLE_SIZE / 4)
#define TRIG_TABLE_DENSITY ((float)TRIG_TABLE_SIZE / TWO_PI) // Samples per radian
#define TRIG_TWO_PI_BITS 0x40C90FDBu                         // IEEE-754 bits of TWO_PI

#if defined(__GNUC__)
#define TRIG_UNLIKELY(x) __builtin_expect(!!(x), 0)
#define TRIG_NOINLINE __attribute__((noinline))
#else
#define TRIG_UNLIKELY(x) (x)
#define TRIG_NOINLINE
#endif

static float trig_table[TRIG_TABLE_SIZE + TRIG_QUARTER + 2]; // 1282 floats, 5 KB of RAM

//----------------------------------------------------------------------------
// Fill the table. MUST run before any trig function (FOC_Init does this).
// sinf (not sin) keeps the double-precision libm out of the image (~5 KB flash).
static void Init_Trigonometric_Tables(void)
{
    for (int i = 0; i < TRIG_TABLE_SIZE + TRIG_QUARTER + 2; i++)
    {
        trig_table[i] = sinf((float)i * (TWO_PI / (float)TRIG_TABLE_SIZE));
    }
}

//----------------------------------------------------------------------------
// Angle reduction constants
#define NORM_INV_TWO_PI 0.159154943f         // 1 / (2pi)
#define NORM_TWO_PI_ERR -1.74845553e-07f     // (true 2pi) - TWO_PI
#define NORM_TWO_PI_HI 6.28314208984375f     // 2pi, high 17 bits (k*HI exact for |k| < 128)
#define NORM_TWO_PI_LO 4.32173364970e-05f    // 2pi - NORM_TWO_PI_HI
#if defined(__FP_FAST_FMAF)
#define NORM_REDUCE_LIMIT 1.0e8f // Above this, float cannot resolve an angle
#else
#define NORM_REDUCE_LIMIT 800.0f // Cody-Waite is exact while |k| < 128
#endif

//----------------------------------------------------------------------------
// Normalize angle to [0, 2pi)
// Fast paths cover the common cases (already in range, or one turn out) with
// compares only; fmodf is used only for angles more than one turn out.
FOC_FAST_CODE static void Normalize_Angle(float *angle)
{
    float a = *angle;

    // Already in range: the usual case (NaN fails this test)
    if (a >= 0.0f && a < TWO_PI)
    {
        return;
    }

    // One turn above: a - 2pi is exact here (Sterbenz lemma)
    if (a >= TWO_PI && a < 2.0f * TWO_PI)
    {
        *angle = a - TWO_PI;
        return;
    }

    // One turn below. A tiny negative angle rounds to exactly 2pi: map to 0.
    if (a < 0.0f && a >= -TWO_PI)
    {
        a += TWO_PI;
        *angle = (a < TWO_PI) ? a : 0.0f;
        return;
    }

    // More than one turn out: subtract k whole turns, k = floor(a / 2pi).
    // Constant time (no loop that could hang the ISR) and no library call.
    // Within the limit, k fits an int and is exact as a float; NaN fails the test.
    if (fabsf(a) < NORM_REDUCE_LIMIT)
    {
        float q = a * NORM_INV_TWO_PI;
        int k = (int)q; // Well defined: |q| < 2^24
        if (q < (float)k)
        {
            k--; // floor() for negative q (floorf is a library call on Cortex-M4)
        }
        float kf = (float)k;
#if defined(__FP_FAST_FMAF)
        // Hardware FMA (VFMA on Cortex-M4F): the product k*TWO_PI is not rounded,
        // so the first step gives exactly what fmodf(a, TWO_PI) would. The second
        // corrects for TWO_PI being 1.75e-7 above true 2pi, which fmodf cannot do.
        a = fmaf(-kf, TWO_PI, a);
        a = fmaf(-kf, NORM_TWO_PI_ERR, a);
#else
        // No fast FMA: Cody-Waite split, 2pi = HI + LO with a 17-bit HI, so
        // k*HI is exact for |k| < 128, hence the 800 rad limit on this path.
        a = (a - kf * NORM_TWO_PI_HI) - kf * NORM_TWO_PI_LO;
#endif
        // q can round across an integer: then the result is one turn out
        if (a < 0.0f)
        {
            a += TWO_PI;
        }
        else if (a >= TWO_PI)
        {
            a -= TWO_PI;
        }
        *angle = (a >= 0.0f && a < TWO_PI) ? a : 0.0f;
        return;
    }

    // Non-finite: 0, so a corrupted angle cannot propagate NaN into the PWM
    if (!isfinite(a))
    {
        *angle = 0.0f;
        return;
    }

    // Beyond the limit: fmodf. (With FMA that is |a| >= 1e8 rad, where a float can
    // no longer resolve the angle within a turn, so the value is meaningless.)
    a = fmodf(a, TWO_PI);
    if (a < 0.0f)
    {
        a += TWO_PI;
        if (a >= TWO_PI)
        {
            a = 0.0f;
        }
    }

    *angle = a;
}

//----------------------------------------------------------------------------
// True iff angle is in [0, 2pi): one unsigned compare on the IEEE-754 bits.
// Non-negative floats order like their bit patterns; negative values (and -0),
// NaN, +-inf and angles >= 2pi all compare >= the bits of 2pi.
// This also makes the later float -> int conversion well defined.
static inline bool Trig_In_Range(float angle)
{
    uint32_t bits;
    memcpy(&bits, &angle, sizeof(bits)); // Compiles to a single VMOV
    return bits < TRIG_TWO_PI_BITS;
}

//---------------------------------------------------------------------------
// Sine and cosine for an angle in [0, 2pi] (no range check).
// Used by SMO_Update and by the slow paths. The mask is a cheap memory-safety
// net for callers that break the precondition.
static inline void Sin_Cos_Wrapped(float angle, float *sin_out, float *cos_out)
{
    float x = angle * TRIG_TABLE_DENSITY;
    int i = (int)x;
    float frac = x - (float)i;
    const float *p = &trig_table[i & TRIG_TABLE_MASK];

    *sin_out = p[0] + frac * (p[1] - p[0]);
    *cos_out = p[TRIG_QUARTER] + frac * (p[TRIG_QUARTER + 1] - p[TRIG_QUARTER]);
}

// Slow paths: wrap the angle into [0, 2pi), then the same interpolation
FOC_FAST_CODE TRIG_NOINLINE static void Fast_Sin_Cos_Slow(float angle, float *sin_out, float *cos_out)
{
    Normalize_Angle(&angle);
    Sin_Cos_Wrapped(angle, sin_out, cos_out);
}

FOC_FAST_CODE TRIG_NOINLINE static float Fast_Sin_Slow(float angle)
{
    float s, c;
    Fast_Sin_Cos_Slow(angle, &s, &c);
    return s;
}

FOC_FAST_CODE TRIG_NOINLINE static float Fast_Cos_Slow(float angle)
{
    float s, c;
    Fast_Sin_Cos_Slow(angle, &s, &c);
    return c;
}

//----------------------------------------------------------------------------
// Sine (any angle)
FOC_FAST_CODE static float Fast_Sin(float angle)
{
    if (TRIG_UNLIKELY(!Trig_In_Range(angle)))
    {
        return Fast_Sin_Slow(angle); // Tail call: no frame on the fast path
    }
    float x = angle * TRIG_TABLE_DENSITY;
    int i = (int)x;
    float frac = x - (float)i;
    const float *p = &trig_table[i];
    return p[0] + frac * (p[1] - p[0]);
}

//----------------------------------------------------------------------------
// Cosine (any angle)
FOC_FAST_CODE static float Fast_Cos(float angle)
{
    if (TRIG_UNLIKELY(!Trig_In_Range(angle)))
    {
        return Fast_Cos_Slow(angle);
    }
    float x = angle * TRIG_TABLE_DENSITY;
    int i = (int)x;
    float frac = x - (float)i;
    const float *p = &trig_table[i + TRIG_QUARTER];
    return p[0] + frac * (p[1] - p[0]);
}

//---------------------------------------------------------------------------
// Interpolate sin and cos at index i (0..N, proven by the caller), fraction frac
// The cos samples sit N/4 floats (1024 bytes) further on, beyond VLDR's
// 1020-byte offset range, so take one explicit second pointer: that keeps the
// fast path inside the four scratch registers (no push of lr).
static inline void Trig_Interp_Sin_Cos(int i, float frac, float *sin_out, float *cos_out)
{
    const float *p = &trig_table[i];
    const float *q = p + TRIG_QUARTER;
    *sin_out = p[0] + frac * (p[1] - p[0]);
    *cos_out = q[0] + frac * (q[1] - q[0]);
}

//---------------------------------------------------------------------------
// Sine and cosine (any angle)
FOC_FAST_CODE static void Fast_Sin_Cos(float angle, float *sin_out, float *cos_out)
{
    if (TRIG_UNLIKELY(!Trig_In_Range(angle)))
    {
        Fast_Sin_Cos_Slow(angle, sin_out, cos_out);
        return;
    }
    float x = angle * TRIG_TABLE_DENSITY;
    int i = (int)x; // 0..N: in range, so no mask needed
    Trig_Interp_Sin_Cos(i, x - (float)i, sin_out, cos_out);
}

//---------------------------------------------------------------------------
// atan2(y, x) in [-pi, pi], following C99 atan2 for zeros, infinities and NaN.
// Reduces to atan(a), a = min/max in [0, 1], then a degree-11 minimax
// polynomial: max error 1.8e-6 rad.
#define ATAN_A1 0.9999772310256958f
#define ATAN_A3 -0.33262282609939575f
#define ATAN_A5 0.19354037940502167f
#define ATAN_A7 -0.11642647534608841f
#define ATAN_A9 0.05264734849333763f
#define ATAN_A11 -0.011719133704900742f

FOC_FAST_CODE static float Fast_Atan2(float y, float x)
{
    if (isnan(x) || isnan(y))
    {
        return x + y; // Propagate NaN
    }

    float abs_x = fabsf(x);
    float abs_y = fabsf(y);
    bool steep = abs_y > abs_x; // |angle| in (pi/4, 3pi/4): use the cot form
    float mx = steep ? abs_y : abs_x;
    float mn = steep ? abs_x : abs_y;

    if (mx == 0.0f)
    {
        // atan2(+-0, +0) = +-0, atan2(+-0, -0) = +-pi
        return signbit(x) ? copysignf(PI, y) : y;
    }

    // mn == mx also covers both infinite (inf/inf would be NaN)
    float a = (mn == mx) ? 1.0f : mn / mx;
    float s = a * a;
    float r = a * (ATAN_A1 + s * (ATAN_A3 + s * (ATAN_A5 + s * (ATAN_A7 + s * (ATAN_A9 + s * ATAN_A11)))));

    if (steep)
    {
        r = HALF_PI - r;
    }
    if (signbit(x))
    {
        r = PI - r;
    }
    return signbit(y) ? -r : r;
}

//---------------------------------------------------------------------------
// Square root. Returns 0 for x <= 0 (as before), NaN for NaN.
// On a Cortex-M with a single-precision FPU this is one VSQRT.F32 (~14 cycles).
// The inline asm avoids the errno check plus library call that sqrtf()
// compiles to unless -fno-math-errno is given.
// (The previous Newton iteration started from x itself and did not converge
//  in 5 steps outside roughly [0.5, 50]: e.g. Fast_Sqrt(10000) was 323.)
FOC_FAST_CODE static float Fast_Sqrt(float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }
#if defined(__GNUC__) && defined(__ARM_FP) && (__ARM_FP & 0x4)
    float r;
    __asm__("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x));
    return r;
#else
    return sqrtf(x);
#endif
}

//===========================================================================
// FOC utility functions
//===========================================================================

// ---------------------------------------------------------------------------
// Space Vector PWM Modulation (inlined into FOC_Update; SVPWM_Modulation wraps it)
FOC_INLINE void SVPWM_Core(float v_alpha, float v_beta, float v_dc, uint32_t pwm_period,
                           uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w)
{
    int32_t max_pwm = (int32_t)pwm_period - 2; // Ensure we don't exceed the period

    // 0. Guard against an invalid bus voltage before dividing by it
    if (!(v_dc > 0.0f))
    {
        *pwm_u = 0;
        *pwm_v = 0;
        *pwm_w = 0;
        return;
    }

    // 0.1 Clamp the reference vector magnitude to the linear SVPWM range
    // (|v| <= v_dc/sqrt(3)). Scaling both components preserves the angle,
    // whereas clamping them independently would distort it.
    // Compare squared magnitudes: the square root is only needed when clamping.
    float v_max = v_dc * ONE_OVER_SQRT3;
    float v_mag_sq = v_alpha * v_alpha + v_beta * v_beta;
    if (v_mag_sq > v_max * v_max)
    {
        float scale = v_max / Fast_Sqrt(v_mag_sq);
        v_alpha *= scale;
        v_beta *= scale;
    }

    // 1. Phase references (inverse Clarke, amplitude-invariant)
    float v_u = v_alpha;
    float v_v = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
    float v_w = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

    // 2. Min-max common-mode injection. Centring the three references between
    // the rails gives exactly the duties of sector-based SVPWM (same linear range
    // |v| <= v_dc/sqrt(3)) without any sector logic.
    // (The previous sector formulas used alpha/beta where the line-voltage
    // projections belong and applied only 2/3 of the commanded voltage, which
    // also fed the eSMO a voltage 1.5x the real one.)
    float v_hi = (v_u > v_v) ? v_u : v_v;
    v_hi = (v_w > v_hi) ? v_w : v_hi;
    float v_lo = (v_u < v_v) ? v_u : v_v;
    v_lo = (v_w < v_lo) ? v_w : v_lo;
    float v_offset = -0.5f * (v_hi + v_lo);

    // 3. Duty counts: 50 % plus the centred reference scaled by period / v_dc
    float counts_per_volt = (float)pwm_period / v_dc;
    float mid = 0.5f * (float)pwm_period + 0.5f; // + 0.5 rounds to nearest below
    int32_t duty_u = (int32_t)(mid + (v_u + v_offset) * counts_per_volt);
    int32_t duty_v = (int32_t)(mid + (v_v + v_offset) * counts_per_volt);
    int32_t duty_w = (int32_t)(mid + (v_w + v_offset) * counts_per_volt);

    // 4. Clamp outputs to valid range (both ends) before the unsigned store
    *pwm_u = (uint32_t)CONSTRAIN(duty_u, 0, max_pwm);
    *pwm_v = (uint32_t)CONSTRAIN(duty_v, 0, max_pwm);
    *pwm_w = (uint32_t)CONSTRAIN(duty_w, 0, max_pwm);
}


// ---------------------------------------------------------------------------
// CLARKE TRANSFORM (abc → αβ)
// Simplified Clarke (only need 2 phase currents)
static void Clarke_Transform(float i_a, float i_b, float *i_alpha, float *i_beta)
{
    *i_alpha = i_a;
    *i_beta = (i_a * ONE_OVER_SQRT3) + (i_b * TWO_OVER_SQRT3);
}

// ---------------------------------------------------------------------------
// PARK TRANSFORM (αβ → dq)
// The transforms inline the table look-up. For theta outside [0, 2pi) they
// tail-call a noinline slow variant, so the fast path needs no stack frame.
FOC_FAST_CODE TRIG_NOINLINE static void Park_Transform_Slow(float i_alpha, float i_beta, float theta, float *i_d, float *i_q)
{
    float cos_theta, sin_theta;
    Fast_Sin_Cos_Slow(theta, &sin_theta, &cos_theta);
    *i_d = cos_theta * i_alpha + sin_theta * i_beta;
    *i_q = -sin_theta * i_alpha + cos_theta * i_beta;
}

// Inlined into the current loop; Park_Transform wraps it
FOC_INLINE void Park_Core(float i_alpha, float i_beta, float theta, float *i_d, float *i_q)
{
    if (TRIG_UNLIKELY(!Trig_In_Range(theta)))
    {
        Park_Transform_Slow(i_alpha, i_beta, theta, i_d, i_q);
        return;
    }

    float cos_theta, sin_theta;
    float x = theta * TRIG_TABLE_DENSITY;
    int i = (int)x; // 0..N: in range, so no mask needed
    Trig_Interp_Sin_Cos(i, x - (float)i, &sin_theta, &cos_theta);

    *i_d = cos_theta * i_alpha + sin_theta * i_beta;
    *i_q = -sin_theta * i_alpha + cos_theta * i_beta;
}

FOC_FAST_CODE static void Park_Transform(float i_alpha, float i_beta, float theta, float *i_d, float *i_q)
{
    Park_Core(i_alpha, i_beta, theta, i_d, i_q);
}

FOC_FAST_CODE static void Clarke_Park_Transform(float i_a, float i_b, float theta,
                           float *i_d, float *i_q)
{
    // Clarke Transform
    float i_alpha = i_a;
    float i_beta = (i_a * ONE_OVER_SQRT3) + (i_b * TWO_OVER_SQRT3);

    // Park Transform
    Park_Transform(i_alpha, i_beta, theta, i_d, i_q);
}

FOC_FAST_CODE TRIG_NOINLINE static void Inverse_Park_Transform_Slow(float v_d, float v_q, float theta, float *v_alpha, float *v_beta)
{
    float cos_theta, sin_theta;
    Fast_Sin_Cos_Slow(theta, &sin_theta, &cos_theta);
    *v_alpha = cos_theta * v_d - sin_theta * v_q;
    *v_beta = sin_theta * v_d + cos_theta * v_q;
}

// Inlined into FOC_Update; Inverse_Park_Transform wraps it
FOC_INLINE void Inverse_Park_Core(float v_d, float v_q, float theta, float *v_alpha, float *v_beta)
{
    if (TRIG_UNLIKELY(!Trig_In_Range(theta)))
    {
        Inverse_Park_Transform_Slow(v_d, v_q, theta, v_alpha, v_beta);
        return;
    }

    float cos_theta, sin_theta;
    float x = theta * TRIG_TABLE_DENSITY;
    int i = (int)x; // 0..N: in range, so no mask needed
    Trig_Interp_Sin_Cos(i, x - (float)i, &sin_theta, &cos_theta);

    *v_alpha = cos_theta * v_d - sin_theta * v_q;
    *v_beta = sin_theta * v_d + cos_theta * v_q;
}


// ---------------------------------------------------------------------------
// Calculate the phase currents from ADC values
// NOTE: *i_a and *i_b are in/out. On entry they must hold the previous filtered
// value (they are the LPF state); on exit they hold the new filtered value.
// Callers must pass persistent storage (e.g. &foc->i_a, &foc->i_b).
static void Calculate_Phase_Currents(uint16_t adc_u, uint16_t adc_v, uint16_t offset_u, uint16_t offset_v,
                              float *i_a, float *i_b)
{
    if (i_a == NULL || i_b == NULL)
    {
        return;
    }

    int32_t eff_adc_u = (int32_t)adc_u - (int32_t)offset_u;
    int32_t eff_adc_v = (int32_t)adc_v - (int32_t)offset_v;

    // ADC counts -> shunt current -> low-pass filter, carrying the previous filtered
    // value held in *i_a / *i_b as the filter state. ALPHA * ADC_2_CURRENT folds into
    // one constant at compile time, so each phase is one multiply and one multiply-add.
    const float k_in = ALPHA_LPF_MEAS * ADC_2_CURRENT;
    *i_a = (float)eff_adc_u * k_in + BETA_LPF_MEAS * (*i_a);
    *i_b = (float)eff_adc_v * k_in + BETA_LPF_MEAS * (*i_b);
}

// ============================================================================
// Sliding-mode observer functions to estimate rotor position and speed
// ============================================================================

// ---------------------------------------------------------------------------
// Pre-compute the discrete current-observer coefficients.
// Guards against a degenerate Rs/Ls (e.g. a failed calibration), which would
// otherwise produce inf/NaN coefficients and poison every later SMO update.
static void SMO_Compute_Coefficients(SMO_t *smo, BLDC_Specs_t *bldc, float sampling_time)
{
    float Rs = bldc->Rs;
    float Ls = bldc->Ls;

    if (!(Rs > 1e-6f))
    {
        Rs = RS; // Fall back to the configured default
    }
    if (!(Ls > 1e-9f))
    {
        Ls = LS;
    }

    // Everything that depends only on Rs, Ld, Lq, Ts
    // Current model uses Ld (extended-EMF form, valid for salient motors too)
    float Ld = (bldc->Ld > 1e-9f) ? bldc->Ld : Ls;
    float Lq = (bldc->Lq > 1e-9f) ? bldc->Lq : Ld;
    float F = expf((-Rs * sampling_time) / Ld);
    float G = (1.0f - F) / Rs;
    smo->esmo_F = F;
    smo->esmo_G = G;
    smo->esmo_dL = Ld - Lq;

    // Boundary layer phi = k*G/lambda, so inside it z = (lambda/G)*error and the
    // current-error pole is F - lambda (stable for 0 < lambda < 1 + F).
    // lambda = F puts the pole at 0 (deadbeat). Because k*sat(err/phi) equals
    // clamp((lambda/G)*err, -k, k), only the slope lambda/G is needed at run time.
    float lambda = (ESMO_BOUNDARY_GAIN > 0.0f) ? ESMO_BOUNDARY_GAIN : F;
    lambda = CONSTRAIN(lambda, 0.05f, 0.95f * (1.0f + F));
    smo->esmo_z_gain = lambda / G;

    // Linear-region steady-state z is kappa*(E - E_hat), kappa = lambda/(1-F+lambda).
    // Scaling z by 1/kappa restores unity EMF gain. It is folded into the LPF
    // coefficient: E_hat += (a/kappa) * z.
    smo->esmo_inv_kappa = (1.0f - F + lambda) / lambda;

    // Back-EMF LPF: wc = ratio*|w| clamped to [wc_lo, wc_hi]; a = 1 - exp(-wc*Ts).
    // At the clamps a is constant, so precompute it there (already times 1/kappa).
    smo->esmo_wc_lo = ESMO_WC_MIN;
    smo->esmo_wc_hi = ESMO_WC_MAX_TS / sampling_time;
    smo->esmo_ratio_ts = ESMO_WC_RATIO * sampling_time;
    smo->esmo_x_lo = ESMO_WC_MIN * sampling_time;
    smo->esmo_a_lo = (1.0f - expf(-smo->esmo_wc_lo * sampling_time)) * smo->esmo_inv_kappa;
    smo->esmo_a_hi = (1.0f - expf(-ESMO_WC_MAX_TS)) * smo->esmo_inv_kappa;

    // While wc tracks speed, the LPF lag atan(w/wc) = atan(1/ratio) is constant
    smo->esmo_lag = atanf(1.0f / ESMO_WC_RATIO);

    smo->esmo_delay_ts = ESMO_DELAY_COMP * sampling_time;

    printf("   SMO coefficients: F = %f, G = %f, z_gain = %f\r\n",
           smo->esmo_F, smo->esmo_G, smo->esmo_z_gain);
}

// ---------------------------------------------------------------------------
// Function to initialize SMO library
static void SMO_Init(SMO_t *smo, BLDC_Specs_t *bldc, float sampling_time)
{
    if (smo == NULL || bldc == NULL)
    {
        printf("SMO_Init: Invalid arguments (smo=%p, bldc=%p)\r\n", (void *)smo, (void *)bldc);
        return;
    }

    PI_Init(&(smo->pll), KP_PLL, KI_PLL, sampling_time, PI_NO_OUTPUT_LIMIT);

    smo->sampling_time = sampling_time;
    SMO_Reset(smo);

    // Pre-computed SMO coefficients
    SMO_Compute_Coefficients(smo, bldc, sampling_time);
}

// ---------------------------------------------------------------------------
// Function to reset SMO states
FOC_FAST_CODE static void SMO_Reset(SMO_t *smo)
{
    if (smo == NULL)
    {
        return;
    }

    PI_Reset(&(smo->pll));
    smo->theta_est = 0.0f;
    smo->omega_est = 0.0f;
    smo->e_alpha_filtered = 0.0f;
    smo->e_beta_filtered = 0.0f;
    smo->i_alpha_est = 0.0f;
    smo->i_beta_est = 0.0f;
    smo->k_slide = ESMO_K_SLIDE_MIN;
    smo->theta_emf = 0.0f;
    smo->pll_err_filtered = 1.0f; // Start "unlocked"
    smo->emf_magnitude = 0.0f;
    smo->locked = false;
    smo->unlock_count = 0;
}

// ---------------------------------------------------------------------------
// Function to update BLDC specifications
static void SMO_BLDC_Specs_Update(SMO_t *smo, BLDC_Specs_t *bldc, float sampling_time)
{
    if (smo == NULL || bldc == NULL)
    {
        return;
    }

    smo->sampling_time = sampling_time;
    PI_Change_Params(&(smo->pll), smo->pll.Kp, smo->pll.Ki, sampling_time); // Refresh pll.Ki_Ts

    // Reset SMO states
    SMO_Reset(smo);

    // Pre-computed SMO coefficients
    SMO_Compute_Coefficients(smo, bldc, sampling_time);
}

// ---------------------------------------------------------------------------
// Wrap an angle that is at most one turn outside [0, 2pi). Cheaper than
// Normalize_Angle (no fmodf); valid for the bounded increments used below.
static inline float Wrap_Once(float a)
{
    if (a >= TWO_PI)
        a -= TWO_PI;
    else if (a < 0.0f)
        a += TWO_PI;
    return a;
}

// ---------------------------------------------------------------------------
// Enhanced sliding-mode observer (eSMO) with PLL.
//
// Structure follows TI's eSMO (Motor Control SDK Universal Lab, SPRUJ26 2.3.1):
//   current model with E_hat inside  ->  saturated sliding term z  ->
//   low-pass filtered back-EMF  ->  PLL  ->  LPF phase-lag compensation.
//
// All parameter-only terms are pre-computed in SMO_Compute_Coefficients().
// Per-call cost: 1 Fast_Sqrt, 1 division, 2 table look-ups, no expf/fmodf.
// Fast_Atan2 is only used while the LPF cutoff sits at its floor or ceiling.
//
// Inputs:  v_alpha/v_beta = voltage applied during the period that ends at this
//          current sample (i.e. the previous call's inverse-Park output).
//          i_alpha/i_beta  = currents sampled now.
// Outputs: smo->theta_est (rad, 0..2pi), smo->omega_est (electrical rad/s).
// Returns true while the observer is locked (with hysteresis), false otherwise.
// Runs in the PWM ISR: no printf in here.
FOC_FAST_CODE static bool SMO_Update(SMO_t *smo, float v_bus, float v_alpha, float v_beta, float i_alpha_meas, float i_beta_meas)
{
    if (smo == NULL)
    {
        return false;
    }

    const float Ts = smo->sampling_time;
    const float omega = smo->omega_est; // Speed from the previous step (rad/s, electrical)
    const float k = smo->k_slide;
    float e_alpha = smo->e_alpha_filtered;
    float e_beta = smo->e_beta_filtered;

    // Step 1+2: Sliding feedback z = k*sat(err/phi), phi = k*G/lambda, which is
    // identical to clamp((lambda/G)*err, -k, k): no division.
    float z_alpha = CONSTRAIN(smo->esmo_z_gain * (smo->i_alpha_est - i_alpha_meas), -k, k);
    float z_beta = CONSTRAIN(smo->esmo_z_gain * (smo->i_beta_est - i_beta_meas), -k, k);

    // Step 3: Current model, extended-EMF form (TI eq. 11 / 17):
    //   Ld di/dt = v - Rs i - w(Ld - Lq) J i - E_hat - z
    float w_dL = omega * smo->esmo_dL; // 0 for a non-salient motor
    smo->i_alpha_est = smo->esmo_F * smo->i_alpha_est +
                       smo->esmo_G * (v_alpha - e_alpha - z_alpha - w_dL * i_beta_meas);
    smo->i_beta_est = smo->esmo_F * smo->i_beta_est +
                      smo->esmo_G * (v_beta - e_beta - z_beta + w_dL * i_alpha_meas);

    // Step 4: Back-EMF LPF (TI eq. 13 / 20). Raw EMF = E_hat + z/kappa, so
    // E_hat += a*(raw - E_hat) reduces to E_hat += (a/kappa)*z.
    // wc = ratio*|w| clamped; the lag atan(w/wc) is compensated in step 7.
    // Worked in x = wc*Ts (one multiply by the pre-computed ratio*Ts).
    float x = smo->esmo_ratio_ts * fabsf(omega);
    float a_lpf; // a/kappa
    float lag;   // LPF phase lag to add back (rad)
    if (x <= smo->esmo_x_lo)
    {
        a_lpf = smo->esmo_a_lo;
        lag = Fast_Atan2(omega, smo->esmo_wc_lo);
    }
    else if (x >= ESMO_WC_MAX_TS)
    {
        a_lpf = smo->esmo_a_hi;
        lag = Fast_Atan2(omega, smo->esmo_wc_hi);
    }
    else
    {
        // a = 1 - exp(-x), x = wc*Ts in (x_lo, 0.7]: 5th-order series, error < 0.05 %
        a_lpf = x * (1.0f - x * (0.5f - x * (0.16666667f - x * (0.041666667f - x * 0.0083333333f))));
        a_lpf *= smo->esmo_inv_kappa;
        lag = (omega >= 0.0f) ? smo->esmo_lag : -smo->esmo_lag; // atan(1/ratio)
    }

    e_alpha += a_lpf * z_alpha;
    e_beta += a_lpf * z_beta;

    // Guard: one check catches NaN/inf in any of the four states
    if (!isfinite(smo->i_alpha_est + smo->i_beta_est + e_alpha + e_beta))
    {
        SMO_Reset(smo);
        return false;
    }
    smo->e_alpha_filtered = e_alpha;
    smo->e_beta_filtered = e_beta;

    // Magnitude and its inverse (one sqrt, one division); the inverse
    // normalises the PLL error so its bandwidth is independent of speed.
    float emf_sq = e_alpha * e_alpha + e_beta * e_beta;
    float emf_mag = Fast_Sqrt(emf_sq);
    float inv_emf = 1.0f / ((emf_mag > 1e-3f) ? emf_mag : 1e-3f); // Not fmaxf: a library call on Cortex-M4
    smo->emf_magnitude = emf_mag;

    // Step 5: Schedule the sliding gain (TI: k > max|E|), capped at the largest
    // phase voltage the inverter can produce.
    float k_max = ((v_bus > 1.0f) ? v_bus : V_RATED) * ONE_OVER_SQRT3;
    k_max = (k_max > ESMO_K_SLIDE_MIN) ? k_max : ESMO_K_SLIDE_MIN;
    smo->k_slide = CONSTRAIN(ESMO_K_SLIDE_MARGIN * emf_mag, ESMO_K_SLIDE_MIN, k_max);

    // Step 6: PLL on the EMF vector angle (theta_emf is kept in [0, 2pi)).
    //   err = (e_beta*cos - e_alpha*sin)/|E| = sin(theta_emf_true - theta_emf)
    // PI inlined (same forward-Euler form as PI_Update, which has no output
    // limit for the PLL). The integral clamp bounds the speed estimate.
    float sin_t, cos_t;
    float theta_emf = smo->theta_emf; // Angle the error is evaluated against
    Sin_Cos_Wrapped(theta_emf, &sin_t, &cos_t);
    float pll_err = (e_beta * cos_t - e_alpha * sin_t) * inv_emf;

    float omega_pll = smo->pll.Kp * pll_err + smo->pll.integral;
    float omega_new = CONSTRAIN(smo->pll.integral + smo->pll.Ki_Ts * pll_err, -ESMO_OMEGA_MAX, ESMO_OMEGA_MAX);
    smo->pll.integral = omega_new;
    omega_pll = CONSTRAIN(omega_pll, -ESMO_OMEGA_MAX, ESMO_OMEGA_MAX);

    // |omega_pll*Ts| < 2pi for any sane Ts, so a single wrap suffices
    smo->theta_emf = Wrap_Once(theta_emf + omega_pll * Ts);

    // The integrator is the smoother speed estimate (the P term carries noise)
    smo->omega_est = omega_new;

    // Step 7: Rotor angle = EMF angle -/+ 90 deg (forward/reverse) + LPF lag
    // + residual latency. The sum stays within one turn of [0, 2pi).
    smo->theta_est = Wrap_Once(theta_emf + ((omega_new >= 0.0f) ? -HALF_PI : HALF_PI) +
                               lag + smo->esmo_delay_ts * omega_new);

    // Step 8: Lock detection with hysteresis. A single noisy sample must not
    // drop lock, because FOC_Current_Loop_Update coasts on the first false.
    smo->pll_err_filtered += ESMO_PLL_ERR_ALPHA * (fabsf(pll_err) - smo->pll_err_filtered);

    if (!smo->locked)
    {
        smo->locked = (emf_mag > MIN_BEMF_AMPLITUDE) && (smo->pll_err_filtered < ESMO_LOCK_ERR);
        smo->unlock_count = 0;
    }
    else if ((emf_mag < 0.5f * MIN_BEMF_AMPLITUDE) || (smo->pll_err_filtered > ESMO_UNLOCK_ERR))
    {
        if (++(smo->unlock_count) >= ESMO_UNLOCK_COUNT)
        {
            smo->locked = false;
            smo->unlock_count = 0;
        }
    }
    else
    {
        smo->unlock_count = 0;
    }

    return smo->locked;
}

// ============================================================================
// FOC control functions
// ============================================================================

// ---------------------------------------------------------------------------
// Function to initialize FOC library
// Clear the 6x harmonic compensation state (see FOC_Harmonic_Comp)
FOC_FAST_CODE static void Harm_Reset(FOC_t *foc)
{
    foc->harm_p[0] = foc->harm_p[1] = 0.0f;
    foc->harm_n[0] = foc->harm_n[1] = 0.0f;
    foc->harm_div = 0;
}

// Products of configuration values that FOC_Update would otherwise recompute every
// period. Rerun whenever sampling_time or bldc changes (FOC_Init, FOC_Recalc_Params).
static void FOC_Precompute(FOC_t *foc)
{
    foc->ipark_adv = IPARK_DELAY_COMP * foc->sampling_time;
    foc->harm_k = HARM_KI * foc->sampling_time;
    // HFI amplitude for a HFI_STEP_CURRENT step per period at the calibrated inductance
    foc->hfi_v = fminf(HFI_STEP_CURRENT * foc->bldc.Ls / foc->sampling_time, HFI_V_MAX_RATIO * V_RATED);

    // MTPA: id = (psi - sqrt(psi^2 + 8 dL^2 Is^2)) / (4 dL), dL = Lq - Ld
    float L_diff = foc->bldc.Lq - foc->bldc.Ld;
    if (L_diff > MIN_SALIENCY_H)
    {
        foc->mtpa_k = 8.0f * L_diff * L_diff;
        foc->mtpa_inv_4l = 1.0f / (4.0f * L_diff);
    }
    else
    {
        foc->mtpa_k = 0.0f;
        foc->mtpa_inv_4l = 0.0f; // Non-salient: all current on q
    }
}

// Initialise the drive: defaults from foc_lib.h, idle, bridge off. pwm_res = the PWM timer's period
// count (100 % duty). The current-sense offsets start at V_OFFSET; overwrite foc->adc_offset_a/b with
// measured zero-current readings before running.
void FOC_Init(FOC_t *foc, uint32_t pwm_res)
{
    // Initialize trigonometric tables
    Init_Trigonometric_Tables();

    if (foc == NULL)
    {
        // Error handling (could be an assertion or error code)
        return;
    }

    // Initialize FOC structure
    foc->rotor_dir = NORMAL_DIRECTION;
    foc->pwm_resolution = pwm_res;
    foc->sampling_time = TS;
    foc->foc_mode = FOC_IDLE;
    foc->v_bus = V_RATED; // Default bus voltage
    foc->v_d = 0.0f;
    foc->v_q = 0.0f;
    foc->i_a = 0.0f;
    foc->i_b = 0.0f;
    foc->i_d = 0.0f;
    foc->i_q = 0.0f;
    foc->id_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->is_ref = 0.0f;
    foc->speed_ref = 0.0f;
    foc->speed_ref_ramp = 0.0f;
    foc->speed_div = 0;
    foc->vbus_fault = false;
    foc->vbus_uv_fault = false;
    foc->vbus_uv_cnt = 0;
    Harm_Reset(foc);
    foc->harm_rp[0] = foc->harm_rn[0] = 1.0f;
    foc->harm_rp[1] = foc->harm_rn[1] = 0.0f;
    foc->v_alpha = 0.0f;
    foc->v_beta = 0.0f;
    foc->id_filtered = 0.0f;
    foc->iq_filtered = 0.0f;
    foc->id_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->pwm_a = 0;
    foc->pwm_b = 0;
    foc->pwm_c = 0;
    foc->outputs_enabled = false;
    foc->theta = 0.0f;
    foc->omega = 0.0f;
    foc->startup_timer = 0.0f;
    foc->startup_theta = 0.0f;
    foc->startup_omega = 0.0f;
    foc->angle_offset = 0.0f;
    foc->bemf_counter = 0;
    foc->adc_offset_a = ADC_OFFSET;
    foc->adc_offset_b = ADC_OFFSET;
    foc->adc_offset_c = ADC_OFFSET;

    // Initialize BLDC specifications
    foc->bldc.Rs = RS;
    foc->bldc.Ld = LS;
    foc->bldc.Lq = LS;
    foc->bldc.Ls = LS;
    foc->bldc.flux = FLUX_LINKAGE; // Starting value; measured while running (Flux_Estimate)
    foc->bldc.max_current = I_RATED;

    // Initialize motor mode
    foc->motor_mode = MOTOR_POWER_ON;

    // Initialize temporary variables
    foc->inj_n = 0;
    foc->inj_start_flag = false;
    foc->inj_fault = false;
    foc->inj_angle = 0.0f;
    foc->hfi_takeover = true;
    foc->hfi_salient = true;
    foc->hfi_pol = 0;
    foc->hfi_on = false;
    foc->hfi_control = false;
    foc->hfi_fault = false;

    // Initialize PI controllers
    PI_Init(&(foc->id_ctrl), KP_I, KI_I, TS, VD_MAX);
    PI_Init(&(foc->iq_ctrl), KP_I, KI_I, TS, V_RATED);
    // Speed loop runs in FOC_Update every SPEED_CTRL_DIVIDER periods (TS_SPEED)
    PI_Init(&(foc->speed_ctrl), KP_SPEED, KI_SPEED, TS_SPEED, I_RATED);

    // Initialize SMO
    SMO_Init(&(foc->smo), &(foc->bldc), TS);

    FOC_Precompute(foc);
}

// ---------------------------------------------------------------------------
// Select the rotation direction for the next start. speed_ref and is_ref are relative to
// it (positive = in this direction). Only accepted while the bridge is off (idle, coasted
// or faulted), because the drive is motoring only and cannot brake a spinning rotor to
// reverse it: to reverse, stop (FOC_Coast or foc_mode = FOC_IDLE), set, start again.
// Returns false, leaving the direction unchanged, if the bridge is energized.
bool FOC_Set_Direction(FOC_t *foc, BLDC_Direction_t dir)
{
    if (foc == NULL || (dir != NORMAL_DIRECTION && dir != REVERSE_DIRECTION))
    {
        return false;
    }
    if (foc->outputs_enabled)
    {
        return false; // Energized: calibration, startup or running
    }

    foc->rotor_dir = dir;
    return true;
}

// ---------------------------------------------------------------------------
// Start the motor in FOC_SPEED_CONTROL (target: foc->speed_ref) or FOC_TORQUE_CONTROL (target:
// foc->is_ref), both relative to the direction. The start from standstill is the HFI start, or
// Align -> I-f (FOC_HFI 0, or no saliency). Call with the drive stopped.
void FOC_Start(FOC_t *foc, FOC_Control_Mode_t mode)
{
    if (foc == NULL || (mode != FOC_SPEED_CONTROL && mode != FOC_TORQUE_CONTROL))
    {
        return;
    }
    foc->motor_mode = MOTOR_IDLE; // The startup state machine begins here
    foc->foc_mode = mode;
}

// Stop: the next FOC_Update turns the bridge off (outputs_enabled = false) and the motor coasts
void FOC_Stop(FOC_t *foc)
{
    if (foc != NULL)
    {
        foc->foc_mode = FOC_IDLE;
    }
}

// Without HFI the eSMO is the only angle source (HFI_PROBE decides after the first start)
FOC_FAST_CODE static bool Esmo_Only(const FOC_t *foc)
{
#if FOC_HFI
    return !foc->hfi_salient;
#else
    (void)foc;
    return true;
#endif
}

// Lowest speed reference (rad/s el) the drive holds in speed mode; a lower one stops the motor.
// 0 while HFI can hold any speed. The HFI start's saliency probe sets this, so read it after the
// first start; it follows the measured flux linkage.
float FOC_Min_Speed(const FOC_t *foc)
{
    if (foc == NULL || !Esmo_Only(foc) || foc->bldc.flux <= 0.0f)
    {
        return 0.0f;
    }
    return ESMO_MIN_SPEED_RATIO * MIN_BEMF_AMPLITUDE / foc->bldc.flux;
}

// Current limit (A) for the current references and the speed loop's output
void FOC_Set_Current_Limit(FOC_t *foc, float amps)
{
    if (foc == NULL)
    {
        return;
    }
    foc->bldc.max_current = amps;
    PI_Set_Output_Limit(&(foc->speed_ctrl), amps);
}

// ---------------------------------------------------------------------------
// Start one calibration run with the motor at standstill: MOTOR_ESTIMATION_RS (two DC levels),
// _LD or _LQ (INJECTION_FREQ sine). The injection frame is at inj_angle (0 for Rs/Ld/Lq; other
// angles for a saliency sweep). Poll FOC_Measurement_Busy, then call the matching
// FOC_BLDC_Parameter_Calculation_* (after both LD and LQ) or FOC_BLDC_Saliency_Point.
// foc->inj_fault reports an over-current abort.
void FOC_Start_Measurement(FOC_t *foc, BLDC_Mode_t which, float inj_angle)
{
    if (foc == NULL ||
        (which != MOTOR_ESTIMATION_RS && which != MOTOR_ESTIMATION_LD && which != MOTOR_ESTIMATION_LQ))
    {
        return;
    }
    foc->inj_angle = inj_angle;
    foc->foc_mode = FOC_CALIBRATION;
    foc->motor_mode = which;
    foc->inj_fault = false;
    // Rs: ramp from zero. Ld/Lq: |Z| >= Rs at 1 kHz, so CAL_CURRENT * Rs cannot exceed CAL_CURRENT
    foc->inj_ramp = (which == MOTOR_ESTIMATION_RS);
    foc->inj_v = foc->inj_ramp ? 0.0f : CAL_CURRENT * foc->bldc.Rs;
    foc->inj_start_flag = true; // Last: the ISR starts the run when it sees this
}

bool FOC_Measurement_Busy(const FOC_t *foc)
{
    return (foc != NULL) && foc->inj_start_flag;
}

// ---------------------------------------------------------------------------
// De-energize the bridge and drop out of closed loop.
// Zeroing the duty registers is what actually stops the motor: the ISR latches
// pwm_a/b/c into CCR1-3 every period regardless of mode.
FOC_FAST_CODE void FOC_Coast(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }

    // Coasting means the bridge stops switching. FOC_Update no longer runs the
    // modulator while outputs_enabled is false (it used to overwrite these zero
    // duties with 50 % on all phases, i.e. the zero vector with the bridge still
    // switching, which brakes a spinning motor through the transistors).
    // The HAL must also disable the gate driver / PWM outputs when it sees
    // outputs_enabled == false, because a 0 duty in complementary mode keeps
    // every low-side switch on.
    foc->outputs_enabled = false;
    foc->pwm_a = 0;
    foc->pwm_b = 0;
    foc->pwm_c = 0;
    foc->v_d = 0.0f;
    foc->v_q = 0.0f;
    foc->v_alpha = 0.0f;
    foc->v_beta = 0.0f;
    foc->id_ref = 0.0f;
    foc->iq_ref = 0.0f;
    foc->is_ref = 0.0f;
    foc->theta = 0.0f;
    foc->omega = 0.0f;
    foc->foc_mode = FOC_IDLE;
    foc->motor_mode = MOTOR_IDLE;
    foc->hfi_on = false; // HFI restarts from the eSMO after the next start
    foc->hfi_control = false;

    PI_Reset(&(foc->id_ctrl));
    PI_Reset(&(foc->iq_ctrl));
    PI_Reset(&(foc->speed_ctrl));
    SMO_Reset(&(foc->smo));
    Harm_Reset(foc);
}

static void FOC_Speed_Control(FOC_t *foc);
#if FOC_HFI
static void FOC_Current_Loop_HFI(FOC_t *foc);
#endif
FOC_INLINE void Current_Loop_Core(FOC_t *foc);

// ---------------------------------------------------------------------------
// Function to update FOC states
// This function should be called at the end of each PWM period
FOC_FAST_CODE void FOC_Update(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }

    // Compute parameters from measured ADC values
    foc->v_bus = (float)(foc->adc_v_bus) * ADC_2_VBUS; // Bus voltage in volts
    Calculate_Phase_Currents(foc->adc_v_a, foc->adc_v_b, foc->adc_offset_a, foc->adc_offset_b,
                             &(foc->i_a), &(foc->i_b));

    // Bus over-voltage (e.g. energy pumped back by the motor): coast and latch a fault.
    // Also ends a calibration run so the waiting task does not hang.
    if (foc->outputs_enabled && foc->v_bus > V_BUS_MAX)
    {
        FOC_Coast(foc);
        foc->inj_start_flag = false;
        foc->vbus_fault = true;
    }

    // Bus under-voltage while driving (bench 2026-10-30: a current-limited supply sagged to 7.5 V
    // and the bridge ended in the hardware over-current trip): coast and latch a fault after
    // V_BUS_UV_PERIODS consecutive low samples, so one noisy ADC sample cannot trip it.
    if (foc->outputs_enabled && foc->v_bus < V_BUS_MIN)
    {
        if (++foc->vbus_uv_cnt >= V_BUS_UV_PERIODS)
        {
            FOC_Coast(foc);
            foc->inj_start_flag = false;
            foc->vbus_uv_fault = true;
        }
    }
    else
    {
        foc->vbus_uv_cnt = 0;
    }

    // Main FOC state machine
    switch (foc->foc_mode)
    {
    case FOC_CALIBRATION: // Measure BLDC parameters
        FOC_BLDC_Parameter_Measurement_Update(foc);
        // Injection is done in a fixed frame at standstill (inj_angle: 0 except in the saliency sweep)
        foc->theta = foc->inj_angle;
        foc->omega = 0.0f;
        foc->outputs_enabled = !foc->inj_fault;
        break;

    case FOC_TORQUE_CONTROL:
    case FOC_SPEED_CONTROL:
        if (foc->foc_mode == FOC_SPEED_CONTROL && Esmo_Only(foc) &&
            foc->speed_ref * foc->bldc.flux < ESMO_MIN_SPEED_RATIO * MIN_BEMF_AMPLITUDE)
        {
            // Below FOC_Min_Speed with no HFI: stop (the eSMO cannot hold it)
            FOC_Coast(foc);
            foc->foc_mode = FOC_IDLE;
            foc->motor_mode = MOTOR_IDLE;
            break;
        }
        switch (foc->motor_mode)
        {
        case MOTOR_IDLE:
        case MOTOR_STARTUP_ALIGN:
        case MOTOR_STARTUP_RAMP:
        case MOTOR_STARTUP_TRANSITION:
        case MOTOR_STARTUP_HFI:
            // Align -> I-f ramp -> transition, or the HFI start (all current-controlled)
            FOC_BLDC_Startup_Update(foc);
            break;

        case MOTOR_RUNNING:
            // Closed-loop control

            // Derive the current references. In speed mode the speed loop drives iq_ref;
            // in torque mode MTPA splits the stator current magnitude is_ref (relative to
            // rotor_dir: positive drives in that direction, negative brakes).
            if (foc->foc_mode == FOC_TORQUE_CONTROL)
            {
                const float is = (foc->rotor_dir == NORMAL_DIRECTION) ? foc->is_ref : -foc->is_ref;
                FOC_MTPA(foc, is, &(foc->id_ref), &(foc->iq_ref));
            }
            else
            {
                FOC_Speed_Control(foc);
            }
#if FOC_HFI
            FOC_Current_Loop_HFI(foc); // eSMO, or HFI at low speed
#else
            Current_Loop_Core(foc); // eSMO only
#endif
            break;

        default:
            break; // Handle other motor modes as needed
        }
        break;

    case FOC_IDLE:
    default:
        // Safety: idle (or an invalid mode) must always de-energize the bridge.
        // The ISR latches pwm_a/b/c every period, so leaving stale duties here
        // would keep the phases driven.
        foc->pwm_a = 0;
        foc->pwm_b = 0;
        foc->pwm_c = 0;
        foc->v_d = 0.0f;
        foc->v_q = 0.0f;
        foc->outputs_enabled = false;
        foc->hfi_on = false;
        foc->hfi_control = false;
        if (foc->motor_mode != MOTOR_POWER_ON)
        {
            foc->motor_mode = MOTOR_IDLE;
        }
        break;
    }

    // Outputs disabled (idle, coast or fault): keep the duties at 0 and do not
    // run the modulator, which would otherwise produce 50 % on every phase.
    if (!foc->outputs_enabled)
    {
        foc->pwm_a = 0;
        foc->pwm_b = 0;
        foc->pwm_c = 0;
        foc->v_alpha = 0.0f;
        foc->v_beta = 0.0f;
        return;
    }

    // Inverse Park Transform with the angle chosen by the state machine.
    // The voltage computed now is applied ~1.5 periods later, by which time the
    // rotor has moved by omega * 1.5 * Ts (about 16 electrical degrees at rated
    // speed with this motor at 10 kHz), so advance the angle. omega is 0 while
    // aligning, the forced speed during the I-f ramp, and the observer's speed
    // in the transition and in closed loop.
    // theta is in [0, 2pi) and the advance is well under one turn (0.3 rad at
    // ESMO_OMEGA_MAX), so one wrap keeps the transform on its table fast path; without
    // it, every period near 0 or 2pi took the Normalize_Angle slow path.
    float theta_ipark = Wrap_Once(foc->theta + foc->ipark_adv * foc->omega);
    Inverse_Park_Core(foc->v_d, foc->v_q, theta_ipark, &(foc->v_alpha), &(foc->v_beta));

    // Space Vector PWM Modulation
    SVPWM_Core(foc->v_alpha, foc->v_beta, foc->v_bus, foc->pwm_resolution,
               &(foc->pwm_a), &(foc->pwm_b), &(foc->pwm_c));
}

// ---------------------------------------------------------------------------
// Recalculate FOC parameters after BLDC specifications update
void FOC_Recalc_Params(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }

    float Kp = foc->bldc.Ld * CURRENT_CONTROL_O_BANDWIDTH;
    float Ki = foc->bldc.Rs * CURRENT_CONTROL_O_BANDWIDTH;

    // Reset ID and IQ controller gains
    PI_Change_Params(&(foc->id_ctrl), Kp, Ki, foc->sampling_time);
    PI_Change_Params(&(foc->iq_ctrl), Kp, Ki, foc->sampling_time);

    // Update SMO BLDC specifications
    SMO_BLDC_Specs_Update(&(foc->smo), &(foc->bldc), foc->sampling_time);

    FOC_Precompute(foc);
}

// ---------------------------------------------------------------------------
FOC_FAST_CODE static void FOC_MTPA(FOC_t *foc, float Is, float *Id_ref, float *Iq_ref)
{
    if (foc == NULL || Id_ref == NULL || Iq_ref == NULL)
    {
        return;
    }

    // Non-salient (surface-PM) motor, or unusable saliency: no reluctance torque
    // to exploit, so put all current on the q-axis. (FOC_Precompute sets
    // mtpa_inv_4l to 0 for Lq - Ld <= MIN_SALIENCY_H, the default Ld == Lq.)
    if (foc->mtpa_inv_4l == 0.0f)
    {
        *Id_ref = 0.0f;
        *Iq_ref = Is;
        return;
    }

    float Is_square = Is * Is;
    const float psi = foc->bldc.flux;
    float temp = Fast_Sqrt(psi * psi + foc->mtpa_k * Is_square);
    float id = (psi - temp) * foc->mtpa_inv_4l;

    // |id| can marginally exceed |Is| through rounding; clamp so the sqrt stays real
    float iq_square = Is_square - (id * id);
    float iq = (iq_square > 0.0f) ? Fast_Sqrt(iq_square) : 0.0f;
    if (Is < 0)
        iq = -iq;

    *Id_ref = id;
    *Iq_ref = iq;
}

// ---------------------------------------------------------------------------
// Update PWM in current controlled loop (Update at the end of PWM Period)
// d/q current PI control at angle foc->theta (shared by startup and closed loop).
// Expects foc->i_alpha / i_beta to be up to date (Clarke done by the caller).
FOC_FAST_CODE static void FOC_Current_Control(FOC_t *foc)
{
    // Limit current references (Current loop control id and iq to the setpoint)
    float id_ref = CONSTRAIN(foc->id_ref, -foc->bldc.max_current, foc->bldc.max_current);
    float iq_ref = CONSTRAIN(foc->iq_ref, -foc->bldc.max_current, foc->bldc.max_current);

    // Perform Park transform to get Id and Iq
    Park_Core(foc->i_alpha, foc->i_beta, foc->theta, &(foc->i_d), &(foc->i_q));

    // Set current output limits based on bus voltage to prevent overmodulation
    float v_limit = foc->v_bus * ONE_OVER_SQRT3; // Max phase voltage in the linear range

    // Calculate errors and update PI controllers.
    // v_d is the PI output, not an accumulator: the PI already integrates.
    foc->id_ctrl.output_limit = v_limit;
    foc->v_d = PI_Step(&(foc->id_ctrl), id_ref - foc->i_d);

    // Give Vq whatever headroom Vd left inside the voltage circle.
    // Guard the argument so |v_d| > v_limit cannot produce a NaN.
    float vq_headroom_sq = v_limit * v_limit - foc->v_d * foc->v_d;
    float vq_limit = (vq_headroom_sq > 0.0f) ? Fast_Sqrt(vq_headroom_sq) : 0.0f;

    foc->iq_ctrl.output_limit = vq_limit;
    foc->v_q = PI_Step(&(foc->iq_ctrl), iq_ref - foc->i_q);
}

// Unit phasor conj(G_eff)/|G_eff| of the plant seen by a d/q-frame signal at frequency w_dq
// (signed, rad/s): G = LPF * PWM delay / (R + j*w_s*L) at the stationary frequency
// w_s = w_dq + omega (the inverse Park advance already covers the frame rotation), and
// G_eff = G / (1 + C*G) with the d/q PI C = Kp - j*Ki/w_dq already closed around it.
FOC_FAST_CODE static void Harm_Phasor(const FOC_t *foc, float w_dq, float out[2])
{
    const float ts = foc->sampling_time;
    const float w_s = w_dq + foc->omega;
    const float R = foc->bldc.Rs, L = foc->bldc.Ld;

    const float inv_den = 1.0f / (R * R + w_s * w_s * L * L);
    float pr = R * inv_den, pi = -w_s * L * inv_den; // 1 / (R + j w_s L)

    // Angles stay within +-2pi up to ESMO_OMEGA_MAX: use |angle| (trig fast path), odd sine
    float a = w_dq * HARM_DELAY_TS * ts;
    float ds, dc;
    Fast_Sin_Cos(fabsf(a), &ds, &dc); // exp(-j w_dq Td) = dc - j ds
    ds = (a < 0.0f) ? -ds : ds;
    float gr = pr * dc + pi * ds, gi = pi * dc - pr * ds;

    a = w_s * ts;
    float hs, hc;
    Fast_Sin_Cos(fabsf(a), &hs, &hc); // LPF alpha / (1 - beta exp(-j w_s Ts))
    hs = (a < 0.0f) ? -hs : hs;
    float hr = 1.0f - BETA_LPF_MEAS * hc, hi = BETA_LPF_MEAS * hs;
    float hm = ALPHA_LPF_MEAS / (hr * hr + hi * hi);
    float g2r = (gr * hr + gi * hi) * hm, g2i = (gi * hr - gr * hi) * hm;

    float cr = foc->iq_ctrl.Kp, ci = -foc->iq_ctrl.Ki / w_dq;
    float lr = 1.0f + cr * g2r - ci * g2i, li = cr * g2i + ci * g2r; // 1 + C G
    // G_eff = G conj(1 + C G) / |1 + C G|^2. Only its direction is used, so the
    // positive real |1 + C G|^2 is dropped instead of divided out.
    float er = g2r * lr + g2i * li, ei = g2i * lr - g2r * li;

    float m = Fast_Sqrt(er * er + ei * ei);
    if (m < 1e-9f)
    {
        out[0] = 1.0f;
        out[1] = 0.0f;
        return;
    }
    const float inv_m = 1.0f / m;
    out[0] = er * inv_m;
    out[1] = -ei * inv_m;
}

// 6x harmonic compensation, after the PI in closed loop (see HARM_* in foc_lib.h).
// The current error is rotated into the +6 and -6 frames and integrated there with the
// phase of conj(G_eff), so each integrator drives its harmonic of the error to zero; the
// outputs are rotated back and added to v_d / v_q.
FOC_FAST_CODE static void FOC_Harmonic_Comp(FOC_t *foc)
{
    if (!HARM_COMP || fabsf(foc->omega) < HARM_OMEGA_MIN)
    {
        Harm_Reset(foc);
        return;
    }

    // Refresh one phasor per slot (each at 1 kHz), at 1/4 and 3/4 of the cycle: never in the
    // same period as each other or as the speed loop (speed_div == 0; both counters start at
    // the handover). Keeps the worst-case ISR time down.
    const uint16_t n_div = (uint16_t)(PWM_FREQ / 1000.0f);
    if (foc->harm_div == n_div / 4)
    {
        Harm_Phasor(foc, 6.0f * foc->omega, foc->harm_rp);
    }
    else if (foc->harm_div == (3 * n_div) / 4)
    {
        Harm_Phasor(foc, -6.0f * foc->omega, foc->harm_rn);
    }
    if (++foc->harm_div >= n_div)
    {
        foc->harm_div = 0;
    }

    // 6*theta wrapped into [0, 2pi) here (theta is in [0, 2pi)): keeps the trig fast path
    float th6 = 6.0f * foc->theta;
    th6 -= TWO_PI * (float)(int)(th6 * (1.0f / TWO_PI));
    float s, c;
    Fast_Sin_Cos(th6, &s, &c);
    const float ed = foc->id_ref - foc->i_d, eq = foc->iq_ref - foc->i_q;
    const float k = foc->harm_k;

    // +6 frame: e * exp(-j6theta); -6 frame: e * exp(+j6theta)
    float px = ed * c + eq * s, py = eq * c - ed * s;
    float nx = ed * c - eq * s, ny = eq * c + ed * s;
    foc->harm_p[0] += k * (foc->harm_rp[0] * px - foc->harm_rp[1] * py);
    foc->harm_p[1] += k * (foc->harm_rp[0] * py + foc->harm_rp[1] * px);
    foc->harm_n[0] += k * (foc->harm_rn[0] * nx - foc->harm_rn[1] * ny);
    foc->harm_n[1] += k * (foc->harm_rn[0] * ny + foc->harm_rn[1] * nx);

    // Limit each harmonic voltage vector
    float *h[2] = {foc->harm_p, foc->harm_n};
    for (int i = 0; i < 2; i++)
    {
        float m2 = h[i][0] * h[i][0] + h[i][1] * h[i][1];
        if (m2 > HARM_V_MAX * HARM_V_MAX)
        {
            float sc = HARM_V_MAX / Fast_Sqrt(m2);
            h[i][0] *= sc;
            h[i][1] *= sc;
        }
    }

    // Back to d/q: harm_p * exp(+j6theta) + harm_n * exp(-j6theta)
    foc->v_d += foc->harm_p[0] * c - foc->harm_p[1] * s + foc->harm_n[0] * c + foc->harm_n[1] * s;
    foc->v_q += foc->harm_p[0] * s + foc->harm_p[1] * c - foc->harm_n[0] * s + foc->harm_n[1] * c;
}

// Flux linkage from the eSMO's back-EMF while it is in control and locked: |E| / (g * |omega|), g
// the eSMO's magnitude gain (FLUX_EST_G*). Only where its back-EMF LPF tracks the speed (between
// the cutoff floor and ceiling, where g holds), above FLUX_EST_OMEGA_MIN, and for values within
// FLUX_LINKAGE / FLUX_EST_RANGE .. * FLUX_EST_RANGE (an unknown motor; the lock and LPF reject transients). Used by MTPA and the startup plausibility check.
FOC_INLINE void Flux_Estimate(FOC_t *foc)
{
    const float w = fabsf(foc->smo.omega_est);
    const float x = foc->smo.esmo_ratio_ts * w; // LPF cutoff * Ts
    if (!foc->smo.locked || w < FLUX_EST_OMEGA_MIN || x <= foc->smo.esmo_x_lo || x >= ESMO_WC_MAX_TS)
    {
        return;
    }
    const float wts = w * foc->sampling_time;
    const float g = FLUX_EST_G0 + wts * (FLUX_EST_G1 + FLUX_EST_G2 * wts);
    const float psi = foc->smo.emf_magnitude / (g * w);
    if (psi > FLUX_LINKAGE / FLUX_EST_RANGE && psi < FLUX_LINKAGE * FLUX_EST_RANGE)
    {
        foc->bldc.flux += FLUX_EST_ALPHA * (psi - foc->bldc.flux);
    }
}

// Closed-loop step, inlined into FOC_Update
FOC_INLINE void Current_Loop_Core(FOC_t *foc)
{
    // Refresh the measured currents in the stationary frame BEFORE running the
    // observer, so the SMO sees this period's samples rather than the previous one's.
    Clarke_Transform(foc->i_a, foc->i_b, &(foc->i_alpha), &(foc->i_beta));

    // Estimate rotor position and speed using SMO
    if (SMO_Update(&(foc->smo), foc->v_bus, foc->v_alpha, foc->v_beta, foc->i_alpha, foc->i_beta) == false)
    {
        // Lost the observer: disable the outputs and fault out. Do NOT fall back
        // to MOTOR_IDLE alone, which would immediately re-enter the startup.
        FOC_Coast(foc);
        return;
    }

    // Closed loop: the transforms use the observer's angle
    foc->theta = foc->smo.theta_est;
    foc->omega = foc->smo.omega_est;
    Flux_Estimate(foc);

    FOC_Current_Control(foc);
    FOC_Harmonic_Comp(foc);
}


// ---------------------------------------------------------------------------
// Startup procedure for BLDC motor (Open-Loop)
// Move x toward target by at most step
static inline float Slew_To(float x, float target, float step)
{
    if (x < target - step)
    {
        return x + step;
    }
    if (x > target + step)
    {
        return x - step;
    }
    return target;
}

// Wrap an angle difference from (-2pi, 2pi) into [-pi, pi]
static inline float Wrap_Pm_Pi(float a)
{
    if (a > PI)
    {
        a -= TWO_PI;
    }
    else if (a < -PI)
    {
        a += TWO_PI;
    }
    return a;
}

#if FOC_HFI
// ---------------------------------------------------------------------------
// HFI tracker (see HFI_STEP_CURRENT in foc_lib.h). Started at the eSMO's angle and speed while running,
// at 0 from standstill (MOTOR_STARTUP_HFI).
FOC_FAST_CODE static void HFI_Start(FOC_t *foc, float theta, float omega)
{
    foc->hfi_on = true;
    foc->hfi_cnt = 0;
    foc->hfi_idx = 0;
    foc->hfi_fault_cnt = 0;
    foc->hfi_theta = theta;
    foc->hfi_omega = omega;
    foc->hfi_probe = 0.0f;
    for (int j = 0; j < 4; j++)
    {
        foc->hfi_par[j] = 0.0f;
        foc->hfi_perp[j] = 0.0f;
    }
}

// One tracker step on this period's current sample: demodulate, then advance the PLL.
// Returns false when the HF response or the track has been bad for HFI_FAULT_PERIODS.
// Command k is applied during period k+1 (mid-period latch: partly k+2) and shows in the
// current step i[k+2] - i[k+1]; the 4-period phasors absorb that timing.
FOC_FAST_CODE static bool HFI_Track(FOC_t *foc)
{
    // Unfiltered currents: the measurement LPF attenuates the PWM_FREQ/4 component
    float ia = (float)((int32_t)foc->adc_v_a - (int32_t)foc->adc_offset_a) * ADC_2_CURRENT;
    float ib = (float)((int32_t)foc->adc_v_b - (int32_t)foc->adc_offset_b) * ADC_2_CURRENT;
    float i_al, i_be;
    Clarke_Transform(ia, ib, &i_al, &i_be);

    float w = foc->hfi_omega;
    if (foc->hfi_cnt > HFI_DELAY) // The axis history is filled
    {
        float d_al = i_al - foc->hfi_ia_h;
        float d_be = i_be - foc->hfi_ib_h;
        float s, c;
        Fast_Sin_Cos(foc->hfi_phi_h[HFI_DELAY - 1], &s, &c);
        const uint32_t m = foc->hfi_idx;
        foc->hfi_par[m] = d_al * c + d_be * s;
        foc->hfi_perp[m] = -d_al * s + d_be * c;
    }
    if (foc->hfi_cnt > HFI_DELAY + 4u) // All 4 slots hold real steps
    {
        // Phasors with the reference e^(-j pi m / 2) = 1, -j, -1, j for slots m = 0..3
        const float *p = foc->hfi_par, *q = foc->hfi_perp;
        float zp_re = p[0] - p[2], zp_im = p[3] - p[1];
        float zq_re = q[0] - q[2], zq_im = q[3] - q[1];
        float zp2 = zp_re * zp_re + zp_im * zp_im;
        foc->hfi_zp = Fast_Sqrt(zp2);

        // eps = HFI_K * sin(2 e) -> e ~ eps / (2 HFI_K)
        float e = (zp2 > 1e-6f) ? (zq_re * zp_re + zq_im * zp_im) / zp2 * (0.5f / HFI_K) : 0.0f;
        foc->hfi_err = e;

        if (foc->hfi_zp < HFI_ZP_MIN || fabsf(e) > HFI_ERR_MAX)
        {
            if (++foc->hfi_fault_cnt >= HFI_FAULT_PERIODS)
            {
                return false;
            }
        }
        else
        {
            foc->hfi_fault_cnt = 0;
        }

        // PLL (forward Euler, integrator = speed estimate); frozen during the saliency probe
        if (foc->hfi_probe == 0.0f)
        {
            e = CONSTRAIN(e, -HFI_ERR_MAX, HFI_ERR_MAX);
            foc->hfi_omega = CONSTRAIN(foc->hfi_omega + HFI_PLL_KI * foc->sampling_time * e,
                                       -ESMO_OMEGA_MAX, ESMO_OMEGA_MAX);
            w = foc->hfi_omega + HFI_PLL_KP * e;
        }
        else
        {
            w = 0.0f;
        }
    }
    foc->hfi_theta = Wrap_Once(foc->hfi_theta + w * foc->sampling_time);

    foc->hfi_ia_h = i_al;
    foc->hfi_ib_h = i_be;
    if (foc->hfi_cnt < HFI_WARMUP)
    {
        foc->hfi_cnt++;
    }
    return true;
}

// Add the square wave along hfi_theta + hfi_probe (after the current controller) and remember
// the alpha-beta axis it is applied on (the inverse Park adds ipark_adv * omega to every voltage)
FOC_FAST_CODE static void HFI_Inject(FOC_t *foc)
{
    const float axis = Wrap_Once(foc->hfi_theta + foc->hfi_probe); // hfi_probe: 0 or HFI_PROBE_ANGLE
    float delta = axis - foc->theta; // Both in [0, 2pi)
    if (delta < 0.0f)
    {
        delta += TWO_PI;
    }
    float s, c;
    Fast_Sin_Cos(delta, &s, &c);
    const float v = (foc->hfi_idx < 2u) ? foc->hfi_v : -foc->hfi_v;
    foc->v_d += v * c;
    foc->v_q += v * s;

    for (int j = HFI_DELAY - 1; j > 0; j--)
    {
        foc->hfi_phi_h[j] = foc->hfi_phi_h[j - 1];
    }
    foc->hfi_phi_h[0] = Wrap_Once(axis + foc->ipark_adv * foc->omega);
    foc->hfi_idx = (foc->hfi_idx + 1u) & 3u;
}

// Closed-loop step with the eSMO <-> HFI switch (MOTOR_RUNNING).
// The eSMO runs every period; its lock flag only matters while it is in control.
FOC_FAST_CODE static void FOC_Current_Loop_HFI(FOC_t *foc)
{
    Clarke_Transform(foc->i_a, foc->i_b, &(foc->i_alpha), &(foc->i_beta));
    const bool smo_locked = SMO_Update(&(foc->smo), foc->v_bus, foc->v_alpha, foc->v_beta,
                                       foc->i_alpha, foc->i_beta);

    if (foc->hfi_on && !HFI_Track(foc))
    {
        if (foc->hfi_control)
        {
            foc->hfi_fault = true;
            FOC_Coast(foc); // No angle source left at this speed
            return;
        }
        HFI_Start(foc, foc->smo.theta_est, foc->smo.omega_est); // Not in control: restart from the eSMO
    }

    if (!foc->hfi_control)
    {
        if (!smo_locked)
        {
            FOC_Coast(foc); // As in Current_Loop_Core
            return;
        }
        const float w = fabsf(foc->smo.omega_est);
        if (!foc->hfi_on && foc->hfi_salient && w < HFI_OMEGA_ON)
        {
            HFI_Start(foc, foc->smo.theta_est, foc->smo.omega_est);
        }
        else if (foc->hfi_on && w > 1.1f * HFI_OMEGA_ON)
        {
            foc->hfi_on = false;
        }
        else if (foc->hfi_takeover && foc->hfi_on && foc->hfi_cnt >= HFI_WARMUP && w < HFI_OMEGA_LOW)
        {
            if (fabsf(Wrap_Pm_Pi(foc->smo.theta_est - foc->hfi_theta)) < HFI_AGREE_ANGLE)
            {
                // Take over; the offset keeps the control angle continuous and then decays
                foc->angle_offset = Wrap_Pm_Pi(Wrap_Once(foc->smo.theta_est + foc->angle_offset) - foc->hfi_theta);
                foc->hfi_control = true;
                foc->hfi_agree_cnt = 0;
                foc->hfi_rev_cnt = 0;
                foc->hfi_vbal_cnt = 0;
            }
            else
            {
                // The tracker disagrees with the still-trusted eSMO (e.g. locked on the wrong
                // pole): re-seed it rather than hand it the motor
                HFI_Start(foc, foc->smo.theta_est, foc->smo.omega_est);
            }
        }
    }
    else
    {
        // Wrong-pole lock: the rotor turns against rotor_dir (see HFI_REVERSE_OMEGA)
        const float dir = (foc->rotor_dir == NORMAL_DIRECTION) ? 1.0f : -1.0f;
        foc->hfi_rev_cnt = (dir * foc->hfi_omega < -HFI_REVERSE_OMEGA) ? foc->hfi_rev_cnt + 1u : 0u;
        if (foc->hfi_rev_cnt >= HFI_REVERSE_PERIODS)
        {
            foc->hfi_fault = true;
            FOC_Coast(foc);
            return;
        }

        // Voltage balance (see HFI_VBAL_RATIO); last period's voltage and current
        const float i_mag = Fast_Sqrt(foc->i_d * foc->i_d + foc->i_q * foc->i_q);
        const float v_expl = foc->bldc.Rs * i_mag +
                             fabsf(foc->hfi_omega) * (foc->bldc.Ls * i_mag + foc->bldc.flux) +
                             HFI_VBAL_RATIO * ONE_OVER_SQRT3 * foc->v_bus;
        const bool v_high = foc->v_d * foc->v_d + foc->v_q * foc->v_q > v_expl * v_expl;
        const bool w_high = fabsf(foc->hfi_omega) > HFI_OVERSPEED * HFI_OMEGA_UP;
        foc->hfi_vbal_cnt = (v_high || w_high) ? foc->hfi_vbal_cnt + 1u : 0u;
        if (foc->hfi_vbal_cnt >= HFI_VBAL_PERIODS)
        {
            foc->hfi_fault = true;
            FOC_Coast(foc);
            return;
        }

        // Back to the eSMO once it is locked and agrees with the tracker above HFI_OMEGA_UP
        const float d = Wrap_Pm_Pi(foc->smo.theta_est - foc->hfi_theta);
        const bool agree = smo_locked && fabsf(foc->hfi_omega) > HFI_OMEGA_UP &&
                           fabsf(d) < HFI_AGREE_ANGLE &&
                           fabsf(foc->smo.omega_est - foc->hfi_omega) < 0.2f * fabsf(foc->hfi_omega);
        foc->hfi_agree_cnt = agree ? foc->hfi_agree_cnt + 1u : 0u;
        if (foc->hfi_agree_cnt >= HFI_AGREE_PERIODS)
        {
            foc->angle_offset = Wrap_Pm_Pi(Wrap_Once(foc->hfi_theta + foc->angle_offset) - foc->smo.theta_est);
            foc->hfi_control = false;
        }
    }

    // Active source plus the decaying switch offset (offset in [-pi, pi]: one wrap)
    foc->angle_offset = Slew_To(foc->angle_offset, 0.0f, ANGLE_BLEND_RATE * foc->sampling_time);
    if (foc->hfi_control)
    {
        foc->theta = Wrap_Once(foc->hfi_theta + foc->angle_offset);
        foc->omega = foc->hfi_omega;
    }
    else
    {
        foc->theta = Wrap_Once(foc->smo.theta_est + foc->angle_offset);
        foc->omega = foc->smo.omega_est;
        Flux_Estimate(foc);
    }

    FOC_Current_Control(foc);
    FOC_Harmonic_Comp(foc);
    if (foc->hfi_on)
    {
        HFI_Inject(foc);
    }
}
#endif // FOC_HFI

// Speed loop (FOC_SPEED_CONTROL, MOTOR_RUNNING), every SPEED_CTRL_DIVIDER periods.
// Works in the rotation-direction frame (speed_ref, speed_ref_ramp, error and torque all
// positive = in rotor_dir), so a single clamp keeps it motoring only: the output and the
// integrator never go negative, so the drive never brakes and returns no energy to the bus.
// When speed_ref drops, the motor slows by friction and load; a negative speed_ref is 0.
FOC_FAST_CODE static void FOC_Speed_Control(FOC_t *foc)
{
    if (++foc->speed_div < SPEED_CTRL_DIVIDER)
    {
        return;
    }
    foc->speed_div = 0;

    const float dir = (foc->rotor_dir == NORMAL_DIRECTION) ? 1.0f : -1.0f;
    const float target = (foc->speed_ref > 0.0f) ? foc->speed_ref : 0.0f;
    foc->speed_ref_ramp = Slew_To(foc->speed_ref_ramp, target, SPEED_REF_SLEW * TS_SPEED);

    float iq = PI_Step(&(foc->speed_ctrl), foc->speed_ref_ramp - dir * foc->omega); // Active estimator (eSMO or HFI)
    if (iq < 0.0f)
    {
        iq = 0.0f;
    }
    if (foc->speed_ctrl.integral < 0.0f)
    {
        foc->speed_ctrl.integral = 0.0f;
    }

    foc->iq_ref = dir * iq;
    foc->id_ref = 0.0f;
}

// Startup: Align -> I-f ramp -> Transition -> Run.
//
// All stages are current-controlled: the d/q current PI loops run from the
// first period, so there is no voltage step and no integrator reset at the
// handover, and the current never exceeds what is commanded.
//
//  ALIGN       id -> I_ALIGN at theta = 0 pulls the rotor's d-axis to 0.
//  RAMP (I-f)  The forced angle accelerates at STARTUP_ACCEL up to OMEGA_FORCE,
//              with iq = I_STARTUP. The rotor follows with a load angle delta
//              (torque = Kt * I_STARTUP * cos(delta)), and the eSMO runs alongside.
//  TRANSITION  Once the observer is locked AND agrees with the forced speed, the
//              transforms switch to the observer's angle PLUS an offset that is
//              initially (forced - observer), so the current vector does not jump.
//              The offset then decays to 0 at ANGLE_BLEND_RATE, smoothly turning
//              the current from the forced frame onto the rotor's q-axis.
//  RUN         Bumpless: the speed controller's integrator is preloaded with the
//              startup current, so its output continues from where startup left.
//
// Faults (observer never locks at OMEGA_FORCE, or loses lock during the
// transition) coast the motor instead of looping back into a restart.
FOC_FAST_CODE static void FOC_BLDC_Startup_Update(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }

    const float ts = foc->sampling_time;
    const float dir = (foc->rotor_dir == NORMAL_DIRECTION) ? 1.0f : -1.0f;
    const float i_step = I_REF_SLEW * ts;

    switch (foc->motor_mode)
    {
    case MOTOR_IDLE:
        // Start from a clean state: no stale integrators or observer state
        foc->startup_timer = 0.0f;
        foc->startup_theta = 0.0f;
        foc->startup_omega = 0.0f;
        foc->angle_offset = 0.0f;
        foc->bemf_counter = 0;
        foc->hfi_on = false;
        foc->hfi_control = false;
        foc->theta = 0.0f;
        foc->omega = 0.0f;
        foc->id_ref = 0.0f;
        foc->iq_ref = 0.0f;
        foc->v_d = 0.0f;
        foc->v_q = 0.0f;
        PI_Reset(&(foc->id_ctrl));
        PI_Reset(&(foc->iq_ctrl));
        SMO_Reset(&(foc->smo));
        Harm_Reset(foc);
        foc->outputs_enabled = true;
        foc->motor_mode = MOTOR_STARTUP_ALIGN;
#if FOC_HFI
        if (foc->hfi_takeover) // The shadow test starts with Align -> I-f
        {
            HFI_Start(foc, 0.0f, 0.0f);
            foc->hfi_stage = 0;
            foc->hfi_pol = 0;
            foc->hfi_pol_n = 0;
            foc->hfi_zp_pos = 0.0f;
            foc->hfi_zp_neg = 0.0f;
            foc->hfi_probe_e = 0.0f;
            foc->hfi_salient = true;
            foc->motor_mode = MOTOR_STARTUP_HFI;
        }
#endif
        break;

#if FOC_HFI
    case MOTOR_STARTUP_HFI:
    {
        // See HFI_LOCK_TIME in foc_lib.h: lock, saliency probe, +pulse, zero, -pulse, zero, decide
        if (!HFI_Track(foc))
        {
            foc->hfi_fault = true;
            FOC_Coast(foc);
            return;
        }
        foc->startup_timer += ts;

        const uint8_t stage = foc->hfi_stage;
        const bool pulse = (stage == 2u || stage == 4u);
        const float id_target = (stage == 2u) ? HFI_POL_CURRENT : (stage == 4u) ? -HFI_POL_CURRENT : 0.0f;
        const float t_end = (stage == 0u) ? HFI_LOCK_TIME : (stage == 1u) ? HFI_PROBE_TIME : pulse ? HFI_POL_TIME : HFI_POL_ZERO;

        // Second half of the probe: tracker error; of each pulse: |Zp| (averaged at the end)
        if ((stage == 1u || pulse) && foc->startup_timer >= 0.5f * t_end)
        {
            if (stage == 1u)
            {
                foc->hfi_probe_e += foc->hfi_err;
            }
            else if (stage == 2u)
            {
                foc->hfi_zp_pos += foc->hfi_zp;
            }
            else
            {
                foc->hfi_zp_neg += foc->hfi_zp;
            }
            foc->hfi_pol_n++;
        }

        // Lock and probe: follow the tracker (no current). Pulses: the angle locked before them.
        foc->theta = (stage <= 1u) ? foc->hfi_theta : foc->startup_theta;
        foc->omega = 0.0f;
        foc->id_ref = Slew_To(foc->id_ref, id_target, HFI_POL_SLEW * ts);
        foc->iq_ref = 0.0f;
        Clarke_Transform(foc->i_a, foc->i_b, &(foc->i_alpha), &(foc->i_beta));
        FOC_Current_Control(foc);
        HFI_Inject(foc);

        if (foc->startup_timer < t_end)
        {
            break;
        }
        foc->startup_timer = 0.0f;
        const float n = (foc->hfi_pol_n > 0u) ? (float)foc->hfi_pol_n : 1.0f;
        foc->hfi_pol_n = 0;
        if (stage == 0u)
        {
            foc->hfi_probe = HFI_PROBE_ANGLE; // Freezes the PLL and offsets the injection axis
        }
        else if (stage == 1u)
        {
            foc->hfi_probe = 0.0f;
            foc->hfi_fault_cnt = 0; // The deliberate probe error is not a fault
            foc->hfi_probe_e /= n;
            foc->startup_theta = foc->hfi_theta;
            // No saliency: the tracker reads ~0 at any angle. Do not pulse at an unknown angle.
            if (foc->hfi_probe_e > -HFI_PROBE_MIN)
            {
                foc->hfi_salient = false; // Nor let the tracker take over later in this run
                foc->hfi_pol = 0;
                foc->hfi_on = false;
                foc->motor_mode = MOTOR_STARTUP_ALIGN;
                break;
            }
        }
        else if (stage == 2u)
        {
            foc->hfi_zp_pos /= n;
        }
        else if (stage == 4u)
        {
            foc->hfi_zp_neg /= n;
        }
        if (stage < 5u)
        {
            foc->hfi_stage = stage + 1u;
            break;
        }

        // Decide. The pulse with the magnet's flux gives the larger HF response.
        const float pos = foc->hfi_zp_pos, neg = foc->hfi_zp_neg;
        const float hi = (pos > neg) ? pos : neg;
        if (hi - ((pos > neg) ? neg : pos) < HFI_POL_MARGIN * hi)
        {
            // Not trusted: the usual Align -> I-f start (the rotor has not moved)
            foc->hfi_pol = 0;
            foc->hfi_on = false;
            foc->motor_mode = MOTOR_STARTUP_ALIGN;
            break;
        }
        if (neg > pos)
        {
            // On the S end: turn the tracker (and its history) by pi
            foc->hfi_pol = -1;
            foc->hfi_theta = Wrap_Once(foc->hfi_theta + PI);
            for (int j = 0; j < (int)HFI_DELAY; j++)
            {
                foc->hfi_phi_h[j] = Wrap_Once(foc->hfi_phi_h[j] + PI);
            }
            for (int j = 0; j < 4; j++)
            {
                foc->hfi_par[j] = -foc->hfi_par[j];
                foc->hfi_perp[j] = -foc->hfi_perp[j];
            }
        }
        else
        {
            foc->hfi_pol = 1;
        }

        // Run on HFI from zero speed; the eSMO starts fresh and takes over above HFI_OMEGA_UP
        SMO_Reset(&(foc->smo));
        foc->theta = foc->hfi_theta;
        foc->hfi_control = true;
        foc->angle_offset = 0.0f;
        foc->hfi_agree_cnt = 0;
        foc->hfi_rev_cnt = 0;
        foc->speed_ctrl.integral = 0.0f;
        foc->speed_ref_ramp = 0.0f;
        foc->speed_div = 0;
        foc->motor_mode = MOTOR_RUNNING;
        break;
    }
#endif // FOC_HFI

    case MOTOR_STARTUP_ALIGN:
        foc->startup_timer += ts;
        foc->theta = 0.0f;
        foc->omega = 0.0f;
        foc->id_ref = Slew_To(foc->id_ref, I_ALIGN, i_step);
        foc->iq_ref = 0.0f;

        Clarke_Transform(foc->i_a, foc->i_b, &(foc->i_alpha), &(foc->i_beta));
        FOC_Current_Control(foc);

        if (foc->startup_timer >= ALIGN_TIME)
        {
            foc->startup_timer = 0.0f;
            foc->bemf_counter = 0;
            foc->motor_mode = MOTOR_STARTUP_RAMP;
        }
        break;

    case MOTOR_STARTUP_RAMP:
    {
        // Forced angle: accelerate up to OMEGA_FORCE, then hold that speed
        foc->startup_omega = CONSTRAIN(foc->startup_omega + dir * STARTUP_ACCEL * ts,
                                       -OMEGA_FORCE, OMEGA_FORCE);
        foc->startup_theta += foc->startup_omega * ts;
        Normalize_Angle(&(foc->startup_theta));
        foc->theta = foc->startup_theta;
        foc->omega = foc->startup_omega;

        // Turn the current vector from the alignment d-axis onto the q-axis
        foc->id_ref = Slew_To(foc->id_ref, 0.0f, i_step);
        foc->iq_ref = Slew_To(foc->iq_ref, dir * I_STARTUP, i_step);

        // Observer runs alongside (it never drives the transforms here)
        Clarke_Transform(foc->i_a, foc->i_b, &(foc->i_alpha), &(foc->i_beta));
        bool locked = SMO_Update(&(foc->smo), foc->v_bus, foc->v_alpha, foc->v_beta,
                                 foc->i_alpha, foc->i_beta);
        FOC_Current_Control(foc);

        // Count consecutive periods where the observer is locked, its speed agrees
        // with the forced speed, AND its back-EMF is physically plausible for that
        // speed. The last check matters: with the rotor stalled, the rotating
        // current vector itself produces a small apparent EMF that the PLL can lock
        // onto at exactly the forced speed (seen in simulation: 0.1 V observed vs
        // 1.1 V expected at 300 rad/s). A turning rotor cannot fail this check.
        bool agree = fabsf(foc->smo.omega_est - foc->startup_omega) <
                     LOCK_SPEED_TOL * fabsf(foc->startup_omega);
        bool plausible = foc->smo.emf_magnitude >=
                         EMF_PLAUSIBLE_RATIO * fabsf(foc->startup_omega) * foc->bldc.flux;
        foc->bemf_counter = (locked && agree && plausible) ? foc->bemf_counter + 1 : 0;

        if (fabsf(foc->startup_omega) >= OMEGA_FORCE)
        {
            foc->startup_timer += ts;
            if (foc->bemf_counter >= BEMF_LOCK_COUNT)
            {
                // Start from the forced angle: no jump in the current vector
                foc->angle_offset = Wrap_Pm_Pi(foc->startup_theta - foc->smo.theta_est);
                foc->startup_timer = 0.0f;
                foc->motor_mode = MOTOR_STARTUP_TRANSITION;
            }
            else if (foc->startup_timer > LOCK_TIMEOUT)
            {
                FOC_Coast(foc); // Observer never locked: stalled or wrong parameters
            }
        }
        break;
    }

    case MOTOR_STARTUP_TRANSITION:
        Clarke_Transform(foc->i_a, foc->i_b, &(foc->i_alpha), &(foc->i_beta));
        if (!SMO_Update(&(foc->smo), foc->v_bus, foc->v_alpha, foc->v_beta,
                        foc->i_alpha, foc->i_beta))
        {
            FOC_Coast(foc);
            return;
        }

        // Observer angle plus a decaying offset (theta_est in [0, 2pi), offset in
        // [-pi, pi], so one wrap brings the sum back into [0, 2pi))
        foc->angle_offset = Slew_To(foc->angle_offset, 0.0f, ANGLE_BLEND_RATE * ts);
        foc->theta = foc->smo.theta_est + foc->angle_offset;
        if (foc->theta >= TWO_PI)
        {
            foc->theta -= TWO_PI;
        }
        else if (foc->theta < 0.0f)
        {
            foc->theta += TWO_PI;
        }
        foc->omega = foc->smo.omega_est;

        foc->id_ref = Slew_To(foc->id_ref, 0.0f, i_step);
        foc->iq_ref = dir * I_STARTUP;
        FOC_Current_Control(foc);

        if (foc->angle_offset == 0.0f)
        {
            // Bumpless handover: the speed loop continues from the startup current
            // (integrator in the rotation-direction frame, see FOC_Speed_Control) and its
            // reference ramp starts at the current speed.
            // (In torque mode, MOTOR_RUNNING applies MTPA(is_ref): set is_ref to
            // about I_STARTUP before starting, or ramp it, to avoid a torque step.)
            // (No printf here: this runs in the ADC ISR. The application logs the handover.)
            foc->speed_ctrl.integral = dir * foc->iq_ref;
            foc->speed_ref_ramp = dir * foc->smo.omega_est;
            foc->speed_div = 0;
            foc->motor_mode = MOTOR_RUNNING;
        }
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// BLDC resistance calculation after measurement
void FOC_BLDC_Parameter_Calculation_Resistance(FOC_t *foc)
{
    float mean_vd = 0.0f, mean_id = 0.0f;

    if (foc == NULL)
    {
        return;
    }

    for (int i = 0; i < INJECTION_ROUNDS; i++)
    {
        mean_vd += foc->vd_inj_buff[i];
        mean_id += foc->id_inj_buff[i];
    }
    mean_vd /= INJECTION_ROUNDS;
    mean_id /= INJECTION_ROUNDS;

    // Reject a failed/aborted injection instead of writing inf/NaN into bldc,
    // which would propagate into the SMO coefficients via FOC_Recalc_Params.
    if (fabsf(mean_id) < 1e-7f)
    {
        printf("\r\nInjection failed: mean_vd = %f, mean_id = %f\r\n", mean_vd, mean_id);
        return; // Keep the previous Rs
    }

    // Two-point Rs: the buffers hold the high DC level, the sums the low one. The slope
    // cancels the inverter's fixed voltage error, which a single point (V/I) includes.
    float low_vd = foc->rs_low_vd_sum / INJECTION_ROUNDS;
    float low_id = foc->rs_low_id_sum / INJECTION_ROUNDS;
    float delta_id = mean_id - low_id;
    printf("  Rs points: %f V -> %f A, %f V -> %f A (single-point Rs %f)\r\n",
           low_vd, low_id, mean_vd, mean_id, fabsf(mean_vd / mean_id));
    if (fabsf(delta_id) < 0.05f) // ~3 LSB: the two levels are not distinguishable
    {
        printf("\r\nRs two-point failed: delta_id = %f\r\n", delta_id);
        return; // Keep the previous Rs
    }

    float rs = fabsf((mean_vd - low_vd) / delta_id);
    if (rs < CAL_RS_MIN || rs > CAL_RS_MAX)
    {
        printf("\r\nRs %f implausible: keeping %f\r\n", rs, foc->bldc.Rs);
        return;
    }
    foc->bldc.Rs = rs;
}

// ---------------------------------------------------------------------------
// Single-frequency DFT of an injection buffer at INJECTION_FREQ, mean removed:
// amplitude-scaled cosine (c) and sine (s) parts
static void Injection_Phasor(const float *buf, float sampling_time, float *c, float *s)
{
    float mean = 0.0f;
    for (int i = 0; i < INJECTION_ROUNDS; i++)
    {
        mean += buf[i];
    }
    mean /= INJECTION_ROUNDS;

    float sc = 0.0f, ss = 0.0f;
    for (int i = 0; i < INJECTION_ROUNDS; i++)
    {
        float angle = INJECTION_OMEGA * i * sampling_time;
        float x = buf[i] - mean;
        sc += x * Fast_Cos(angle);
        ss += x * Fast_Sin(angle);
    }
    *c = sc * (2.0f / INJECTION_ROUNDS);
    *s = ss * (2.0f / INJECTION_ROUNDS);
}

// ---------------------------------------------------------------------------
// BLDC inductance calculation after measurement
void FOC_BLDC_Parameter_Calculation_Inductance(FOC_t *foc)
{
    float Vc_d, Vs_d, Ic_d, Is_d, Vc_q, Vs_q, Ic_q, Is_q;

    if (foc == NULL)
    {
        return;
    }

    Injection_Phasor(foc->vd_inj_buff, foc->sampling_time, &Vc_d, &Vs_d);
    Injection_Phasor(foc->id_inj_buff, foc->sampling_time, &Ic_d, &Is_d);
    Injection_Phasor(foc->vq_inj_buff, foc->sampling_time, &Vc_q, &Vs_q);
    Injection_Phasor(foc->iq_inj_buff, foc->sampling_time, &Ic_q, &Is_q);

    // V & I amplitude
    float Vd_mag = Fast_Sqrt(Vc_d * Vc_d + Vs_d * Vs_d);
    float Id_mag = Fast_Sqrt(Ic_d * Ic_d + Is_d * Is_d);
    float Vq_mag = Fast_Sqrt(Vc_q * Vc_q + Vs_q * Vs_q);
    float Iq_mag = Fast_Sqrt(Ic_q * Ic_q + Is_q * Is_q);

    // (phi = arctan(Vs/Vc) - arctan(Is/Ic))
    float phi_d = Fast_Atan2(Vs_d, Vc_d) - Fast_Atan2(Is_d, Ic_d);
    float phi_q = Fast_Atan2(Vs_q, Vc_q) - Fast_Atan2(Is_q, Ic_q);

    // Reject a failed/aborted injection instead of writing inf/NaN into bldc
    if (Id_mag < 1e-6f || Iq_mag < 1e-6f)
    {
        printf("\r\nInjection failed: Id_mag = %f, Iq_mag = %f\r\n", Id_mag, Iq_mag);
        return; // Keep the previous Ld/Lq/Ls
    }

    // Impedansi & parameters
    float Zd_mag = Vd_mag / Id_mag;
    float Zq_mag = Vq_mag / Iq_mag;

    // Estimate Ld and Lq. sinf() is used rather than the interpolated table:
    // phi may be negative, and this runs once outside the control ISR.
    float Ld_est = (Zd_mag * sinf(phi_d)) / INJECTION_OMEGA;
    float Lq_est = (Zq_mag * sinf(phi_q)) / INJECTION_OMEGA;

    // Signs before fabsf (checked in tools/sim calib_sim): (-,-) correct; (+,+) all currents
    // inverted; opposite signs = beta axis flipped (phase V/W swapped between ADC and PWM)
    printf("  signed Ld_est = %e, Lq_est = %e\r\n", Ld_est, Lq_est);

    // Reject implausible results (no bus voltage once gave ~35x the real L; the Rs ramp now faults
    // without a bus, so this is the second line)
    const float ld = fabsf(Ld_est), lq = fabsf(Lq_est);
    if (ld < CAL_L_MIN || ld > CAL_L_MAX || lq < CAL_L_MIN || lq > CAL_L_MAX ||
        lq > CAL_LQ_LD_MAX * ld || ld > CAL_LQ_LD_MAX * lq)
    {
        printf("\r\nLd/Lq implausible: keeping Ld %e, Lq %e\r\n", foc->bldc.Ld, foc->bldc.Lq);
        return;
    }

    foc->bldc.Ld = ld;
    foc->bldc.Lq = lq;
    foc->bldc.Ls = (foc->bldc.Ld + foc->bldc.Lq) * 0.5f;
}

// ---------------------------------------------------------------------------
// Saliency sweep: one point, after an MOTOR_ESTIMATION_LD run at foc->inj_angle.
// L_axis: apparent inductance along the injection axis, Ld*Lq / (L0 - L1*cos(2*d)) with
// d = rotor d-axis minus inj_angle, L0 = (Ld+Lq)/2, L1 = (Ld-Lq)/2 (R neglected).
// cross = I_perp / I_axis (complex): ~ -L1*sin(2*d) / (L0 - L1*cos(2*d)), mostly real.
// This is the signal a pulsating HF-injection observer would track. Returns false if no current.
bool FOC_BLDC_Saliency_Point(const FOC_t *foc, float *L_axis, float *cross_re, float *cross_im)
{
    float Vc, Vs, Ic, Is, Xc, Xs;
    Injection_Phasor(foc->vd_inj_buff, foc->sampling_time, &Vc, &Vs);
    Injection_Phasor(foc->id_inj_buff, foc->sampling_time, &Ic, &Is);
    Injection_Phasor(foc->iq_inj_buff, foc->sampling_time, &Xc, &Xs);

    float i2 = Ic * Ic + Is * Is;
    if (i2 < 1e-12f)
    {
        return false;
    }

    // Z = V / I_axis = |V|/|I| at phase phi; L = |Z| sin(phi) / w (sign as in the Ld/Lq fit)
    float phi = Fast_Atan2(Vs, Vc) - Fast_Atan2(Is, Ic);
    float z = Fast_Sqrt((Vc * Vc + Vs * Vs) / i2);
    *L_axis = fabsf(z * sinf(phi)) / INJECTION_OMEGA;

    // I_perp / I_axis with phasors (c, s) treated as c + j*s
    *cross_re = (Xc * Ic + Xs * Is) / i2;
    *cross_im = (Xs * Ic - Xc * Is) / i2;
    return true;
}

// ---------------------------------------------------------------------------
// Measurement of BLDC parameters using pre-defined signal injection
FOC_FAST_CODE static void FOC_BLDC_Parameter_Measurement_Update(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }

    if (foc->inj_start_flag)
    {
        // Safety: the injection drives a DC/AC voltage across a fraction of an ohm.
        // Abort before the bridge or the winding is damaged.
        // Only trips once current is actually flowing, so it cannot fire on the
        // very first tick before any voltage has been applied.
        float i_mag = Fast_Sqrt(foc->i_a * foc->i_a + foc->i_b * foc->i_b);
        if (i_mag > INJECTION_MAX_CURRENT)
        {
            // Stop driving, but leave foc_mode in FOC_CALIBRATION and preserve the
            // captured buffers so the caller can inspect what was measured.
            printf("\r\nInjection fault: i_mag = %f, i_a = %f, i_b = %f\r\n", i_mag, foc->i_a, foc->i_b);

            foc->inj_n = 0;
            foc->inj_start_flag = false;
            foc->inj_fault = true;
            foc->motor_mode = MOTOR_IDLE;
            foc->v_d = 0.0f;
            foc->v_q = 0.0f;
            foc->smo.theta_est = 0.0f;
            return;
        }

        // The Rs run holds the high level for RS_ALIGN_ROUNDS (the rotor swings into alignment;
        // its back-EMF would bias the reading), then two DC levels, low and high, each
        // INJECTION_ROUNDS settling + INJECTION_ROUNDS recorded. The Ld/Lq runs have one
        // settle + record block.
        const bool rs_run = (foc->motor_mode == MOTOR_ESTIMATION_RS);
        const uint32_t rs_low_end = RS_ALIGN_ROUNDS + 2u * INJECTION_ROUNDS;
        const uint32_t rounds_total = rs_run ? rs_low_end + 2u * INJECTION_ROUNDS : 2u * INJECTION_ROUNDS;

        if (rs_run && foc->inj_ramp)
        {
            // Ramp until the current reaches CAL_CURRENT; inj_v keeps the level reached. A
            // high-resistance motor may stop at the ceiling with at least CAL_MIN_RATIO of it.
            // Without a bus, or with an open winding, the ceiling comes first: fault.
            Clarke_Park_Transform(foc->i_a, foc->i_b, foc->inj_angle, &(foc->i_d), &(foc->i_q));
            const float v_max = CAL_V_MAX_RATIO * foc->v_bus;
            const float i_abs = fabsf(foc->i_d);
            if (i_abs >= CAL_CURRENT || (foc->inj_v > v_max && i_abs >= CAL_MIN_RATIO * CAL_CURRENT))
            {
                foc->inj_ramp = false;
            }
            else if (foc->v_bus < V_BUS_MIN || foc->inj_v > v_max)
            {
                foc->inj_start_flag = false;
                foc->inj_fault = true;
                foc->motor_mode = MOTOR_IDLE;
                foc->inj_v = 0.0f;
            }
            else
            {
                foc->inj_v += v_max * foc->sampling_time / CAL_RAMP_TIME;
            }
            foc->v_d = foc->inj_v;
            foc->v_q = 0.0f;
            return;
        }

        if (rs_run)
        {
            if (foc->inj_n == 0)
            {
                foc->rs_low_vd_sum = 0.0f;
                foc->rs_low_id_sum = 0.0f;
            }
            // The alignment hold runs at the full level: the swinging rotor's back-EMF stops the
            // ramp early (bench: 5.25 A during the ramp, 6.1 A once aligned), corrected below
            const bool low = (foc->inj_n >= RS_ALIGN_ROUNDS && foc->inj_n < rs_low_end);
            foc->v_d = low ? 0.5f * foc->inj_v : foc->inj_v;
            foc->v_q = 0.0f;
        }
        else
        {
            float angle = INJECTION_OMEGA * foc->inj_n * foc->sampling_time;
            float v_inj = foc->inj_v * Fast_Sin(angle);
            if (foc->motor_mode == MOTOR_ESTIMATION_LD)
            {
                foc->v_d = v_inj;
                foc->v_q = 0.0f;
            }
            else if (foc->motor_mode == MOTOR_ESTIMATION_LQ)
            {
                // Without the hold the q-axis torque moves a light rotor (T200: Lq 62-212 uH)
                foc->v_d = CAL_LQ_HOLD * CAL_CURRENT * foc->bldc.Rs;
                foc->v_q = v_inj;
            }
            else
            {
                foc->v_d = 0.0f;
                foc->v_q = 0.0f;
                return; // Invalid mode
            }
        }

        // Inject voltages in open-loop in the inj_angle frame
        foc->smo.theta_est = 0.0f;

        // Measure the resulting voltages and currents  (from the previous PWM cycle)
        Clarke_Park_Transform(foc->i_a, foc->i_b, foc->inj_angle, &(foc->i_d), &(foc->i_q));

        // End of the alignment hold: scale the level back to CAL_CURRENT (only down, so a motor
        // that stopped at the ramp ceiling keeps it)
        if (rs_run && foc->inj_n == RS_ALIGN_ROUNDS - 1u && fabsf(foc->i_d) > CAL_CURRENT)
        {
            foc->inj_v *= CAL_CURRENT / fabsf(foc->i_d);
        }

        // Accumulate injected voltages and measured currents after certain rounds
        if (rs_run && foc->inj_n >= rs_low_end - INJECTION_ROUNDS && foc->inj_n < rs_low_end)
        {
            // Low DC level of the Rs run: only the means are needed
            foc->rs_low_vd_sum += foc->v_d;
            foc->rs_low_id_sum += foc->i_d;
        }
        else if (foc->inj_n >= rounds_total - INJECTION_ROUNDS)
        {
            uint32_t index = foc->inj_n - (rounds_total - INJECTION_ROUNDS);
            foc->i_a_buff[index] = foc->i_a;
            foc->i_b_buff[index] = foc->i_b;

            if (foc->motor_mode == MOTOR_ESTIMATION_LQ)
            {
                foc->vq_inj_buff[index] = foc->v_q;
                foc->iq_inj_buff[index] = foc->i_q;
            }
            else
            {
                foc->vd_inj_buff[index] = foc->v_d;
                foc->id_inj_buff[index] = foc->i_d;
                foc->iq_inj_buff[index] = foc->i_q; // Cross-axis response (saliency sweep); the Lq run overwrites it
            }
        }

        // Increment injection round counter
        foc->inj_n++;

        // If reached maximum rounds, stop injection
        if (foc->inj_n >= rounds_total)
        {
            foc->motor_mode = MOTOR_IDLE;
            foc->inj_n = 0;
            foc->inj_start_flag = 0;
            foc->v_d = 0.0f;
            foc->v_q = 0.0f;
            foc->smo.theta_est = 0.0f;
        }
    }
}
