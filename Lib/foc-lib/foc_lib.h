#ifndef FOC_LIB_H
#define FOC_LIB_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// foc-lib: sensorless FOC for a 3-phase PMSM / BLDC (eSMO + PLL; HF injection at low speed)
// ============================================================================
//
// Porting
//   1. Provide the parameters of part 1 below for the board and the motor: define them in a
//      foc_lib_conf.h on the include path and build with -DFOC_LIB_USE_CONF (or pass -D...).
//      Part 2 holds optional tuning. Everything else is internal (foc_lib.c).
//   2. Run a centre-aligned PWM at PWM_FREQ that triggers one current/bus-voltage sample per
//      period. In its interrupt:
//        foc.adc_v_a, foc.adc_v_b, foc.adc_v_bus = raw ADC counts (phase A, phase B, bus);
//        FOC_Update(&foc);
//        apply foc.pwm_a / pwm_b / pwm_c (0..pwm_res) as compare values, and when
//        foc.outputs_enabled is false switch the bridge off (all six switches open). A zero duty
//        alone keeps the low-side switches on in complementary mode.
//      Never call printf or block in there.
//   3. At start-up (motor stopped):
//        FOC_Init(&foc, pwm_res);                    pwm_res = the timer's period count
//        foc.adc_offset_a/b = ADC counts at zero current (average a few hundred samples).
//   4. Optional calibration (standstill), each run polled with FOC_Measurement_Busy:
//        FOC_Start_Measurement(&foc, MOTOR_ESTIMATION_RS, 0) -> FOC_BLDC_Parameter_Calculation_Resistance
//        ... MOTOR_ESTIMATION_LD, then MOTOR_ESTIMATION_LQ  -> FOC_BLDC_Parameter_Calculation_Inductance
//        FOC_Recalc_Params(&foc);
//   5. Run: FOC_Set_Direction, foc.speed_ref (or foc.is_ref), FOC_Start(&foc, FOC_SPEED_CONTROL).
//      Stop: FOC_Stop(&foc). Faults coast the drive by themselves (vbus_fault, vbus_uv_fault,
//      hfi_fault, observer lost); the drive is motoring only (it never brakes).
//
// Units: angles in electrical radians, speeds in electrical rad/s, SI otherwise.

#ifdef FOC_LIB_USE_CONF
#include "foc_lib_conf.h"
#endif

// ============================================================================
// Part 1: board and motor parameters (provide for your setup)
// ============================================================================
// The defaults are the reference setup: Nucleo-F303RE + X-NUCLEO-IHM08M1 + Turnigy Aerodrive
// SK3 3542-1250KV. Define any of them before this header to override.

// PWM and ADC
#ifndef PWM_FREQ
#define PWM_FREQ 20000.0f // PWM = control loop frequency (Hz)
#endif
#ifndef ADC_RES
#define ADC_RES 4095 // ADC full-scale count
#endif
#ifndef ADC_V_REF
#define ADC_V_REF 3.3f // ADC reference voltage (V)
#endif

// Current and bus-voltage sensing
#ifndef R_SHUNT
#define R_SHUNT 0.01f // Phase current shunt (ohm)
#endif
#ifndef V_GAIN
#define V_GAIN 5.18f // Current-sense amplifier gain (V/V)
#endif
#ifndef V_OFFSET
#define V_OFFSET 1.71f // Current-sense output at zero current (V); FOC_Init's default offset
#endif
#ifndef CURRENT_SENSE_POLARITY
#define CURRENT_SENSE_POLARITY (-1.0f) // +1 if the reading rises with current INTO the phase, else -1.
                                       // The IHM08M1 reads below the offset (bench 2026-10-26).
                                       // Calibration cannot catch a wrong sign (see STATUS.md).
#endif
#ifndef V_BUS_GAIN
#define V_BUS_GAIN 0.0522f // Bus-voltage divider ratio (V at the ADC per V of bus)
#endif

// Motor (SK3 3542-1250KV: Ri 0.022 ohm, 1250 KV, 3-4S, 50-54 A max)
#ifndef RS
#define RS 0.022f // Phase resistance (ohm); bench calibration 0.0227
#endif
#ifndef LS
#define LS 0.000007f // Phase inductance (H), Ld ~ Lq; bench calibration Ld 5.5-7 / Lq 7.3-8.1 uH
#endif
#ifndef FLUX_LINKAGE
#define FLUX_LINKAGE 0.000630f // Magnet flux linkage per electrical rad (Wb): the starting value; the
                               // drive measures it while running (foc->bldc.flux). A rough value is
                               // enough, e.g. back-EMF peak / electrical speed, or from the KV rating:
                               // 60 / (sqrt(3) * 2 pi * KV * pole pairs).
#endif
// The pole-pair count is not needed: the library works in electrical angles and speeds
// (rpm = omega_el * 60 / (2 pi * pole pairs), for the application to convert if it wants to).

// Ratings and protection
#ifndef V_RATED
#define V_RATED 14.8f // Rated voltage (V): sets the d-axis voltage limit
#endif
#ifndef I_RATED
#define I_RATED 15.0f // Current limit (A): the IHM08M1's 15 A rms
#endif
#ifndef V_BUS_MIN
#define V_BUS_MIN 8.0f // Bus under-voltage (V): no calibration/start below it; trip while driving
#endif
#ifndef V_BUS_MAX
#define V_BUS_MAX 18.0f // Bus over-voltage trip (V): coast. Above a full 4S pack (16.8 V)
#endif

// ============================================================================
// Part 2: optional tuning (the defaults suit the reference setup)
// ============================================================================

// Port: code placement and PWM timing
#ifndef FOC_FAST_CODE
#define FOC_FAST_CODE // Attribute placing FOC_Update's call tree in the fastest code memory (e.g. a
                      // RAM section); the port must load that section before the first FOC_Update
#endif
#ifndef IPARK_DELAY_COMP
#define IPARK_DELAY_COMP 1.5f // Periods between a computed voltage and its application (PWM latch
                              // + centre of the next period): angle advance of the inverse Park
#endif
#ifndef ESMO_DELAY_COMP
#define ESMO_DELAY_COMP 0.23f // Residual observer latency to compensate (periods). Re-tune with
                              // IPARK_DELAY_COMP when PWM_FREQ or the latch timing changes
#endif
#ifndef ALPHA_LPF_MEAS
#define ALPHA_LPF_MEAS 0.5f // Phase-current low-pass alpha (1.0 = no filter)
#endif

// Low-speed HF injection (HFI): standstill start and running below HFI_OMEGA_LOW
#ifndef FOC_HFI
#define FOC_HFI 1 // 0 for a motor without enough saliency (Lq != Ld): Align -> I-f start, eSMO only
#endif
#ifndef HFI_STEP_CURRENT
#define HFI_STEP_CURRENT 0.54f // Current step per period of the PWM_FREQ / 4 square wave (A). Its
                               // amplitude is this * Ls * PWM_FREQ from the calibrated Ls: 0.075 V at 7 uH
#endif
#ifndef HFI_OMEGA_LOW
#define HFI_OMEGA_LOW 300.0f // eSMO -> HFI below this |speed| (rad/s el; 409 rpm at 7 pole pairs).
                             // Measured: bench shadow sweep (STATUS.md) + tools/hfi_threshold.py
#endif
#ifndef HFI_K
#define HFI_K 0.30f // HFI saliency signal |Zq| / |Zp| at 2e = 90 deg (bench 0.28-0.31; sims 0.14)
#endif
#ifndef HFI_POL_CURRENT
#define HFI_POL_CURRENT (0.4f * I_RATED) // N/S polarity pulses at standstill (A, d-axis: no torque)
#endif
#ifndef HFI_POL_MARGIN
#define HFI_POL_MARGIN 0.02f // Minimum relative HF-response contrast to trust the polarity
#endif

// Startup without HFI (FOC_HFI 0, or no saliency/polarity): Align -> I-f ramp -> observer
#ifndef ALIGN_TIME
#define ALIGN_TIME 0.5f // Rotor alignment time (s)
#endif
#ifndef I_ALIGN
#define I_ALIGN 2.0f // d-axis alignment current (A)
#endif
#ifndef I_STARTUP
#define I_STARTUP 3.0f // q-axis current of the I-f ramp (A): torque = 1.5 * pole pairs * FLUX_LINKAGE * I
#endif
#ifndef I_REF_SLEW
#define I_REF_SLEW 200.0f // Startup current-reference slew (A/s)
#endif
#ifndef STARTUP_ACCEL
#define STARTUP_ACCEL 800.0f // Forced acceleration (rad/s^2 el)
#endif
#ifndef OMEGA_FORCE
#define OMEGA_FORCE 500.0f // Forced -> observer handover speed (rad/s el); back-EMF there should be
                           // well above 0.1 V (OMEGA_FORCE * FLUX_LINKAGE = 0.32 V)
#endif
#ifndef LOCK_TIMEOUT
#define LOCK_TIMEOUT 0.5f // Max time at OMEGA_FORCE waiting for the observer (s), then coast
#endif
#ifndef LOCK_SPEED_TOL
#define LOCK_SPEED_TOL 0.2f // Observer speed must agree with the forced speed within 20 %
#endif
#ifndef EMF_PLAUSIBLE_RATIO
#define EMF_PLAUSIBLE_RATIO 0.5f // Lock needs |E| >= ratio * |forced speed| * FLUX_LINKAGE (rejects a
                                 // "lock" onto the forced current vector of a stalled rotor)
#endif
#ifndef BEMF_LOCK_COUNT
#define BEMF_LOCK_COUNT ((uint32_t)(0.05f * PWM_FREQ)) // Locked periods before the handover (50 ms)
#endif
#ifndef ANGLE_BLEND_RATE
#define ANGLE_BLEND_RATE 10.0f // Angle offset decay at a handover or an eSMO <-> HFI switch (rad/s)
#endif

// Control loops
#ifndef CURRENT_CONTROL_F_BANDWIDTH
#define CURRENT_CONTROL_F_BANDWIDTH 1000.0f // Current loop bandwidth (Hz). Keep it off the 6x back-EMF
                                            // harmonic (sim at 20 kHz: 2 kHz gave 3x the ripple)
#endif
#ifndef SPEED_PLANT_GAIN
#define SPEED_PLANT_GAIN 3300.0f // d(speed)/dt per A of iq (rad/s^2 el per A) = 1.5 pp^2 psi / J;
                                 // bench 2026-10-26, unloaded rotor
#endif
#ifndef SPEED_CTRL_BW_HZ
#define SPEED_CTRL_BW_HZ 10.0f // Speed loop crossover (Hz)
#endif
#ifndef SPEED_REF_SLEW
#define SPEED_REF_SLEW 2000.0f // Speed reference ramp (rad/s^2 el)
#endif
#ifndef ESMO_OMEGA_MAX
#define ESMO_OMEGA_MAX 4000.0f // Observer speed clamp (rad/s el), ~5500 rpm at 7 pole pairs
#endif
#ifndef HARM_COMP
#define HARM_COMP 1 // 1: cancel the 6x current ripple from back-EMF 5th/7th harmonics (running)
#endif
#ifndef HARM_KI
#define HARM_KI 20.0f // Its integrator gain (V per A per s)
#endif

// Rs / Ld / Lq calibration (self-commissioning at standstill; no motor constant needed). The Rs run
// ramps a DC voltage up until the current reaches CAL_CURRENT, then measures at half that voltage
// and at the full one. The Ld/Lq runs inject CAL_CURRENT * Rs at 1 kHz (|Z| >= Rs: at most
// CAL_CURRENT). Keep CAL_CURRENT well above the dead-time knee (low level ~ CAL_CURRENT / 2).
#ifndef CAL_CURRENT
#define CAL_CURRENT (0.35f * I_RATED) // A: 5.25 A here (bench: ~0.14 V, low level ~2.3 A, AC ~2.4 A)
#endif
#ifndef INJECTION_MAX_CURRENT
#define INJECTION_MAX_CURRENT I_RATED // Abort a calibration above this current (A)
#endif

// ============================================================================
// Part 3: fixed sizes (array dimensions in FOC_t)
// ============================================================================
#define INJECTION_ROUNDS 256 // Samples recorded per calibration run
#define HFI_DELAY 2u         // Periods from an HFI command to the current step it causes

// ============================================================================
// Types
// ============================================================================
// BLDC motor operation modes
typedef enum
{
    MOTOR_POWER_ON = 0, // Initial power-on state. All parameters are set to default.
    MOTOR_IDLE,         // Motor is powered and parameters are ready but not running.

    // Calibration modes
    MOTOR_ESTIMATION_RS,
    MOTOR_ESTIMATION_LD,
    MOTOR_ESTIMATION_LQ,

    // Startup modes
    MOTOR_STARTUP_ALIGN,
    MOTOR_STARTUP_RAMP,
    MOTOR_STARTUP_TRANSITION, // Observer angle + decaying offset, startup current

    // Closed-loop modes
    MOTOR_RUNNING,

    // Startup from standstill on HFI (FOC_HFI): lock, N/S polarity, then MOTOR_RUNNING.
    // Last, so the existing mode numbers (logs) do not change.
    MOTOR_STARTUP_HFI
} BLDC_Mode_t;

// FOC Control Modes
typedef enum
{
    FOC_IDLE = 0,
    FOC_CALIBRATION,
    FOC_TORQUE_CONTROL,
    FOC_SPEED_CONTROL
} FOC_Control_Mode_t;

// BLDC rotation direction
typedef enum
{
    NORMAL_DIRECTION = 0,
    REVERSE_DIRECTION
} BLDC_Direction_t;

// PI Controller structure
typedef struct
{
    float Kp;
    float Ki;
    float Kb; // Anti-windup back-calculation gain Kt = Ki/Kp = 1/Ti (1/s)
    float Ts; // Sampling time
    float Ki_Ts; // Ki * Ts (pre-computed in PI_Change_Params)
    float Kb_Ts; // min(Kb * Ts, 1) (pre-computed in PI_Change_Params)
    float integral;
    float output_limit; // Symmetric output clamp; negative means unlimited
} PI_Controller_t;

// BLDC specifications structure
typedef struct
{
    float max_current;
    float Rs;   // Stator resistance
    float Ld;   // Stator inductance (d-axis)
    float Lq;   // Stator inductance (q-axis)
    float Ls;   // Stator inductance (average of Ld and Lq)
    float flux; // Magnet flux linkage (Wb): FLUX_LINKAGE, then measured while running on the eSMO
} BLDC_Specs_t;

// Enhanced sliding-mode observer (eSMO + PLL) structure
typedef struct
{
    // PLL for rotor position and speed estimation
    PI_Controller_t pll;
    float sampling_time;
    float theta_est;
    float omega_est;

    // Observer states
    float e_alpha_filtered;
    float e_beta_filtered;
    float i_alpha_est;
    float i_beta_est;

    // Pre-computed in SMO_Compute_Coefficients() (rerun on BLDC spec / Ts change)
    float esmo_F;         // Current-model pole exp(-Rs*Ts/Ld)
    float esmo_G;         // Current-model gain (1 - F)/Rs (A/V)
    float esmo_z_gain;    // lambda/G: linear sliding gain inside the boundary layer (V/A)
    float esmo_inv_kappa; // (1 - F + lambda)/lambda: restores unity EMF gain
    float esmo_dL;        // Ld - Lq (H), extended-EMF cross-coupling
    float esmo_ratio_ts;  // ESMO_WC_RATIO * Ts: wc*Ts per rad/s of |omega|
    float esmo_x_lo;      // esmo_wc_lo * Ts
    float esmo_wc_lo;     // LPF cutoff floor (rad/s)
    float esmo_wc_hi;     // LPF cutoff ceiling (rad/s)
    float esmo_a_lo;      // LPF coefficient at the floor, already divided by kappa
    float esmo_a_hi;      // LPF coefficient at the ceiling, already divided by kappa
    float esmo_lag;       // atan(1/ESMO_WC_RATIO): constant LPF lag while wc tracks speed
    float esmo_delay_ts;  // ESMO_DELAY_COMP * Ts (s)

    // Run-time state
    float k_slide;          // Scheduled sliding gain (V)
    float theta_emf;        // PLL angle of the back-EMF vector (rad, 0..2pi)
    float pll_err_filtered; // Smoothed normalised PLL error, sin(angle error)
    float emf_magnitude;    // |E_hat| (V)
    bool locked;            // Lock state reported by SMO_Update
    uint16_t unlock_count;  // Consecutive samples failing the lock criterion
} SMO_t;

// Drive state. One instance per motor; FOC_Update owns it in the ISR. The application uses:
//   in (ISR):   adc_v_a, adc_v_b, adc_v_bus             raw ADC counts of the period
//   out (ISR):  pwm_a, pwm_b, pwm_c, outputs_enabled    compare values; false = bridge off
//   setup:      adc_offset_a, adc_offset_b              measured zero-current counts
//   targets:    speed_ref (rad/s el), is_ref (A)        relative to the direction
//   status:     foc_mode, motor_mode, v_bus, i_d, i_q, theta, omega, bldc (Rs, Ld, Lq),
//               vbus_fault, vbus_uv_fault, hfi_fault, inj_fault, hfi_control, hfi_pol
// Everything else is internal state (readable for logging; do not write it).
typedef struct
{
    FOC_Control_Mode_t foc_mode;
    BLDC_Mode_t motor_mode;

    BLDC_Specs_t bldc;

    SMO_t smo;

    PI_Controller_t id_ctrl;
    PI_Controller_t iq_ctrl;
    PI_Controller_t speed_ctrl;

    // Configuration parameters
    float sampling_time;
    uint16_t pwm_resolution; // Max PWM count value

    // Pre-computed from the configuration (FOC_Init / FOC_Recalc_Params)
    float ipark_adv;   // IPARK_DELAY_COMP * Ts (s)
    float harm_k;      // HARM_KI * Ts
    float mtpa_k;      // 8 * (Lq - Ld)^2, MTPA
    float mtpa_inv_4l; // 1 / (4 * (Lq - Ld)), 0 for a non-salient motor (MTPA: id = 0)

    // DC offset voltage for current sensing
    uint16_t adc_offset_a;
    uint16_t adc_offset_b;
    uint16_t adc_offset_c;

    // Measured variables
    uint16_t adc_v_a, adc_v_b, adc_v_c; // ADC values for voltage across current sensing resistors
    uint16_t adc_v_bus;                 // ADC value for bus voltage

    // Operation parameters
    BLDC_Direction_t rotor_dir;     // Rotation direction: set with FOC_Set_Direction (bridge off only)
    float v_bus;                    // DC bus voltage
    float i_a, i_b;                 // Measured phase currents
    float i_alpha, i_beta;          // Clarke transformed currents
    float i_d, i_q;                 // Park transformed currents
    float id_filtered, iq_filtered; // Filtered DQ currents

    float speed_ref;       // Desired speed in rotor_dir (electrical rad/s, >= 0; negative = 0, motoring only)
    float speed_ref_ramp;  // speed_ref after the SPEED_REF_SLEW ramp (what the speed PI tracks), in rotor_dir
    uint16_t speed_div;    // Current-loop periods since the last speed-loop update
    bool vbus_fault;       // Set when the bus over-voltage trip (V_BUS_MAX) coasted the drive
    bool vbus_uv_fault;    // Set when the bus under-voltage trip (V_BUS_MIN while driving) coasted the drive
    uint32_t vbus_uv_cnt;  // Consecutive periods below V_BUS_MIN with the outputs on
    float harm_p[2], harm_n[2]; // 6x harmonic integrators (+6 / -6 sequence frames), volts
    float harm_rp[2], harm_rn[2]; // Their phase-compensation phasors (unit), refreshed at 1 kHz
    uint16_t harm_div;     // Current-loop periods since the last phasor refresh
    float id_ref, iq_ref;  // Desired DQ currents
    float is_ref;          // Torque mode: stator current (A) relative to rotor_dir (positive drives, negative brakes)
    float v_d, v_q;        // DQ voltages
    float v_alpha, v_beta; // Inverse Park transformed voltages

    // PWM outputs
    uint32_t pwm_a;
    uint32_t pwm_b;
    uint32_t pwm_c;

    // Temporary variables for BLDC calibration
    uint32_t inj_n; // Injection round counter
    bool inj_start_flag;
    bool inj_fault; // Set when injection aborted: over-current, or CAL_CURRENT not reached (no bus, open winding)
    bool inj_ramp;  // Rs run: still ramping the DC voltage up to CAL_CURRENT
    float inj_v;    // Rs run: DC voltage reached at CAL_CURRENT (V); Ld/Lq runs: 1 kHz amplitude (V)
    float inj_angle; // Injection frame angle (rad el, 0..2pi): 0 for Rs/Ld/Lq, swept by the saliency test
    float rs_low_vd_sum, rs_low_id_sum; // Two-point Rs: sums recorded at the low level (inj_v / 2)
    float i_a_buff[INJECTION_ROUNDS], i_b_buff[INJECTION_ROUNDS]; // Phase currents (current-sign diagnostic)
    float vd_inj_buff[INJECTION_ROUNDS]; // Buffer for injected voltages
    float vq_inj_buff[INJECTION_ROUNDS];
    float id_inj_buff[INJECTION_ROUNDS]; // Buffer for measured currents
    float iq_inj_buff[INJECTION_ROUNDS];

    // HFI tracker and eSMO <-> HFI switch (see foc_lib.c); ISR state, except hfi_takeover.
    bool hfi_takeover;              // Test only (default true). false = shadow run for measuring HFI_OMEGA_LOW:
                                    // Align -> I-f start, tracker logged beside the eSMO, never in control
    uint8_t hfi_stage;              // MOTOR_STARTUP_HFI stage: 0 lock, 1 probe, 2 +pulse, 3 zero, 4 -pulse, 5 zero
    int8_t hfi_pol;                 // Polarity result: +1 kept, -1 flipped by pi, 0 not trusted (Align -> I-f)
    uint32_t hfi_pol_n;             // Samples in the current |Zp| average
    float hfi_zp_pos, hfi_zp_neg;   // Mean |Zp| (A) during the +/- pulses, diagnostic
    float hfi_probe;                // Injection axis offset (rad): HFI_PROBE_ANGLE during the probe (PLL frozen), else 0
    float hfi_probe_e;              // Mean tracker error during the probe (rad), diagnostic
    bool hfi_salient;               // false once the start's probe found too little saliency: the
                                    // tracker stays off for that run (eSMO only)
    float hfi_v;                    // Square-wave amplitude (V), sized from bldc.Ls (HFI_STEP_CURRENT)
    bool hfi_on;                    // Tracker running (injecting)
    bool hfi_control;               // true: the transforms use the HFI angle; false: the eSMO's
    bool hfi_fault;                 // Latched: lost the HF response or the track (coasted)
    uint32_t hfi_cnt;               // Periods since the tracker started (stops at HFI_WARMUP)
    uint32_t hfi_idx;               // Square-wave phase 0..3 (sign + for 0, 1)
    uint32_t hfi_agree_cnt;         // HFI -> eSMO: consecutive periods of agreement
    uint32_t hfi_fault_cnt;         // Consecutive periods with a bad HF response or a large error
    uint32_t hfi_rev_cnt;           // Consecutive periods of rotation against rotor_dir in HFI control
    uint32_t hfi_vbal_cnt;          // Consecutive periods of unexplained voltage in HFI control
    float hfi_theta;                // Tracker angle (rad el, 0..2pi): rotor d-axis estimate
    float hfi_omega;                // Tracker speed (PLL integrator, electrical rad/s)
    float hfi_phi_h[HFI_DELAY];     // Applied injection axis angle (alpha-beta), [0] = newest
    float hfi_ia_h, hfi_ib_h;       // Unfiltered alpha/beta currents of the previous sample
    float hfi_par[4], hfi_perp[4];  // Current steps of the last 4 periods, by square-wave phase
    float hfi_err;                  // Angle error e (rad), diagnostic
    float hfi_zp;                   // |Zp| (A), diagnostic
    // Angle used by the Park / inverse Park transforms, set by the state machine:
    // align angle, forced angle, blended angle or observer angle. The observer's
    // own state (smo.theta_est) is never overwritten.
    float theta;           // Electrical angle for the transforms (rad, 0..2pi)
    float omega;           // Rate of theta (electrical rad/s), for the inverse-Park delay advance
    float omega_ff;        // omega low-pass filtered for the current-loop feedforward (electrical rad/s)
    bool outputs_enabled;  // false: the HAL must disable the gate driver / PWM outputs (coast)

    // Temporary variables for FOC control
    float startup_timer;   // Timer for startup routines
    float startup_theta;   // Open-loop (forced) angle for startup
    float startup_omega;   // Open-loop (forced) speed for startup (electrical rad/s)
    float angle_offset;    // Transition: forced minus observer angle, decays to 0
    uint32_t bemf_counter; // Counter for back-EMF detection during startup
} FOC_t;

// ============================================================================
// API
// ============================================================================
#ifdef __cplusplus
extern "C"
{
#endif

    // Setup (motor stopped)
    void FOC_Init(FOC_t *foc, uint32_t pwm_res);              // pwm_res = PWM period count (100 % duty)
    bool FOC_Set_Direction(FOC_t *foc, BLDC_Direction_t dir); // Bridge off only; false otherwise
    void FOC_Set_Current_Limit(FOC_t *foc, float amps);       // Default I_RATED
    float FOC_Min_Speed(const FOC_t *foc);                    // Speed mode: a lower speed_ref stops the motor

    // Run and stop. Targets: foc->speed_ref (speed mode) or foc->is_ref (torque mode), relative
    // to the direction; the drive is motoring only (a negative speed_ref means 0).
    void FOC_Start(FOC_t *foc, FOC_Control_Mode_t mode); // FOC_SPEED_CONTROL or FOC_TORQUE_CONTROL
    void FOC_Stop(FOC_t *foc);                           // Bridge off at the next FOC_Update: coast
    void FOC_Coast(FOC_t *foc);                          // The same, immediately (from the ISR context)

    // Per PWM period, in the ISR: after writing foc->adc_v_a, adc_v_b, adc_v_bus
    void FOC_Update(FOC_t *foc);

    // Calibration (standstill): Rs, then Ld and Lq, then FOC_Recalc_Params. See FOC_Start_Measurement.
    void FOC_Start_Measurement(FOC_t *foc, BLDC_Mode_t which, float inj_angle);
    bool FOC_Measurement_Busy(const FOC_t *foc);
    void FOC_BLDC_Parameter_Calculation_Resistance(FOC_t *foc); // After an Rs run: foc->bldc.Rs
    void FOC_BLDC_Parameter_Calculation_Inductance(FOC_t *foc); // After LD and LQ runs: Ld, Lq, Ls
    void FOC_Recalc_Params(FOC_t *foc);                         // Apply foc->bldc to the loops and observer
    // Diagnostic: apparent inductance along inj_angle after an LD run (saliency sweep)
    bool FOC_BLDC_Saliency_Point(const FOC_t *foc, float *L_axis, float *cross_re, float *cross_im);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // FOC_LIB_H
