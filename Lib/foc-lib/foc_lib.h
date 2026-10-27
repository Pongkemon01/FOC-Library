#ifndef FOC_LIB_H
#define FOC_LIB_H

#include <stdint.h>
#include <stdbool.h>

// Optional port configuration. Build with -DFOC_LIB_USE_CONF to include "foc_lib_conf.h"
// (from the application's include path) before the defaults below. A port may define:
//   FOC_FAST_CODE  Attribute placing the per-period code (FOC_Update's call tree) in the
//                  platform's fastest code memory, e.g. a RAM section on an MCU that runs
//                  flash with wait states. The port must then also load that section
//                  before the PWM ISR first calls FOC_Update.
#ifdef FOC_LIB_USE_CONF
#include "foc_lib_conf.h"
#endif

#ifndef FOC_FAST_CODE
#define FOC_FAST_CODE // Default: ordinary code placement
#endif

// ============================================================================
// Constants
// ============================================================================
#define ONE_OVER_SQRT3 0.57735026919f // 1 / sqrt(3)
#define TWO_OVER_SQRT3 1.15470053838f // 2 / sqrt(3)
#define TWO_PI 6.28318530718f         // 2 * pi
#define PI 3.14159265359f             // pi
#define HALF_PI 1.57079632679f        // pi / 2
#define SQRT3_OVER_2 0.86602540378f  // sqrt(3) / 2

// ---------------------------------------------------------------------------
// Board, sensing and motor parameters. Each is a default for the reference setup
// (Nucleo-F303RE + X-NUCLEO-IHM08M1 + Turnigy Aerodrive SK3 3542-1250KV). A port overrides
// any of them by defining it first, in its foc_lib_conf.h (see FOC_LIB_USE_CONF above) or
// with -D. The derived constants below them (TS, ADC_2_*, V_2_CURRENT, ADC_OFFSET) follow.

// System default parameters
#ifndef PWM_FREQ
#define PWM_FREQ 20000.0f                       // Sampling frequency (Hz) (aka. PWM frequency). 20 kHz since
                                                // 2026-10-26 (ISR ~2000 of 3600 cycles); was 10 kHz
#endif
#define TS (1.0f / PWM_FREQ)                    // Sampling period (s)
#ifndef PWM_RES
#define PWM_RES 28799                           // Maximum PWM value (abs. max = 65535)
#endif
#ifndef ADC_RES
#define ADC_RES 4095                            // ADC resolution
#endif
#ifndef ADC_V_REF
#define ADC_V_REF 3.3f                          // ADC reference voltage (V)
#endif
#define ADC_2_VOLT (ADC_V_REF / (float)ADC_RES) // ADC to voltage conversion factor

// Default motor electrical parameters: Turnigy Aerodrive SK3 3542-1250KV
// (spec: 1250 KV, Ri 0.022 ohm, 3-4S, 50-54 A max, ~760 W; pole count not published)
#ifndef RS
#define RS 0.022f                    // Stator resistance (Ω): spec Ri; bench calibration 0.0227 (2026-10-26)
#endif
#ifndef LS
#define LS 0.000007f                 // Stator inductance (H) - assume Ld ≈ Lq. Not in spec: bench calibration
                                     // Ld 6.7 / Lq 8.1 uH (calib_sim bias depends on R/L: 5.9 uH read for 7 uH)
#endif
#ifndef POLE_PAIRS
#define POLE_PAIRS 7                 // Number of pole pairs (14 magnets assumed, typical for SK3 35xx: count to confirm)
#endif
#ifndef FLUX_LINKAGE
#define FLUX_LINKAGE 0.000630f       // Flux linkage (Wb) = 60 / (sqrt(3) * 2*pi * KV * POLE_PAIRS), KV = 1250
                                     // Re-derive if POLE_PAIRS changes (6 pp -> 0.000735)
#endif

// Default motor ratings
#ifndef V_RATED
#define V_RATED 14.8f       // Rated voltage (V): motor spec 3-4S = 11.1-14.8 V (bench supply ~11.4 V)
#endif
#ifndef I_RATED
#define I_RATED 15.0f        // Rated current (A): IHM08M1 limit (15 A rms); motor spec 50-54 A max
#endif
#ifndef I_PEAK
#define I_PEAK 30.0f         // Peak current (A): just inside the ~+-31 A sense range (1.65 V / (V_GAIN * R_SHUNT))
#endif

// Default motor controller parameters
#ifndef R_SHUNT
#define R_SHUNT 0.01f                                               // Shunt resistor value (Ω)
#endif
#ifndef V_OFFSET
#define V_OFFSET 1.71f                                              // ADC voltage offset (V)
#endif
#ifndef V_GAIN
#define V_GAIN 5.18f                                                // Gain of the voltage sensing circuit (V/V)
#endif
#ifndef V_BUS_GAIN
#define V_BUS_GAIN 0.0522f                                          // Gain of the bus voltage sensing circuit (V/V)
#endif
#define ADC_OFFSET ((int)((V_OFFSET / (float)ADC_V_REF) * ADC_RES)) // ADC value when current = 0
// Sign of the ADC reading for current flowing INTO the motor phase. X-NUCLEO-IHM08M1 low-side
// shunts read below the offset for positive phase current (bench-verified 2026-10-26: +v_d gave
// i_a < 0 before this was -1), so -1 here. Set +1 for a non-inverting sense chain.
#ifndef CURRENT_SENSE_POLARITY
#define CURRENT_SENSE_POLARITY (-1.0f)
#endif
#define V_2_CURRENT (CURRENT_SENSE_POLARITY / (V_GAIN * R_SHUNT))   // Voltage to current conversion factor (A/V)
#define ADC_2_VBUS (ADC_2_VOLT / V_BUS_GAIN)                        // ADC to bus voltage conversion factor (V)
#define ADC_2_CURRENT (ADC_2_VOLT * V_2_CURRENT)                    // ADC counts to current (A), folded at compile time

// Default parameters for startup: Align -> I-f ramp -> Transition -> Run
// All stages are current-controlled (the current PI loops run throughout), as in
// TI's Universal Lab force-angle startup (SPRUJ26A, Appendix A).
#ifndef ALIGN_TIME
#define ALIGN_TIME 0.5f        // Rotor alignment time (s)
#endif
#ifndef I_ALIGN
#define I_ALIGN 2.0f           // d-axis alignment current (A), 5..50 % of rated
#endif
#ifndef I_STARTUP
#define I_STARTUP 3.0f         // q-axis current for the I-f ramp and transition (A), 10..100 % of rated
#endif
#ifndef I_REF_SLEW
#define I_REF_SLEW 200.0f      // Startup current-reference slew rate (A/s): no current steps
#endif
#ifndef STARTUP_ACCEL
#define STARTUP_ACCEL 800.0f   // Open-loop acceleration (electrical rad/s^2)
#endif
#ifndef OMEGA_FORCE
#define OMEGA_FORCE 500.0f     // Open-loop -> observer handover speed (electrical rad/s);
                               // Back-EMF here = OMEGA_FORCE * FLUX_LINKAGE: 0.32 V at 500 (want >= 0.25 V).
                               // 500/800 suits a light load (sim); with a prop 300/600 hands over, 500/800 loses sync
#endif
#ifndef LOCK_TIMEOUT
#define LOCK_TIMEOUT 0.5f      // Max time at OMEGA_FORCE waiting for observer lock (s), then fault
#endif
#ifndef LOCK_SPEED_TOL
#define LOCK_SPEED_TOL 0.2f    // Observer speed must agree with the forced speed within 20 %
#endif
#ifndef EMF_PLAUSIBLE_RATIO
#define EMF_PLAUSIBLE_RATIO 0.5f // Lock needs |E_observed| >= ratio * |forced speed| * FLUX_LINKAGE:
                                 // rejects a "lock" onto the forced current vector of a stalled rotor
#endif
#ifndef BEMF_LOCK_COUNT
#define BEMF_LOCK_COUNT ((uint32_t)(0.05f * PWM_FREQ)) // Consecutive locked observer updates before the transition (50 ms)
#endif
#ifndef ANGLE_BLEND_RATE
#define ANGLE_BLEND_RATE 10.0f // Transition: forced-to-observer angle offset decay (rad/s);
                               // 90 deg takes ~0.16 s
#endif

// ---------------------------------------------------------------------------
// Low-pass filter parameters used in measurement
#ifndef ALPHA_LPF_MEAS
#define ALPHA_LPF_MEAS 0.5f                  // LPF alpha for current and voltage measurement (1.0 = no filter)
#endif
#define BETA_LPF_MEAS (1.0f - ALPHA_LPF_MEAS) // LPF beta for current and voltage measurement

// ---------------------------------------------------------------------------
// Enhanced sliding-mode observer (eSMO) parameters  -> see SMO_Update()
#define KP_PLL 630.0f             // PLL proportional gain
#define KI_PLL 100000.0f          // PLL integral gain
#define MIN_BEMF_AMPLITUDE 0.1f  // Minimum back-EMF amplitude for reliable estimation (V)
#define ESMO_K_SLIDE_MIN 1.0f         // Minimum sliding gain (V)
#define ESMO_K_SLIDE_MARGIN 1.5f      // Sliding gain = margin * |E_hat| (must be > 1)
#define ESMO_BOUNDARY_GAIN 0.0f       // lambda in (0, 1+F). 0 = auto (lambda = F, deadbeat current error)
#define ESMO_WC_MIN (TWO_PI * 30.0f)  // Minimum back-EMF LPF cutoff (rad/s)
#define ESMO_WC_RATIO 2.0f            // LPF cutoff = ratio * |omega| -> constant atan(1/ratio) lag, compensated
#define ESMO_WC_MAX_TS 0.7f           // Ceiling on wc*Ts: keeps the discrete back-EMF loop well damped
#define ESMO_OMEGA_MAX 4000.0f        // PLL speed clamp (electrical rad/s) = ~5500 rpm at 7 pp. Below the motor's
                                      // no-load 11000-13600 rad/s el (12-14.8 V): at 10 kHz PWM that would be < 6 samples/el. cycle
#ifndef ESMO_DELAY_COMP
#define ESMO_DELAY_COMP 0.23f         // Residual observer latency to compensate, in sample periods
#endif
#define ESMO_LOCK_ERR 0.10f           // |sin(angle error)| below which the PLL counts as locked (~6 deg)
#define ESMO_UNLOCK_ERR 0.50f         // |sin(angle error)| above which lock is lost (~30 deg)
#define ESMO_PLL_ERR_ALPHA (TS / 0.005f) // Smoothing of the PLL error used for the lock detector (5 ms time constant)
#define ESMO_UNLOCK_COUNT ((uint16_t)(0.005f * PWM_FREQ)) // Consecutive bad samples before reporting loss of lock (5 ms)

// 6x harmonic compensation in closed loop (FOC_Harmonic_Comp). The 5th/7th back-EMF harmonics of
// the 12N14P outrunner appear in d/q as vectors rotating at -6 and +6 x omega (1.4-2.4 kHz at
// 1500-2500 rad/s el), beyond the PI current loop. Each is integrated in its own rotating frame,
// phase-compensated with the plant seen at that frequency (PWM delay, current LPF, RL, PI loop).
#ifndef HARM_COMP
#define HARM_COMP 1               // 1: enable in MOTOR_RUNNING
#endif
#ifndef HARM_KI
#define HARM_KI 20.0f             // Harmonic integrator gain (V per A per s)
#endif
#define HARM_V_MAX 0.5f           // Magnitude limit of each harmonic voltage vector (V)
#define HARM_OMEGA_MIN 300.0f     // Active above this |omega| (electrical rad/s)
#define HARM_DELAY_TS 1.5f        // PWM delay seen by the harmonic, in periods (as IPARK_DELAY_COMP)

// Angle advance for the inverse Park transform in closed loop, in sample periods.
// The voltage computed now is applied ~1.5 periods later (PWM shadow-register
// update + centre of the next period). Set to 0.0f to disable.
#define IPARK_DELAY_COMP 1.5f

// ---------------------------------------------------------------------------
// BLDC calibration parameters
#define INJECTION_ROUNDS 256 // Number of injection rounds for parameter measurement
// Injection voltage (V). In the amplitude-invariant alpha-beta frame the
// winding resistance is exactly Rs, so the DC injection current is V/Rs:
// minus the inverter voltage error (~20-30 mV at 20 kHz): at Rs = 0.022 ohm, ~5 A DC at 0.14 V
// (Rs run); the 1 kHz Ld/Lq runs give V / |Rs + j*w*L| ~= 2.8 A amplitude at L = 7 uH.
// (An earlier "1.5*Rs" note here described the SVPWM gain bug, now fixed.)
// Raising this scales the current inversely with Rs - re-check before changing.
#define INJECTION_VOLTAGE 0.14f        // Injection voltage for parameter measurement (V)
// Rs is measured at two DC levels (INJECTION_VOLTAGE_RS_LOW, then INJECTION_VOLTAGE) and taken
// from the difference, which cancels the inverter's voltage error only where that error is
// constant. At 20 kHz it still grew between 0.04 V and 0.07 V (16 -> 26 mV, Rs read 0.029-0.033),
// so both levels sit higher (2026-10-26). ~2.3 A at 0.08 V.
#define INJECTION_VOLTAGE_RS_LOW 0.08f // Low DC level of the two-point Rs measurement (V)
// Time at the low level before recording: the DC field pulls the unloaded rotor into alignment
// and its swing (back-EMF) biased Rs at 20 kHz when only INJECTION_ROUNDS (12.8 ms) were allowed.
#define RS_ALIGN_ROUNDS ((uint32_t)(0.3f * PWM_FREQ))
#define INJECTION_MAX_CURRENT I_RATED // Abort calibration above this measured current (A); I_PEAK (30 A)
                                      // is above the IHM08M1's ~21 A peak, so it would not protect the bridge
#define MIN_SALIENCY_H 1e-7f                      // Below this Lq-Ld the motor is treated as non-salient (H)
#define INJECTION_FREQ 1000.0f                    // Injection frequency for parameter measurement (Hz)
#define INJECTION_OMEGA (TWO_PI * INJECTION_FREQ) // Injection angular frequency (rad/s)

// ---------------------------------------------------------------------------
// FOC control parameters
#define VD_MAX (V_RATED * ONE_OVER_SQRT3)                                  // Maximum D-axis voltage (V)
#define CURRENT_CONTROL_F_BANDWIDTH 1000.0f // Default current control bandwidth (Hz). Fixed, not PWM_FREQ/10:
                                            // at 20 kHz a 2 kHz loop puts the 6x back-EMF harmonic (2.4 kHz at
                                            // 2500 rad/s el) on its sensitivity peak (sim: 3x the ripple)
#define CURRENT_CONTROL_O_BANDWIDTH (TWO_PI * CURRENT_CONTROL_F_BANDWIDTH) // Default current control bandwidth (rad/s)
#define KP_I (LS * CURRENT_CONTROL_O_BANDWIDTH)                            // D and Q axis current controller proportional gain
#define KI_I (RS * CURRENT_CONTROL_O_BANDWIDTH)                            // D and Q axis current controller integral gain
// Speed loop: runs inside FOC_Update (MOTOR_RUNNING, FOC_SPEED_CONTROL) every SPEED_CTRL_DIVIDER
// periods. Speeds are electrical rad/s. Plant: d(omega_el)/dt = SPEED_PLANT_GAIN * iq, with
// SPEED_PLANT_GAIN = 1.5 * pp^2 * psi / J; bench 2026-10-26: 3 A gave ~9800 rad/s^2 el unloaded.
// Motoring only (no regenerative braking): a bench supply cannot absorb the returned energy.
#define SPEED_CTRL_DIVIDER ((uint16_t)(PWM_FREQ / 1000.0f))                // Speed loop every N current-loop periods (1 kHz)
#define TS_SPEED (SPEED_CTRL_DIVIDER * TS)                                 // Speed loop sampling period (s)
#define SPEED_PLANT_GAIN 3300.0f                                           // (rad/s^2 el) per A, unloaded rotor
#define SPEED_CTRL_BANDWIDTH (TWO_PI * 10.0f)                              // Speed loop crossover (rad/s), well below the PLL
#define KP_SPEED (SPEED_CTRL_BANDWIDTH / SPEED_PLANT_GAIN)                 // Speed controller proportional gain (A per rad/s el)
#define KI_SPEED (KP_SPEED * SPEED_CTRL_BANDWIDTH * 0.25f)                 // Speed controller integral gain (PI zero at bandwidth/4)
#define SPEED_REF_SLEW 2000.0f                                             // Speed reference ramp (rad/s^2 el)
#ifndef V_BUS_MIN
#define V_BUS_MIN 8.0f                                                     // Minimum bus voltage to calibrate or start (V): supply off/low
#endif
#define CAL_PARAM_TOL 4.0f                                                 // Calibrated Rs/Ld/Lq accepted within RS,LS / tol .. * tol
#ifndef V_BUS_MAX
#define V_BUS_MAX 18.0f                                                   // Bus over-voltage trip (V): coast. Above a full 4S pack (16.8 V)
#endif

// ============================================================================
// Enumerations
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
    MOTOR_RUNNING
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

// ============================================================================
// Macros
// ============================================================================
#define CONSTRAIN(val, min, max) \
    ((val) <= (min) ? (min) : ((val) >= (max) ? (max) : (val)))

// ============================================================================
// DATA STRUCTURES
// ============================================================================
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

// Pass as output_limit to disable output clamping
#define PI_NO_OUTPUT_LIMIT (-1.0f)

// BLDC specifications structure
typedef struct
{
    uint32_t pole_pairs;
    float max_current;
    float Rs; // Stator resistance
    float Ld; // Stator inductance (d-axis)
    float Lq; // Stator inductance (q-axis)
    float Ls; // Stator inductance (average of Ld and Lq)
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

// FOC control structure
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
    bool inj_fault; // Set when injection aborted on over-current
    float rs_low_vd_sum, rs_low_id_sum; // Two-point Rs: sums recorded at INJECTION_VOLTAGE_RS_LOW
    uint16_t adc_v_a_buff[INJECTION_ROUNDS], adc_v_b_buff[INJECTION_ROUNDS];
    float i_a_buff[INJECTION_ROUNDS], i_b_buff[INJECTION_ROUNDS];
    float vd_inj_buff[INJECTION_ROUNDS]; // Buffer for injected voltages
    float vq_inj_buff[INJECTION_ROUNDS];
    float id_inj_buff[INJECTION_ROUNDS]; // Buffer for measured currents
    float iq_inj_buff[INJECTION_ROUNDS];
    uint32_t pwm_a_buff[INJECTION_ROUNDS], pwm_b_buff[INJECTION_ROUNDS], pwm_c_buff[INJECTION_ROUNDS];

    // Angle used by the Park / inverse Park transforms, set by the state machine:
    // align angle, forced angle, blended angle or observer angle. The observer's
    // own state (smo.theta_est) is never overwritten.
    float theta;           // Electrical angle for the transforms (rad, 0..2pi)
    float omega;           // Rate of theta (electrical rad/s), for the inverse-Park delay advance
    bool outputs_enabled;  // false: the HAL must disable the gate driver / PWM outputs (coast)

    // Temporary variables for FOC control
    float startup_timer;   // Timer for startup routines
    float startup_theta;   // Open-loop (forced) angle for startup
    float startup_omega;   // Open-loop (forced) speed for startup (electrical rad/s)
    float angle_offset;    // Transition: forced minus observer angle, decays to 0
    float backup_theta;    // Unused (kept for source compatibility)
    uint32_t bemf_counter; // Counter for back-EMF detection during startup
} FOC_t;

//===========================================================================
// FUNCTION PROTOTYPES
#ifdef __cplusplus
extern "C"
{
#endif

    // Utility functions

    // PI Controller functions
    void PI_Init(PI_Controller_t *controller, float Kp, float Ki, float sampling_time, float output_limit);
    void PI_Reset(PI_Controller_t *controller);
    void PI_Change_Params(PI_Controller_t *controller, float Kp, float Ki, float sampling_time);
    void PI_Set_Output_Limit(PI_Controller_t *controller, float output_limit);
    float PI_Update(PI_Controller_t *controller, float error);

    // BLDC parameter estimation functions
    void FOC_BLDC_Parameter_Measurement_Update(FOC_t *foc);
    void FOC_BLDC_Parameter_Calculation_Resistance(FOC_t *foc);
    void FOC_BLDC_Parameter_Calculation_Inductance(FOC_t *foc);

    // Trigonometric functions: table look-up (1024 samples/turn) + linear interpolation.
    // Frameless fast path for angles in [0, 2pi); other angles are wrapped first.
    void Init_Trigonometric_Tables(void);                           // MUST be called first (FOC_Init does)
    void Normalize_Angle(float *angle);                             // -> [0, 2pi); non-finite -> 0
    float Fast_Sin(float angle);                                    // < 5e-6 abs error
    float Fast_Cos(float angle);                                    // < 5e-6 abs error
    void Fast_Sin_Cos(float angle, float *sin_out, float *cos_out); // < 5e-6 abs error
    float Fast_Atan2(float y, float x);                             // < 2e-6 rad; C99 atan2 edge cases
    float Fast_Sqrt(float x); // Exact (VSQRT); 0 for x <= 0

    // FOC utility functions
    void SVPWM_Modulation(float v_alpha, float v_beta, float v_dc, uint32_t pwm_period,
                          uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w);
    void Clarke_Transform(float i_a, float i_b, float *i_alpha, float *i_beta);
    void Park_Transform(float i_alpha, float i_beta, float theta, float *i_d, float *i_q);
    void Clarke_Park_Transform(float i_a, float i_b, float theta, float *i_d, float *i_q);
    void Inverse_Park_Transform(float v_d, float v_q, float theta, float *v_alpha, float *v_beta);
    // i_a/i_b are in/out: they hold the LPF state, so pass persistent storage
    void Calculate_Phase_Currents(uint16_t adc_u, uint16_t adc_v, uint16_t offset_u, uint16_t offset_v, float *i_a, float *i_b);

    // Sliding-mode observer functions to estimate rotor position and speed
    void SMO_Init(SMO_t *smo, BLDC_Specs_t *bldc, float sampling_time);
    void SMO_Reset(SMO_t *smo);
    void SMO_BLDC_Specs_Update(SMO_t *smo, BLDC_Specs_t *bldc, float sampling_time);
    bool SMO_Update(SMO_t *smo, float v_bus, float v_alpha, float v_beta, float i_alpha_meas, float i_beta_meas); // Enhanced SMO + PLL; returns true while locked

    // FOC control functions
    void FOC_Init(FOC_t *foc, uint32_t pwm_res, uint32_t adc_offset);
    void FOC_Recalc_Params(FOC_t *foc); // Recalculate FOC parameters after BLDC specs update
    void FOC_Coast(FOC_t *foc);         // Disable the outputs (outputs_enabled = false) and drop to idle
    bool FOC_Set_Direction(FOC_t *foc, BLDC_Direction_t dir); // Bridge off only; false otherwise
    void FOC_MTPA(FOC_t *foc, float Is, float *Id_ref, float *Iq_ref);
    void FOC_Current_Loop_Update(FOC_t *foc);
    void FOC_BLDC_Startup_Update(FOC_t *foc);
    void FOC_Update(FOC_t *foc); // Update FOC states (to be called at the end of each PWM period)

#ifdef __cplusplus
} // extern "C"
#endif

#endif // FOC_LIB_H
