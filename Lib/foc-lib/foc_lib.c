#include <stdio.h>
#include <math.h>
#include "foc_lib.h"
#include <string.h> // memcpy (bit test in Trig_In_Range)

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
void PI_Init(PI_Controller_t *controller, float Kp, float Ki, float sampling_time, float output_limit)
{
    if (controller == NULL)
    {
        return;
    }

    PI_Change_Params(controller, Kp, Ki, sampling_time);
    PI_Set_Output_Limit(controller, output_limit);
}

// ---------------------------------------------------------------------------
FOC_FAST_CODE void PI_Reset(PI_Controller_t *controller)
{
    if (controller == NULL)
    {
        return;
    }

    controller->integral = 0.0f;
}

// ----------------------------------------------------------------------------
void PI_Change_Params(PI_Controller_t *controller, float Kp, float Ki, float sampling_time)
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
void PI_Set_Output_Limit(PI_Controller_t *controller, float output_limit)
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

float PI_Update(PI_Controller_t *controller, float error)
{
    if (controller == NULL)
    {
        return 0.0f;
    }

    return PI_Step(controller, error);
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
void Init_Trigonometric_Tables(void)
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
FOC_FAST_CODE void Normalize_Angle(float *angle)
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
FOC_FAST_CODE float Fast_Sin(float angle)
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
FOC_FAST_CODE float Fast_Cos(float angle)
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
FOC_FAST_CODE void Fast_Sin_Cos(float angle, float *sin_out, float *cos_out)
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

FOC_FAST_CODE float Fast_Atan2(float y, float x)
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
FOC_FAST_CODE float Fast_Sqrt(float x)
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

void SVPWM_Modulation(float v_alpha, float v_beta, float v_dc, uint32_t pwm_period,
                      uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w)
{
    SVPWM_Core(v_alpha, v_beta, v_dc, pwm_period, pwm_u, pwm_v, pwm_w);
}

// ---------------------------------------------------------------------------
// CLARKE TRANSFORM (abc → αβ)
// Simplified Clarke (only need 2 phase currents)
void Clarke_Transform(float i_a, float i_b, float *i_alpha, float *i_beta)
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

FOC_FAST_CODE void Park_Transform(float i_alpha, float i_beta, float theta, float *i_d, float *i_q)
{
    Park_Core(i_alpha, i_beta, theta, i_d, i_q);
}

FOC_FAST_CODE void Clarke_Park_Transform(float i_a, float i_b, float theta,
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

void Inverse_Park_Transform(float v_d, float v_q, float theta, float *v_alpha, float *v_beta)
{
    Inverse_Park_Core(v_d, v_q, theta, v_alpha, v_beta);
}

// ---------------------------------------------------------------------------
// Calculate the phase currents from ADC values
// NOTE: *i_a and *i_b are in/out. On entry they must hold the previous filtered
// value (they are the LPF state); on exit they hold the new filtered value.
// Callers must pass persistent storage (e.g. &foc->i_a, &foc->i_b).
void Calculate_Phase_Currents(uint16_t adc_u, uint16_t adc_v, uint16_t offset_u, uint16_t offset_v,
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
void SMO_Init(SMO_t *smo, BLDC_Specs_t *bldc, float sampling_time)
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
FOC_FAST_CODE void SMO_Reset(SMO_t *smo)
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
void SMO_BLDC_Specs_Update(SMO_t *smo, BLDC_Specs_t *bldc, float sampling_time)
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
FOC_FAST_CODE bool SMO_Update(SMO_t *smo, float v_bus, float v_alpha, float v_beta, float i_alpha_meas, float i_beta_meas)
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

void FOC_Init(FOC_t *foc, uint32_t pwm_res, uint32_t adc_offset)
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
    foc->backup_theta = 0.0f;
    foc->bemf_counter = 0;
    foc->adc_offset_a = adc_offset;
    foc->adc_offset_b = adc_offset;
    foc->adc_offset_c = adc_offset;

    // Initialize BLDC specifications
    foc->bldc.Rs = RS;
    foc->bldc.Ld = LS;
    foc->bldc.Lq = LS;
    foc->bldc.Ls = LS;
    foc->bldc.pole_pairs = POLE_PAIRS;
    foc->bldc.max_current = I_RATED;

    // Initialize motor mode
    foc->motor_mode = MOTOR_POWER_ON;

    // Initialize temporary variables
    foc->inj_n = 0;
    foc->inj_start_flag = false;
    foc->inj_fault = false;
    foc->inj_angle = 0.0f;
    foc->hfi_enable = false;
    foc->hfi_cnt = 0;

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
    foc->hfi_cnt = 0;

    PI_Reset(&(foc->id_ctrl));
    PI_Reset(&(foc->iq_ctrl));
    PI_Reset(&(foc->speed_ctrl));
    SMO_Reset(&(foc->smo));
    Harm_Reset(foc);
}

static void FOC_Speed_Control(FOC_t *foc);
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
        switch (foc->motor_mode)
        {
        case MOTOR_IDLE:
        case MOTOR_STARTUP_ALIGN:
        case MOTOR_STARTUP_RAMP:
        case MOTOR_STARTUP_TRANSITION:
            // Align -> I-f ramp -> transition (all current-controlled)
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
            Current_Loop_Core(foc);
            // Passive HFI: adds the square wave to v_d/v_q, leaves foc->theta alone.
            // Current_Loop_Core may have coasted (observer lost): then do nothing.
            if (foc->hfi_enable && foc->motor_mode == MOTOR_RUNNING)
            {
                FOC_HFI_Update(foc);
            }
            else
            {
                foc->hfi_cnt = 0;
            }
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
        foc->hfi_cnt = 0;
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
FOC_FAST_CODE void FOC_MTPA(FOC_t *foc, float Is, float *Id_ref, float *Iq_ref)
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
    float temp = Fast_Sqrt(FLUX_LINKAGE * FLUX_LINKAGE + foc->mtpa_k * Is_square);
    float id = (FLUX_LINKAGE - temp) * foc->mtpa_inv_4l;

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

// Closed-loop step, inlined into FOC_Update; FOC_Current_Loop_Update wraps it
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

    FOC_Current_Control(foc);
    FOC_Harmonic_Comp(foc);
}

// ---------------------------------------------------------------------------
// Passive HF-injection estimator (see HFI_VOLTAGE in foc_lib.h), after the current loop in
// MOTOR_RUNNING. The per-period current step i[k] - i[k-1] carries the + + - - square wave
// delayed by 1..2 periods (depending on when TIM1 latches the duty); its PWM_FREQ/4 phasor
// (reference e^(-j pi k / 2): 1, -j, -1, j, no trig) is taken along and across the applied axis.
// Only the perpendicular phasor's component in phase with the parallel one is used, so the
// unknown delay drops out, and the slow fundamental averages to zero over each 4 periods.
FOC_FAST_CODE void FOC_HFI_Update(FOC_t *foc)
{
    // Unfiltered currents: the measurement LPF attenuates the PWM_FREQ/4 component
    float ia = (float)((int32_t)foc->adc_v_a - (int32_t)foc->adc_offset_a) * ADC_2_CURRENT;
    float ib = (float)((int32_t)foc->adc_v_b - (int32_t)foc->adc_offset_b) * ADC_2_CURRENT;
    float i_al, i_be;
    Clarke_Transform(ia, ib, &i_al, &i_be);

    if (foc->hfi_cnt == 0)
    {
        foc->hfi_idx = 0;
        foc->hfi_delta = 0.0f;
        foc->hfi_n = 0;
        foc->hfi_zp[0] = foc->hfi_zp[1] = 0.0f;
        foc->hfi_zc[0] = foc->hfi_zc[1] = 0.0f;
        foc->hfi_zs[0] = foc->hfi_zs[1] = 0.0f;
    }
    else if (foc->hfi_cnt > HFI_DELAY) // The axis history is filled
    {
        float d_al = i_al - foc->hfi_ia_h;
        float d_be = i_be - foc->hfi_ib_h;
        float s, c;
        Fast_Sin_Cos(foc->hfi_phi_h[HFI_DELAY - 1], &s, &c);
        float par = d_al * c + d_be * s;
        float perp = -d_al * s + d_be * c;
        float d = foc->hfi_delta_h[HFI_DELAY - 1];
        Fast_Sin_Cos(2.0f * d, &s, &c);
        float pc = perp * c, ps = perp * s;

        // Multiply by e^(-j pi idx / 2): one of 1, -j, -1, j
        switch (foc->hfi_idx)
        {
        case 0:
            foc->hfi_zp[0] += par;
            foc->hfi_zc[0] += pc;
            foc->hfi_zs[0] += ps;
            break;
        case 1:
            foc->hfi_zp[1] -= par;
            foc->hfi_zc[1] -= pc;
            foc->hfi_zs[1] -= ps;
            break;
        case 2:
            foc->hfi_zp[0] -= par;
            foc->hfi_zc[0] -= pc;
            foc->hfi_zs[0] -= ps;
            break;
        default:
            foc->hfi_zp[1] += par;
            foc->hfi_zc[1] += pc;
            foc->hfi_zs[1] += ps;
            break;
        }

        if (++foc->hfi_n >= HFI_WINDOW)
        {
            // Project the perpendicular phasors on the parallel one's direction. Then, as for an
            // in-phase signal: perp ~ -K * par * sin(2 (delta - e)) over a full 0..pi sweep gives
            // sc = |Zp| K sin(2e) / 2, ss = -|Zp| K cos(2e) / 2.
            float zp = Fast_Sqrt(foc->hfi_zp[0] * foc->hfi_zp[0] + foc->hfi_zp[1] * foc->hfi_zp[1]);
            if (zp > 1e-6f)
            {
                float ur = foc->hfi_zp[0] / zp, ui = foc->hfi_zp[1] / zp;
                float sc = foc->hfi_zc[0] * ur + foc->hfi_zc[1] * ui;
                float ss = foc->hfi_zs[0] * ur + foc->hfi_zs[1] * ui;
                foc->hfi_k = 2.0f * Fast_Sqrt(sc * sc + ss * ss) / zp;
                foc->hfi_err = 0.5f * Fast_Atan2(sc, -ss);
                foc->hfi_phase = Fast_Atan2(foc->hfi_zp[1], foc->hfi_zp[0]);
            }
            else
            {
                foc->hfi_k = 0.0f;
            }
            // A step of +-A held 2 periods has a PWM_FREQ/4 phasor of sqrt(2)/2 * A per sample
            foc->hfi_apar = zp * (1.41421356f / (float)HFI_WINDOW);
            foc->hfi_windows++;
            foc->hfi_n = 0;
            foc->hfi_zp[0] = foc->hfi_zp[1] = 0.0f;
            foc->hfi_zc[0] = foc->hfi_zc[1] = 0.0f;
            foc->hfi_zs[0] = foc->hfi_zs[1] = 0.0f;
        }
    }

    // Inject along control d-axis + delta; remember the applied alpha-beta axis
    float s, c;
    Fast_Sin_Cos(foc->hfi_delta, &s, &c);
    const float v = (foc->hfi_idx < 2u) ? HFI_VOLTAGE : -HFI_VOLTAGE;
    foc->v_d += v * c;
    foc->v_q += v * s;
    for (int j = HFI_DELAY - 1; j > 0; j--)
    {
        foc->hfi_phi_h[j] = foc->hfi_phi_h[j - 1];
        foc->hfi_delta_h[j] = foc->hfi_delta_h[j - 1];
    }
    foc->hfi_phi_h[0] = foc->theta + foc->ipark_adv * foc->omega + foc->hfi_delta;
    foc->hfi_delta_h[0] = foc->hfi_delta;

    foc->hfi_ia_h = i_al;
    foc->hfi_ib_h = i_be;

    foc->hfi_delta += PI / (float)HFI_WINDOW;
    if (foc->hfi_delta >= PI)
    {
        foc->hfi_delta -= PI;
    }
    foc->hfi_idx = (foc->hfi_idx + 1u) & 3u;
    if (foc->hfi_cnt < 0xFFFFFFFFu)
    {
        foc->hfi_cnt++;
    }
}

void FOC_Current_Loop_Update(FOC_t *foc)
{
    if (foc == NULL)
    {
        return;
    }

    Current_Loop_Core(foc);
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

    float iq = PI_Step(&(foc->speed_ctrl), foc->speed_ref_ramp - dir * foc->smo.omega_est);
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
FOC_FAST_CODE void FOC_BLDC_Startup_Update(FOC_t *foc)
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
        break;

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
                         EMF_PLAUSIBLE_RATIO * fabsf(foc->startup_omega) * FLUX_LINKAGE;
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
        // printf("ADC_Va_buff = %d,ADC_Vb_buff = %d, Ia = %f, Ib = %f, Vd = %f, Id = %f\r\n", foc->adc_v_a_buff[i], foc->adc_v_b_buff[i], foc->i_a_buff[i], foc->i_b_buff[i],
        //        foc->vd_inj_buff[i], foc->id_inj_buff[i]);
        mean_vd += foc->vd_inj_buff[i];
        mean_id += foc->id_inj_buff[i];
    }
    mean_vd /= INJECTION_ROUNDS;
    mean_id /= INJECTION_ROUNDS;

    // printf("\r\nMean Vd = %f, Id = %f\r\n", mean_vd, mean_id);

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
    if (rs < RS / CAL_PARAM_TOL || rs > RS * CAL_PARAM_TOL)
    {
        printf("\r\nRs %f implausible (default %f): keeping %f\r\n", rs, RS, foc->bldc.Rs);
        return;
    }
    foc->bldc.Rs = rs;
}

// ---------------------------------------------------------------------------
// BLDC inductance calculation after measurement
void FOC_BLDC_Parameter_Calculation_Inductance(FOC_t *foc)
{
    float vd, vq, id, iq;
    float Vc_d = 0.0f, Vs_d = 0.0f, Ic_d = 0.0f, Is_d = 0.0f;
    float Vc_q = 0.0f, Vs_q = 0.0f, Ic_q = 0.0f, Is_q = 0.0f;
    float mean_vd = 0.0f, mean_vq = 0.0f, mean_id = 0.0f, mean_iq = 0.0f;

    if (foc == NULL)
    {
        return;
    }

    // Calculate DC offset
    for (int i = 0; i < INJECTION_ROUNDS; i++)
    {
        mean_vd += foc->vd_inj_buff[i];
        mean_vq += foc->vq_inj_buff[i];
        mean_id += foc->id_inj_buff[i];
        mean_iq += foc->iq_inj_buff[i];
    }
    mean_vd /= INJECTION_ROUNDS;
    mean_vq /= INJECTION_ROUNDS;
    mean_id /= INJECTION_ROUNDS;
    mean_iq /= INJECTION_ROUNDS;

    // Single-frequency DFT
    float angle;
    for (int i = 0; i < INJECTION_ROUNDS; i++)
    {
        angle = INJECTION_OMEGA * i * foc->sampling_time;

        // Remove DC offset
        vd = foc->vd_inj_buff[i] - mean_vd;
        vq = foc->vq_inj_buff[i] - mean_vq;
        id = foc->id_inj_buff[i] - mean_id;
        iq = foc->iq_inj_buff[i] - mean_iq;

        Vc_d += vd * Fast_Cos(angle);
        Vs_d += vd * Fast_Sin(angle);
        Ic_d += id * Fast_Cos(angle);
        Is_d += id * Fast_Sin(angle);

        Vc_q += vq * Fast_Cos(angle);
        Vs_q += vq * Fast_Sin(angle);
        Ic_q += iq * Fast_Cos(angle);
        Is_q += iq * Fast_Sin(angle);
    }

    const float norm = 2.0f / INJECTION_ROUNDS;
    Vc_d *= norm;
    Vs_d *= norm;
    Ic_d *= norm;
    Is_d *= norm;
    Vc_q *= norm;
    Vs_q *= norm;
    Ic_q *= norm;
    Is_q *= norm;

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

    // Reject implausible results (e.g. no bus voltage: ~35x LS was accepted once and set the
    // current-loop gains from it)
    const float l_min = LS / CAL_PARAM_TOL, l_max = LS * CAL_PARAM_TOL;
    if (fabsf(Ld_est) < l_min || fabsf(Ld_est) > l_max || fabsf(Lq_est) < l_min || fabsf(Lq_est) > l_max)
    {
        printf("\r\nLd/Lq implausible (default LS %e): keeping Ld %e, Lq %e\r\n", LS, foc->bldc.Ld, foc->bldc.Lq);
        return;
    }

    foc->bldc.Ld = fabsf(Ld_est);
    foc->bldc.Lq = fabsf(Lq_est);
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
    float mv = 0.0f, mi = 0.0f, mx = 0.0f;
    for (int i = 0; i < INJECTION_ROUNDS; i++)
    {
        mv += foc->vd_inj_buff[i];
        mi += foc->id_inj_buff[i];
        mx += foc->iq_inj_buff[i];
    }
    mv /= INJECTION_ROUNDS;
    mi /= INJECTION_ROUNDS;
    mx /= INJECTION_ROUNDS;

    // Single-frequency DFT (as in FOC_BLDC_Parameter_Calculation_Inductance)
    float Vc = 0.0f, Vs = 0.0f, Ic = 0.0f, Is = 0.0f, Xc = 0.0f, Xs = 0.0f;
    for (int i = 0; i < INJECTION_ROUNDS; i++)
    {
        float angle = INJECTION_OMEGA * i * foc->sampling_time;
        float c = Fast_Cos(angle), s = Fast_Sin(angle);
        Vc += (foc->vd_inj_buff[i] - mv) * c;
        Vs += (foc->vd_inj_buff[i] - mv) * s;
        Ic += (foc->id_inj_buff[i] - mi) * c;
        Is += (foc->id_inj_buff[i] - mi) * s;
        Xc += (foc->iq_inj_buff[i] - mx) * c;
        Xs += (foc->iq_inj_buff[i] - mx) * s;
    }

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
FOC_FAST_CODE void FOC_BLDC_Parameter_Measurement_Update(FOC_t *foc)
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

        // The Rs run holds two DC levels: low for RS_ALIGN_ROUNDS (the rotor swings into
        // alignment; its back-EMF would bias the reading) + INJECTION_ROUNDS recorded, then high
        // for INJECTION_ROUNDS settling + INJECTION_ROUNDS recorded. The Ld/Lq runs have one
        // settle + record block.
        const bool rs_run = (foc->motor_mode == MOTOR_ESTIMATION_RS);
        const uint32_t rs_low_end = RS_ALIGN_ROUNDS + INJECTION_ROUNDS;
        const uint32_t rounds_total = rs_run ? rs_low_end + 2u * INJECTION_ROUNDS : 2u * INJECTION_ROUNDS;

        if (rs_run)
        {
            if (foc->inj_n == 0)
            {
                foc->rs_low_vd_sum = 0.0f;
                foc->rs_low_id_sum = 0.0f;
            }
            foc->v_d = (foc->inj_n < rs_low_end) ? INJECTION_VOLTAGE_RS_LOW : INJECTION_VOLTAGE;
            foc->v_q = 0.0f;
        }
        else
        {
            float angle = INJECTION_OMEGA * foc->inj_n * foc->sampling_time;
            float v_inj = INJECTION_VOLTAGE * Fast_Sin(angle);
            if (foc->motor_mode == MOTOR_ESTIMATION_LD)
            {
                foc->v_d = v_inj;
                foc->v_q = 0.0f;
            }
            else if (foc->motor_mode == MOTOR_ESTIMATION_LQ)
            {
                foc->v_d = 0.0f;
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

        // Accumulate injected voltages and measured currents after certain rounds
        if (rs_run && foc->inj_n >= RS_ALIGN_ROUNDS && foc->inj_n < rs_low_end)
        {
            // Low DC level of the Rs run: only the means are needed
            foc->rs_low_vd_sum += foc->v_d;
            foc->rs_low_id_sum += foc->i_d;
        }
        else if (foc->inj_n >= rounds_total - INJECTION_ROUNDS)
        {
            uint32_t index = foc->inj_n - (rounds_total - INJECTION_ROUNDS);
            foc->pwm_a_buff[index] = foc->pwm_a;
            foc->pwm_b_buff[index] = foc->pwm_b;
            foc->pwm_c_buff[index] = foc->pwm_c;
            foc->adc_v_a_buff[index] = foc->adc_v_a;
            foc->adc_v_b_buff[index] = foc->adc_v_b;
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
