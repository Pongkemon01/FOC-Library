/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : foc-lib demo / bench application for the STM32F303RE port
 *                      (Nucleo-F303RE + X-NUCLEO-IHM08M1)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// How this demo uses foc-lib (see the porting guide at the top of foc_lib.h):
//
//   MX_FREERTOS_Init     TIM1 at PWM_FREQ (TIMER_config), FOC_Init, start the ADC interrupt.
//   ISR (ADC injected    HAL_ADCEx_InjectedConvCpltCallback: ADC counts -> FOC_Update -> TIM1
//   end of sequence)     compare registers, and the bridge on/off for foc.outputs_enabled.
//   StartDefaultTask     1. PWM on, measure the zero-current offsets (motor at rest)
//                        2. check the bus voltage
//                        3. calibrate Rs, Ld, Lq (optional, FOC_Recalc_Params after it)
//                        4. saliency sweep (diagnostic, RUN_SALIENCY_SWEEP)
//                        5. speed test with a log (RUN_FOC_TEST, TEST_PROFILE)
//
// The control runs entirely in the ISR (current loop, observers, startup, speed loop at 1 kHz):
// a task only sets targets (foc.speed_ref / foc.is_ref) and reads the state. Keep printf and
// anything blocking out of the ISR.
#include <stdio.h>
#include <math.h>
#include "usart.h"
#include "tim.h"
#include "adc.h"
#include "foc_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ---- Demo options ----------------------------------------------------------
#define RUN_SALIENCY_SWEEP 1 // 1: after calibration, Ld-type injection at SWEEP_POINTS angles (diagnostic)
#define SWEEP_POINTS 24
#define RUN_FOC_TEST 1 // 1: run the speed test after calibration; 0: stop there
#define RUN_DIRECTION NORMAL_DIRECTION // NORMAL_DIRECTION or REVERSE_DIRECTION
#define TEST_I_MAX 5.0f // Current cap during the test (A). Recommended on a bench: well below I_RATED,
                        // and keep the input power below what the supply can deliver
#define TEST_PROFILE 5 // Speed reference over time (rad/s el), see Test_Speed_Ref:
                       // 0: 1500, 2500 from 3 s
                       // 1: gentle staircase for a propeller load: 1500, ramps to 2000 and 2500
                       //    (the bench 12 V 5 A supply collapses at ~20 W: slow ramps, no 3000)
                       // 2: 1500, 150 from 2.5 s (coasts down through eSMO -> HFI), 0 from 8.5 s,
                       //    1500 from 10 s
                       // 3: slow sweep down for HFI_SHADOW_TEST: 1500, then 800 -> 100 at 100 rad/s^2
                       // 4: HFI from standstill: 150, 0 from 5 s, 150 from 6.5 s, 1500 from 8 s
                       // 5: low-speed I-f steps: 1500, 150 from 2 s, 75, 50, 20, 0 every 1.5 s,
                       //    150 from 9.5 s, 1500 from 11 s
#define HFI_SHADOW_TEST 0 // Test only. 1: measure the eSMO's low-speed limit (with TEST_PROFILE 3,
                          // then tools/hfi_threshold.py on the log): Align -> I-f start, the HFI
                          // tracker is logged beside the eSMO but never takes control
#ifndef ADC_INJ_SMP
#define ADC_INJ_SMP 0 // Injected-channel ADC sampling time code: 0 = keep CubeMX (1.5 cycles), 4 = 19.5, 5 = 61.5
#endif

#define OFFSET_SAMPLES 64 // Zero-current offset: average of this many samples, 1 ms apart
#define APP_PI 3.14159265359f // For log conversions (foc-lib keeps its constants internal)
#define APP_TWO_PI 6.28318530718f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
FOC_t foc; // The drive. Written by FOC_Update in the ISR; tasks set targets and read state.

// ISR execution time in CPU cycles (DWT cycle counter, 72 MHz). Budget: 72e6 / PWM_FREQ per period
// (3600 at 20 kHz). isr_cyc: the callback alone. irq_cyc: from ADC1_2_IRQHandler entry (stamped
// into irq_cyc0 in stm32f3xx_it.c) to the end of the callback, i.e. with the dispatch.
// PC8 is also high for the callback's duration (scope).
static volatile uint32_t isr_cyc_last, isr_cyc_max;
volatile uint32_t irq_cyc0;
static volatile uint32_t irq_cyc_last, irq_cyc_max;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// Recommendation: the default task has 512 B of stack (set in CubeMX / stm32-foc.ioc). printf with
// floats uses much of it, which is why the log prints integers in short printf calls. Raise the
// task's stack in CubeMX before adding deeper code here.

// ============================================================================
// Port layer: console, PWM timer, bridge
// ============================================================================

// printf -> USART2 (ST-Link virtual COM port, 230400 8N1)
#ifndef __PLATFORMIO__
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
#else
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}
#endif

// Timer input clock: the APB clock, doubled when the APB prescaler is not 1
static uint32_t get_timer_clock(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 || htim->Instance == TIM8)
    {
        uint32_t pclk = HAL_RCC_GetPCLK2Freq(); // On APB2
        return ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) ? 2 * pclk : pclk;
    }
    uint32_t pclk = HAL_RCC_GetPCLK1Freq(); // On APB1
    return ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) ? 2 * pclk : pclk;
}

// Set TIM1 (centre-aligned) to pwm_freq and store its period in foc->pwm_resolution, which
// FOC_Init then takes as 100 % duty. This overrides the CubeMX PSC/ARR at run time.
int TIMER_config(TIM_HandleTypeDef *timer, FOC_t *foc, float pwm_freq)
{
    if (foc == NULL || timer == NULL)
        return 0;

    const uint32_t timer_clock = get_timer_clock(timer);

    // Centre-aligned: the counter goes up and down, so one PWM period is 2 * (ARR + 1) clocks
    uint32_t prescaler = 0;
    uint32_t period = (timer_clock / (2 * pwm_freq)) - 1;
    if (period > 0xFFFF)
    {
        prescaler = period / 0xFFFF;
        period = (timer_clock / (2 * pwm_freq * (prescaler + 1))) - 1;
    }
    timer->Instance->PSC = prescaler;
    timer->Instance->ARR = period;

    // Channel 4 triggers the injected ADC conversion once per period
    timer->Instance->CCR4 = period - 1;

    // An over-current break (MOE = 0) must turn all six switches off. The CubeMX config leaves
    // OISxN = 0, which with the IHM08M1's active-low L6398 low-side inputs turns every low side ON
    // (brakes a spinning motor through the bridge), and AOE = 1, which re-sets MOE at every update
    // event and undoes the break. Fixed here so it survives regeneration. A break also stops the
    // OC4 ADC trigger, so the control ISR halts with the bridge off until reset.
    timer->Instance->CR2 |= TIM_CR2_OIS1N | TIM_CR2_OIS2N | TIM_CR2_OIS3N;
    timer->Instance->BDTR &= ~TIM_BDTR_AOE;

    foc->pwm_resolution = period;
    return 1;
}

// Start the three PWM pairs and the ADC trigger channel. The bridge itself stays off until
// FOC_Update enables the outputs (Bridge_Outputs below).
void Start_PWM(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    htim1.Instance->CCR4 = htim1.Instance->ARR - 1; // ADC trigger

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

// Bridge on/off for foc.outputs_enabled. Off switches the six gate pins from TIM1 (AF6) to GPIO
// outputs at their inactive levels: HIN (PA8/9/10) low, LIN (PA7, PB0, PB1) high, since the
// IHM08M1's L6398 low-side input is active-low. MOE is NOT used for this: the injected ADC is
// triggered by the OC4 output, which MOE also gates, so clearing MOE stops the control ISR.
static void Bridge_Outputs(bool enable)
{
    const uint32_t mask_a = (3u << (7 * 2)) | (3u << (8 * 2)) | (3u << (9 * 2)) | (3u << (10 * 2));
    const uint32_t mask_b = (3u << (0 * 2)) | (3u << (1 * 2));

    if (enable)
    {
        GPIOA->MODER = (GPIOA->MODER & ~mask_a) | (0xAAAAAAAAu & mask_a); // 10 = alternate function
        GPIOB->MODER = (GPIOB->MODER & ~mask_b) | (0xAAAAAAAAu & mask_b);
    }
    else
    {
        GPIOA->BSRR = GPIO_PIN_7 | ((uint32_t)(GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10) << 16);
        GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
        GPIOA->MODER = (GPIOA->MODER & ~mask_a) | (0x55555555u & mask_a); // 01 = general output
        GPIOB->MODER = (GPIOB->MODER & ~mask_b) | (0x55555555u & mask_b);
    }
}

// ============================================================================
// Demo steps (task context)
// ============================================================================

// Wait for a calibration run started with FOC_Start_Measurement. Returns false on an
// over-current abort.
static bool Wait_Measurement(void)
{
    while (FOC_Measurement_Busy(&foc))
        osDelay(1);
    return !foc.inj_fault;
}

// Step 1: zero-current offsets of the current-sense chain. The motor must be at rest and the
// bridge off (it is after FOC_Init). Averaging removes the ADC noise of a single sample.
static void Measure_Current_Offsets(void)
{
    uint32_t sum_a = 0, sum_b = 0, sum_c = 0;
    for (int i = 0; i < OFFSET_SAMPLES; i++)
    {
        osDelay(1);
        sum_a += foc.adc_v_a;
        sum_b += foc.adc_v_b;
        sum_c += foc.adc_v_c;
    }
    foc.adc_offset_a = (uint16_t)((sum_a + OFFSET_SAMPLES / 2) / OFFSET_SAMPLES);
    foc.adc_offset_b = (uint16_t)((sum_b + OFFSET_SAMPLES / 2) / OFFSET_SAMPLES);
    foc.adc_offset_c = (uint16_t)((sum_c + OFFSET_SAMPLES / 2) / OFFSET_SAMPLES);
    printf("Current-sense offsets (ADC counts): A %d, B %d, C %d\r\n", foc.adc_offset_a, foc.adc_offset_b,
           foc.adc_offset_c);
}

// Step 3: Rs, then Ld and Lq, at standstill. Each run injects at most INJECTION_MAX_CURRENT.
// Recommendation: run it once per motor and keep the results (foc.bldc) if the start-up time
// matters; FOC_Init's defaults are the foc_lib.h values.
static void Calibrate(void)
{
    printf("Measuring Rs...\r\n");
    FOC_Start_Measurement(&foc, MOTOR_ESTIMATION_RS, 0.0f);
    // The library ramps the voltage until the current reaches CAL_CURRENT: no motor constant needed
    if (Wait_Measurement())
        FOC_BLDC_Parameter_Calculation_Resistance(&foc);
    else
        printf("  !! aborted (over %f A, or %f A not reached: no bus / open winding): Rs not updated\r\n",
               (double)INJECTION_MAX_CURRENT, (double)CAL_CURRENT);
    printf("  Rs = %f ohm (ramp reached %f V)\r\n", foc.bldc.Rs, foc.inj_v);

    // Current-sign check: the Rs run is DC on phase A (v_d > 0 at theta = 0), so a correct sense
    // chain gives i_a > 0 and i_b ~ -i_a / 2. The calibration itself cannot see a wrong
    // CURRENT_SENSE_POLARITY (it uses magnitudes). A drift between the first and last samples
    // means the rotor was still moving while Rs was recorded.
    float m_id = 0.0f, m_ia = 0.0f, m_ib = 0.0f, first = 0.0f, last = 0.0f;
    for (int i = 0; i < INJECTION_ROUNDS; i++)
    {
        m_id += foc.id_inj_buff[i];
        m_ia += foc.i_a_buff[i];
        m_ib += foc.i_b_buff[i];
    }
    for (int i = 0; i < 32; i++)
    {
        first += foc.id_inj_buff[i];
        last += foc.id_inj_buff[INJECTION_ROUNDS - 32 + i];
    }
    printf("  signed means: i_d %f, i_a %f, i_b %f A (expect i_a > 0, i_b ~ -i_a/2)\r\n",
           m_id / INJECTION_ROUNDS, m_ia / INJECTION_ROUNDS, m_ib / INJECTION_ROUNDS);
    printf("  i_d first/last 32 samples: %f / %f A\r\n", first / 32.0f, last / 32.0f);

    printf("Measuring Ld, Lq...\r\n");
    FOC_Start_Measurement(&foc, MOTOR_ESTIMATION_LD, 0.0f);
    bool ok = Wait_Measurement();
    FOC_Start_Measurement(&foc, MOTOR_ESTIMATION_LQ, 0.0f);
    ok = Wait_Measurement() && ok;
    if (ok)
        FOC_BLDC_Parameter_Calculation_Inductance(&foc);
    else
        printf("  !! aborted on over-current: Ld/Lq not updated\r\n");
    printf("  Ld = %e H, Lq = %e H\r\n", foc.bldc.Ld, foc.bldc.Lq);

    FOC_Recalc_Params(&foc); // Apply the measured values to the current loops, the observer and HFI
    printf("  HFI amplitude %f V\r\n", foc.hfi_v);
}

#if RUN_SALIENCY_SWEEP
// Step 4 (diagnostic): saliency map. The 1 kHz Ld-type injection, applied along SWEEP_POINTS
// angles with the rotor at rest (left where the Rs run aligned it). Fit L(phi) = A0 +
// A1 cos(phi - p1) + A2 cos(2 phi - 2 p2): A2 is the saliency HFI relies on (min L on the
// rotor's d-axis, here near 0 or 180 deg), A1 should be ~0 (the rotor did not move).
static void Saliency_Sweep(void)
{
    static float L[SWEEP_POINTS], xr[SWEEP_POINTS], xi[SWEEP_POINTS];
    printf("Saliency sweep (%d points, 1 kHz, %f V):\r\n  deg  L_axis(nH)  cross_re(x1e4)  cross_im(x1e4)\r\n",
           SWEEP_POINTS, (double)(CAL_CURRENT * foc.bldc.Rs));
    for (int k = 0; k < SWEEP_POINTS; k++)
    {
        FOC_Start_Measurement(&foc, MOTOR_ESTIMATION_LD, (float)k * (APP_TWO_PI / SWEEP_POINTS));
        if (!Wait_Measurement() || !FOC_BLDC_Saliency_Point(&foc, &L[k], &xr[k], &xi[k]))
        {
            printf("  !! sweep aborted (over-current or no current)\r\n");
            return;
        }
        printf("  %3d  %6d  %6d  %6d\r\n", k * 360 / SWEEP_POINTS, (int)(L[k] * 1e9f), (int)(xr[k] * 1e4f),
               (int)(xi[k] * 1e4f));
    }

    float s0 = 0, c1 = 0, s1 = 0, c2 = 0, s2 = 0, xc2 = 0, xs2 = 0;
    for (int k = 0; k < SWEEP_POINTS; k++)
    {
        const float p = (float)k * (APP_TWO_PI / SWEEP_POINTS);
        s0 += L[k];
        c1 += L[k] * cosf(p);
        s1 += L[k] * sinf(p);
        c2 += L[k] * cosf(2.0f * p);
        s2 += L[k] * sinf(2.0f * p);
        xc2 += xr[k] * cosf(2.0f * p);
        xs2 += xr[k] * sinf(2.0f * p);
    }
    const float n = (float)SWEEP_POINTS;
    const float A0 = s0 / n, A1 = 2.0f * sqrtf(c1 * c1 + s1 * s1) / n, A2 = 2.0f * sqrtf(c2 * c2 + s2 * s2) / n;
    const float p2 = 0.5f * atan2f(s2, c2); // Angle of max L (q-axis)
    const float X2 = 2.0f * sqrtf(xc2 * xc2 + xs2 * xs2) / n;

    float res = 0.0f; // Residual after the 0th..2nd harmonics: the measurement noise floor
    for (int k = 0; k < SWEEP_POINTS; k++)
    {
        const float p = (float)k * (APP_TWO_PI / SWEEP_POINTS);
        const float fit = A0 + 2.0f / n * (c1 * cosf(p) + s1 * sinf(p) + c2 * cosf(2.0f * p) + s2 * sinf(2.0f * p));
        res += (L[k] - fit) * (L[k] - fit);
    }
    res = sqrtf(res / n);

    printf("  fit: L0 %d nH, A2 %d nH (min-L axis %d deg el, mod 180), A1 %d nH, residual rms %d nH\r\n",
           (int)(A0 * 1e9f), (int)(A2 * 1e9f), (int)((p2 + 0.5f * APP_PI) * (180.0f / APP_PI)) % 180,
           (int)(A1 * 1e9f), (int)(res * 1e9f));
    printf("  -> Ld ~ %d nH, Lq ~ %d nH; cross-axis 2nd harmonic %d x1e-4 of the axis current\r\n",
           (int)((A0 - A2) * 1e9f), (int)((A0 + A2) * 1e9f), (int)(X2 * 1e4f));
}
#endif

#if RUN_FOC_TEST
// Test profiles (TEST_PROFILE): the speed reference at t_ms after the start (rad/s el, in
// RUN_DIRECTION) and the number of 20 ms log rows. The library ramps speed_ref at SPEED_REF_SLEW,
// and the drive is motoring only: a lower reference lets the motor coast down by its load.
#if TEST_PROFILE == 0
#define TEST_ROWS 250
static float Test_Speed_Ref(uint32_t t_ms) { return (t_ms >= 3000) ? 2500.0f : 1500.0f; }
#elif TEST_PROFILE == 1
#define TEST_ROWS 400
static float Test_Speed_Ref(uint32_t t_ms)
{
    if (t_ms < 2500)
        return 1500.0f;
    if (t_ms < 3500)
        return 1500.0f + 0.5f * (float)(t_ms - 2500); // 500 rad/s^2
    if (t_ms < 4500)
        return 2000.0f;
    if (t_ms < 6500)
        return 2000.0f + 0.25f * (float)(t_ms - 4500); // 250 rad/s^2
    return 2500.0f;
}
#elif TEST_PROFILE == 2
#define TEST_ROWS 600
static float Test_Speed_Ref(uint32_t t_ms)
{
    return (t_ms >= 10000) ? 1500.0f : (t_ms >= 8500) ? 0.0f : (t_ms >= 2500) ? 150.0f : 1500.0f;
}
#elif TEST_PROFILE == 3
#define TEST_ROWS 550
static float Test_Speed_Ref(uint32_t t_ms)
{
    if (t_ms < 2500)
        return 1500.0f;
    if (t_ms < 9500)
        return 800.0f - 0.1f * (float)(t_ms - 2500); // 100 rad/s^2
    return 100.0f;
}
#elif TEST_PROFILE == 5
#define TEST_ROWS 650
static float Test_Speed_Ref(uint32_t t_ms)
{
    return (t_ms >= 11000) ? 1500.0f : (t_ms >= 9500) ? 150.0f : (t_ms >= 8000) ? 0.0f
         : (t_ms >= 6500) ? 20.0f : (t_ms >= 5000) ? 50.0f : (t_ms >= 3500) ? 75.0f
         : (t_ms >= 2000) ? 150.0f : 1500.0f;
}
#else // 4
#define TEST_ROWS 550
static float Test_Speed_Ref(uint32_t t_ms)
{
    return (t_ms >= 8000) ? 1500.0f : (t_ms >= 6500) ? 150.0f : (t_ms >= 5000) ? 0.0f : 150.0f;
}
#endif

// One log row. Integers only (rad/s, mV, mA, 0.1 deg), in two short printf calls: the task stack
// is small. Columns as printed by Speed_Test (tools/hfi_threshold.py reads them by name).
static void Log_Row(uint32_t t_ms)
{
    printf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", (int)t_ms, (int)foc.motor_mode,
           (int)foc.startup_omega, (int)foc.speed_ref_ramp, (int)foc.smo.omega_est,
           (int)(foc.smo.emf_magnitude * 1000.0f), (int)foc.smo.locked, (int)foc.bemf_counter,
           (int)(foc.i_d * 1000.0f), (int)(foc.i_q * 1000.0f), (int)(foc.i_a * 1000.0f), (int)(foc.i_b * 1000.0f),
           (int)(foc.v_d * 1000.0f), (int)(foc.v_q * 1000.0f), (int)(foc.v_bus * 1000.0f),
           (TIM1->SR & TIM_SR_BIF) ? 1 : 0, (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0, (int)isr_cyc_last,
           (int)irq_cyc_last);

    float smo_hfi = foc.smo.theta_est - foc.hfi_theta; // eSMO minus HFI angle, wrapped to +-pi
    if (smo_hfi > APP_PI)
        smo_hfi -= APP_TWO_PI;
    else if (smo_hfi < -APP_PI)
        smo_hfi += APP_TWO_PI;
    printf(" %d %d %d %d %d %d %d\r\n", (int)foc.hfi_on, (int)foc.hfi_control, (int)(smo_hfi * (1800.0f / APP_PI)),
           (int)foc.hfi_omega, (int)(foc.hfi_zp * 1000.0f), (int)(foc.hfi_err * (1800.0f / APP_PI)),
           (int)foc.hfi_fault);
}

// Step 5: speed test. Start, follow the TEST_PROFILE reference, log at ~50 Hz, stop, and check
// that the bridge really lets the motor coast.
static void Speed_Test(void)
{
    // The calibration leaves the bridge energized; the direction can only be set with it off.
    FOC_Stop(&foc);
    osDelay(2); // The ISR turns the outputs off within one PWM period
    if (!FOC_Set_Direction(&foc, RUN_DIRECTION))
        printf("!! FOC_Set_Direction refused (bridge energized)\r\n");
    FOC_Set_Current_Limit(&foc, TEST_I_MAX);
    foc.hfi_takeover = !HFI_SHADOW_TEST; // Test only (see HFI_SHADOW_TEST)
    foc.speed_ref = Test_Speed_Ref(0);
    // Torque mode instead: foc.is_ref = <A>; FOC_Start(&foc, FOC_TORQUE_CONTROL). Unloaded, a
    // motor in torque mode accelerates until the observer's speed limit: use a load or a speed cap.

    // brk = TIM1 break flag (the IHM08M1 over-current comparator latched the outputs off),
    // moe = TIM1 main output enable. Clear a stale break flag before the run (rc_w0).
    printf("Starting FOC control (before: brk %d, moe %d)\r\n", (TIM1->SR & TIM_SR_BIF) ? 1 : 0,
           (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0);
    TIM1->SR = ~TIM_SR_BIF;
    isr_cyc_max = 0;
    irq_cyc_max = 0;
    const uint32_t t0 = osKernelGetTickCount();
    FOC_Start(&foc, FOC_SPEED_CONTROL);

    // Columns: t_ms, motor_mode (5 align, 6 I-f, 7 transition, 8 running, 9 HFI start), forced
    // speed, ramped reference, eSMO speed, back-EMF (mV), eSMO lock, lock counter, id/iq/ia/ib (mA),
    // vd/vq (mV), vbus (mV), brk, moe, ISR/IRQ cycles; HFI: on = tracker running, ctl = HFI in
    // control, smo_hfi = eSMO minus HFI angle (0.1 deg), w_hfi = HFI speed, zp = HF response (mA),
    // err = tracker error (0.1 deg), flt = HFI fault.
    printf("t_ms mode w_force w_ref w_est emf_mV lock cnt id_mA iq_mA ia_mA ib_mA vd_mV vq_mV vbus_mV brk moe "
           "isr_cyc irq_cyc on ctl smo_hfi w_hfi zp err flt\r\n");
    BLDC_Mode_t last_mode = foc.motor_mode;
    bool stopped = false;
    for (int k = 0; k < TEST_ROWS; k++)
    {
        const uint32_t t_ms = osKernelGetTickCount() - t0;
        foc.speed_ref = Test_Speed_Ref(t_ms);

        // Events, printed here because FOC_Update runs in the ISR (up to one row late)
        if (foc.motor_mode == MOTOR_RUNNING && last_mode != MOTOR_RUNNING)
            printf("*** Startup complete: closed loop at %f rad/s, id = %f A, iq = %f A\r\n", foc.omega,
                   foc.id_ref, foc.iq_ref);
        if (foc.motor_mode != MOTOR_STARTUP_HFI && last_mode == MOTOR_STARTUP_HFI)
            printf("*** HFI start: probe %d mrad (expect ~-500), |Zp| +pulse %d mA, -pulse %d mA -> polarity %d "
                   "(1 kept, -1 flipped, 0 -> Align/I-f)\r\n",
                   (int)(foc.hfi_probe_e * 1000.0f), (int)(foc.hfi_zp_pos * 1000.0f), (int)(foc.hfi_zp_neg * 1000.0f),
                   (int)foc.hfi_pol);
        if (foc.motor_mode != MOTOR_STARTUP_HFI && last_mode == MOTOR_STARTUP_HFI)
            printf("*** Min speed %d rad/s el (0: HFI holds any speed)\r\n", (int)FOC_Min_Speed(&foc));
        // Below FOC_Min_Speed the library stops the motor; start again once the reference is back up
        if (foc.foc_mode == FOC_IDLE && !stopped)
        {
            printf("*** Stopped: speed_ref %d below min speed %d rad/s el\r\n", (int)foc.speed_ref,
                   (int)FOC_Min_Speed(&foc));
            stopped = true;
        }
        else if (stopped && foc.speed_ref >= FOC_Min_Speed(&foc))
        {
            printf("*** Restart at speed_ref %d\r\n", (int)foc.speed_ref);
            stopped = false;
            FOC_Start(&foc, FOC_SPEED_CONTROL);
        }
        last_mode = foc.motor_mode;

        Log_Row(t_ms);
        osDelay(20);
    }

    printf("ISR cycles: max %lu, budget %lu per period (%d Hz PWM)\r\n", (unsigned long)isr_cyc_max,
           (unsigned long)(SystemCoreClock / (uint32_t)PWM_FREQ), (int)PWM_FREQ);
    printf("IRQ cycles (handler entry to callback end): max %lu\r\n", (unsigned long)irq_cyc_max);
    printf("Stopping FOC control... (vbus_fault = %d, vbus_uv_fault = %d)\r\n", (int)foc.vbus_fault,
           (int)foc.vbus_uv_fault);
    // Measured while running on the eSMO (above 2 x OMEGA_FORCE). Recommendation: once it has
    // settled, set FLUX_LINKAGE to it (foc_lib_conf.h), so the first start also uses the real value.
    printf("Flux linkage: measured %d nWb (FLUX_LINKAGE %d nWb)\r\n", (int)(foc.bldc.flux * 1e9f),
           (int)(FLUX_LINKAGE * 1e9f));
    FOC_Stop(&foc);
    FOC_Set_Current_Limit(&foc, I_RATED);

    // Coast check: the rotor is still spinning here; with the bridge off ia/ib must stay ~0,
    // otherwise the bridge is still conducting (braking the motor)
    for (int k = 0; k < 25; k++)
    {
        osDelay(20);
        printf("coast %d: mode %d ia_mA %d ib_mA %d moe %d\r\n", (k + 1) * 20, (int)foc.motor_mode,
               (int)(foc.i_a * 1000.0f), (int)(foc.i_b * 1000.0f), (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0);
    }
}
#endif

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */
    TIMER_config(&htim1, &foc, PWM_FREQ); // TIM1 at PWM_FREQ; sets foc.pwm_resolution
    FOC_Init(&foc, foc.pwm_resolution);    // Library defaults, drive idle, bridge off

    // DWT cycle counter for the ISR timing measurement
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#if ADC_INJ_SMP
    // Longer sampling of the injected channels (before conversions start). SMPR1 field of
    // channel x is at bit 3*x: CURR_U ch1, V_BUS ch2, CURR_W ch6, CURR_V ch7.
    ADC1->SMPR1 = (ADC1->SMPR1 & ~((7u << 3) | (7u << 6) | (7u << 18) | (7u << 21))) |
                  ((uint32_t)ADC_INJ_SMP << 3) | ((uint32_t)ADC_INJ_SMP << 6) |
                  ((uint32_t)ADC_INJ_SMP << 18) | ((uint32_t)ADC_INJ_SMP << 21);
#endif

    // The control interrupt: ADC1 injected sequence (phase currents, bus voltage), triggered by
    // TIM1 CC4 once per PWM period once the PWM runs (Start_PWM in the task)
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Demo sequence: offsets, bus check, calibration, saliency sweep, speed test.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    printf("\r\n=======================================\r\nfoc-lib demo\r\n");
    // Confirms the coast config from TIMER_config: OIS1N/2N/3N (CR2 bits 9/11/13) set, AOE (BDTR bit 14) clear
    printf("TIM1 CR2 = 0x%08lx, BDTR = 0x%08lx\r\n", (unsigned long)TIM1->CR2, (unsigned long)TIM1->BDTR);
    osDelay(1000); // Let the supply and the sense amplifiers settle

    // 1. PWM on (bridge still off: FOC_Init left outputs_enabled false), so the ISR samples the
    //    currents; then the zero-current offsets
    Start_PWM();
    osDelay(50);
    Measure_Current_Offsets();
    printf("PWM resolution %d, Vbus %f V\r\n=======================================\r\n", foc.pwm_resolution,
           foc.v_bus);

    // 2. No (or too low) bus voltage: calibration would measure garbage and nothing can run
    if (foc.v_bus < V_BUS_MIN)
    {
        printf("!! Vbus = %f V < V_BUS_MIN (%f V): motor supply off? Calibration and run skipped.\r\n", foc.v_bus,
               V_BUS_MIN);
        for (;;)
            osDelay(1000);
    }

    // 3. Calibration
    Calibrate();

#if RUN_SALIENCY_SWEEP
    // 4. Saliency map (diagnostic)
    Saliency_Sweep();
#endif

#if RUN_FOC_TEST
    // 5. Speed test
    Speed_Test();
#else
    printf("FOC run skipped (RUN_FOC_TEST = 0)\r\n");
#endif

    for (;;)
    {
        osDelay(5000);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// ============================================================================
// Control interrupt
// ============================================================================
// ADC1 injected end of sequence, once per PWM period (TIM1 CC4). This is the whole contract with
// foc-lib: ADC counts in, FOC_Update, compare values and bridge state out.
// FOC_FAST_CODE: runs from CCM SRAM together with FOC_Update (this port's Inc/foc_lib_conf.h).
FOC_FAST_CODE void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1)
        return;

    const uint32_t cyc0 = DWT->CYCCNT;
    GPIOC->BSRR = GPIO_PIN_8; // PC8 high for the callback's duration (scope timing)

    // Injected ranks 1-4 are JDR1-4 (what HAL_ADCEx_InjectedGetValue returns, without the call)
    foc.adc_v_a = ADC1->JDR1;
    foc.adc_v_b = ADC1->JDR2;
    foc.adc_v_c = ADC1->JDR3;
    foc.adc_v_bus = ADC1->JDR4;

    FOC_Update(&foc);

    TIM1->CCR1 = foc.pwm_a;
    TIM1->CCR2 = foc.pwm_b;
    TIM1->CCR3 = foc.pwm_c;

    // Bridge state (gate pins, see Bridge_Outputs). MOE is left alone: an over-current break
    // clears it and, with AOE off, it stays latched off.
    static bool outputs_were_enabled = false;
    if (foc.outputs_enabled && !outputs_were_enabled)
        Bridge_Outputs(true);
    else if (!foc.outputs_enabled)
        Bridge_Outputs(false);
    outputs_were_enabled = foc.outputs_enabled;

    GPIOC->BSRR = (uint32_t)GPIO_PIN_8 << 16; // PC8 low
    const uint32_t cyc1 = DWT->CYCCNT;
    isr_cyc_last = cyc1 - cyc0;
    if (isr_cyc_last > isr_cyc_max)
        isr_cyc_max = isr_cyc_last;
    irq_cyc_last = cyc1 - irq_cyc0;
    if (irq_cyc_last > irq_cyc_max)
        irq_cyc_max = irq_cyc_last;
}

// Fast dispatch of the control interrupt, called first by ADC1_2_IRQHandler. The only ADC
// interrupt in use is ADC1's end of injected sequence (JEOS, TIM1 CC4 trigger), which this
// serves without HAL_ADC_IRQHandler; stm32f3xx_it.c calls that for ADC1 and for ADC2 (never
// started) and it scans every flag. For JEOS with an external trigger, HAL would only set the
// INJ_EOC state bit (nothing reads it), call the callback and clear JEOC/JEOS.
// Returns false for anything else, which then goes through the HAL handlers as before.
FOC_FAST_CODE bool ADC_IRQ_Fast_Dispatch(void)
{
    if ((ADC1->ISR & ADC_ISR_JEOS) == 0 || (ADC1->IER & ADC_IER_JEOSIE) == 0)
    {
        return false;
    }
    // rc_w1. Cleared before the callback so the write has landed long before the exception
    // returns (a late clear can re-pend the interrupt).
    ADC1->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    return true;
}

/* USER CODE END Application */
