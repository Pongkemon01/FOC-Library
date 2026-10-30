/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include <stdio.h>
#include "usart.h"
#include "tim.h"
#include "adc.h"
#include "foc_lib.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef ADC_INJ_SMP
#define ADC_INJ_SMP 0 // Injected-channel ADC sampling time code: 0 = keep CubeMX (1.5 cycles), 4 = 19.5, 5 = 61.5
#endif
#define RUN_FOC_TEST 1 // 0: stop after calibration (current-sign diagnostic), 1: run the 5 s speed test
#define RUN_DIRECTION NORMAL_DIRECTION // Speed test direction: NORMAL_DIRECTION or REVERSE_DIRECTION
#define RUN_SALIENCY_SWEEP 1 // 1: after calibration, Ld-type HF injection at SWEEP_POINTS angles over 0..2pi
#define SWEEP_POINTS 24
// HFI (standstill start, low speed) is always on (foc-lib FOC_HFI). Test only:
#define HFI_SHADOW_TEST 0 // 1: shadow run for measuring HFI_OMEGA_LOW (with TEST_PROFILE 3, then
                          // tools/hfi_threshold.py): Align -> I-f start, the tracker runs below
                          // HFI_OMEGA_ON and is logged beside the eSMO but never takes control (the
                          // eSMO coasts the drive when it loses lock)
#define TEST_PROFILE 4 // Speed test profile (rad/s el):
                       // 0: 1500, 2500 from 3 s (5 s)
                       // 1: gentle staircase for a propeller load (8 s): 1500, ramp 500 rad/s^2 to 2000,
                       //    hold, ramp 250 rad/s^2 to 2500, hold. The bench 12 V 5 A supply collapses at
                       //    ~20 W (2026-10-30), so no 3000 step and slow ramps (less inertia current).
                       // 2: low speed (12 s): 1500, 150 from 2.5 s (the drive cannot brake: it coasts
                       //    down through the eSMO -> HFI switch), 0 from 8.5 s, 1500 from 10 s
                       // 3: slow sweep down (11 s), for HFI_SHADOW_TEST 1: 1500 to 2.5 s, then 800 falling at
                       //    100 rad/s^2 to 100 at 9.5 s, hold. smo_hfi vs speed shows the eSMO's limit
                       // 4: HFI from standstill (11 s): 150 to 5 s (with a prop, the start
                       //    accelerates its inertia at the current cap; the hold may be braked by hand),
                       //    0 from 5 s, 150 from 6.5 s, 1500 from 8 s (HFI -> eSMO)
#define TEST_I_MAX 5.0f // Speed-test current cap (A): iq_ref and the speed PI output (default I_RATED)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
FOC_t foc;

// ADC-callback execution time in CPU cycles (DWT cycle counter, 72 MHz), excluding the HAL IRQ
// handler and exception entry. Budget: 72e6 / PWM_FREQ cycles per period.
static volatile uint32_t isr_cyc_last, isr_cyc_max;
// Same, but from ADC1_2_IRQHandler entry (stamped there into irq_cyc0) to the end of the
// callback: also covers the interrupt dispatch. Excludes only the exception entry itself.
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

// Get timer clock frequency
static uint32_t get_timer_clock(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 || htim->Instance == TIM8)
    {
        // Timer di APB2
        uint32_t pclk = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
            return pclk * 2; // Timer clock doubled if APB prescaler != 1
        else
            return pclk;
    }
    else
    {
        // Timer di APB1
        uint32_t pclk = HAL_RCC_GetPCLK1Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
            return pclk * 2;
        else
            return pclk;
    }
}

// -----------------------------------------------------------
// Calculate timer period for desired PWM frequency
int TIMER_config(TIM_HandleTypeDef *timer, FOC_t *foc, float pwm_freq)
{
    if (foc == NULL || timer == NULL)
        return 0;

    const uint32_t timer_clock = get_timer_clock(timer);

    // prescaler dan period untuk mode center-aligned
    uint32_t prescaler = 0;
    uint32_t period = (timer_clock / (2 * pwm_freq)) - 1;

    if (period > 0xFFFF)
    {
        prescaler = period / 0xFFFF;
        period = (timer_clock / (2 * pwm_freq * (prescaler + 1))) - 1;
    }

    // Set prescaler and period
    timer->Instance->PSC = prescaler;
    timer->Instance->ARR = period;

    // Set CCR for channel 4 used for ADC trigger
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

// -----------------------------------------------------------
// Start and Stop PWM functions
void Start_PWM()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    htim1.Instance->CCR4 = htim1.Instance->ARR - 1; // Set CCR4 for ADC trigger

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void Stop_PWM()
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
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

void Set_Debug()
{
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

void Clear_Debug()
{
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

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
    TIMER_config(&htim1, &foc, PWM_FREQ);           // Re-configure timer for desired PWM frequency
    FOC_Init(&foc, foc.pwm_resolution, ADC_OFFSET); // Initialize FOC structure

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

    // Enable interrupts for ADC injected conversion complete
    HAL_ADCEx_InjectedStart_IT(&hadc1); // Current sensors and Vbus

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
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    printf("\r\n=======================================\r\n");
    printf("Starting main task...\r\n");
    // Confirms the coast config from TIMER_config: OIS1N/2N/3N (CR2 bits 9/11/13) set, AOE (BDTR bit 14) clear
    printf("TIM1 CR2 = 0x%08lx, BDTR = 0x%08lx\r\n", (unsigned long)TIM1->CR2, (unsigned long)TIM1->BDTR);
    osDelay(1000); // Delay a bit

    // HAL_TIM_Base_Start_IT(&htim1);      // Timer 1 update interrupt

    Start_PWM(); // Start PWM signals

    // Delay for 50ms
    osDelay(50);

    // Read current values and set as offsets (because at this point, the motor is not running and the current should be zero)
    foc.adc_offset_a = foc.adc_v_a;
    foc.adc_offset_b = foc.adc_v_b;
    foc.adc_offset_c = foc.adc_v_c;
    printf("ADC offset 1 = %d\r\n", foc.adc_offset_a);
    printf("ADC offset 2 = %d\r\n", foc.adc_offset_b);
    printf("ADC offset 3 = %d\r\n", foc.adc_offset_c);

    printf("PWM calculations:\r\n");
    printf("  - Parameters : PWM-Resolution = %d , Vbus = %f\r\n", foc.pwm_resolution, foc.v_bus);
    printf("=======================================\r\n");

    // No (or too low) bus voltage: calibration would measure garbage and the run cannot work
    if (foc.v_bus < V_BUS_MIN)
    {
        printf("!! Vbus = %f V < V_BUS_MIN (%f V): motor supply off? Calibration and run skipped.\r\n",
               foc.v_bus, V_BUS_MIN);
        for (;;)
            osDelay(1000);
    }
    foc.foc_mode = FOC_CALIBRATION;
    foc.motor_mode = MOTOR_ESTIMATION_RS;
    foc.inj_fault = false;
    foc.inj_start_flag = true;
    printf("Measuring Rs....\r\n");
    while (foc.inj_start_flag)
        osDelay(1);
    printf("Finish measuring.\r\n");
    if (foc.inj_fault)
        printf("  !! ABORTED on over-current (I > %f A) - Rs not updated\r\n", (double)INJECTION_MAX_CURRENT);
    else
        FOC_BLDC_Parameter_Calculation_Resistance(&foc);

    printf("Measured Rs = %f\r\n", foc.bldc.Rs);
    printf("  sample: i_a = %f, i_b = %f, i_d = %f\r\n", foc.i_a, foc.i_b, foc.i_d);

    // Current-sign diagnostic: Rs injection is DC on phase A (v_d > 0 at theta = 0),
    // so a correct sense chain gives i_a > 0 and i_b ~= -i_a / 2. Rs uses fabsf and hides this.
    {
        float m_vd = 0.0f, m_id = 0.0f, m_ia = 0.0f, m_ib = 0.0f;
        for (int i = 0; i < INJECTION_ROUNDS; i++)
        {
            m_vd += foc.vd_inj_buff[i];
            m_id += foc.id_inj_buff[i];
            m_ia += foc.i_a_buff[i];
            m_ib += foc.i_b_buff[i];
        }
        printf("  signed means: v_d = %f, i_d = %f, i_a = %f, i_b = %f (expect i_a > 0, i_b ~ -i_a/2)\r\n",
               m_vd / INJECTION_ROUNDS, m_id / INJECTION_ROUNDS, m_ia / INJECTION_ROUNDS, m_ib / INJECTION_ROUNDS);

        // Settling check of the recorded high level: a drift means the rotor was still moving
        float first = 0.0f, last = 0.0f;
        for (int i = 0; i < 32; i++)
        {
            first += foc.id_inj_buff[i];
            last += foc.id_inj_buff[INJECTION_ROUNDS - 32 + i];
        }
        printf("  i_d first/last 32 samples of the high level: %f / %f A\r\n", first / 32.0f, last / 32.0f);
    }

    foc.foc_mode = FOC_CALIBRATION;
    foc.motor_mode = MOTOR_ESTIMATION_LD;
    foc.inj_fault = false;
    foc.inj_start_flag = true;
    printf("Measuring Ld....\r\n");
    while (foc.inj_start_flag)
        osDelay(1);
    printf("Finish measuring.\r\n");
    if (foc.inj_fault)
        printf("  !! ABORTED on over-current - Ld invalid\r\n");

    foc.foc_mode = FOC_CALIBRATION;
    foc.motor_mode = MOTOR_ESTIMATION_LQ;
    foc.inj_fault = false;
    foc.inj_start_flag = true;
    printf("Measuring Lq....\r\n");
    while (foc.inj_start_flag)
        osDelay(1);
    printf("Finish measuring.\r\n");
    if (foc.inj_fault)
        printf("  !! ABORTED on over-current - Lq invalid\r\n");

    FOC_BLDC_Parameter_Calculation_Inductance(&foc);
    printf("Measured Ls = %f, Ld = %f, Lq = %f\r\n", foc.bldc.Ls, foc.bldc.Ld, foc.bldc.Lq);

    FOC_Recalc_Params(&foc);

#if RUN_SALIENCY_SWEEP
    // Saliency map (stationary rotor, left where the Rs run aligned it: d-axis near 0 within a
    // cogging step). The same 1 kHz INJECTION_VOLTAGE sine as the Ld run, applied along each angle;
    // no DC, so no mean torque. Fit L(phi) = A0 + A1 cos(phi - p1) + A2 cos(2 phi - 2 p2):
    // A2 is the saliency (expect min L at phi ~ 0 / pi), A1 should be ~0 (rotor did not move).
    {
        static float sw_L[SWEEP_POINTS], sw_xr[SWEEP_POINTS], sw_xi[SWEEP_POINTS];
        bool sw_ok = true;
        printf("Saliency sweep (%d points, 1 kHz, %f V):\r\n  deg  L_axis(nH)  cross_re(x1e4)  cross_im(x1e4)\r\n",
               SWEEP_POINTS, (double)INJECTION_VOLTAGE);
        for (int k = 0; k < SWEEP_POINTS && sw_ok; k++)
        {
            foc.inj_angle = (float)k * (TWO_PI / SWEEP_POINTS);
            foc.foc_mode = FOC_CALIBRATION;
            foc.motor_mode = MOTOR_ESTIMATION_LD;
            foc.inj_fault = false;
            foc.inj_start_flag = true;
            while (foc.inj_start_flag)
                osDelay(1);
            sw_ok = !foc.inj_fault && FOC_BLDC_Saliency_Point(&foc, &sw_L[k], &sw_xr[k], &sw_xi[k]);
            if (sw_ok)
                printf("  %3d  %6d  %6d  %6d\r\n", k * 360 / SWEEP_POINTS, (int)(sw_L[k] * 1e9f),
                       (int)(sw_xr[k] * 1e4f), (int)(sw_xi[k] * 1e4f));
        }
        foc.inj_angle = 0.0f;

        if (!sw_ok)
            printf("  !! sweep aborted (over-current or no current)\r\n");
        else
        {
            float s0 = 0, c1 = 0, s1 = 0, c2 = 0, s2 = 0, xc2 = 0, xs2 = 0;
            for (int k = 0; k < SWEEP_POINTS; k++)
            {
                float p = (float)k * (TWO_PI / SWEEP_POINTS);
                s0 += sw_L[k];
                c1 += sw_L[k] * cosf(p);
                s1 += sw_L[k] * sinf(p);
                c2 += sw_L[k] * cosf(2.0f * p);
                s2 += sw_L[k] * sinf(2.0f * p);
                xc2 += sw_xr[k] * cosf(2.0f * p);
                xs2 += sw_xr[k] * sinf(2.0f * p);
            }
            const float n = (float)SWEEP_POINTS;
            float A0 = s0 / n, A1 = 2.0f * sqrtf(c1 * c1 + s1 * s1) / n, A2 = 2.0f * sqrtf(c2 * c2 + s2 * s2) / n;
            float p2 = 0.5f * atan2f(s2, c2); // angle of max L (q-axis)
            float X2 = 2.0f * sqrtf(xc2 * xc2 + xs2 * xs2) / n;

            // Residual after the 0th..2nd harmonics: measurement noise floor
            float res = 0.0f;
            for (int k = 0; k < SWEEP_POINTS; k++)
            {
                float p = (float)k * (TWO_PI / SWEEP_POINTS);
                float fit = A0 + 2.0f / n * (c1 * cosf(p) + s1 * sinf(p) + c2 * cosf(2.0f * p) + s2 * sinf(2.0f * p));
                res += (sw_L[k] - fit) * (sw_L[k] - fit);
            }
            res = sqrtf(res / n);

            printf("  fit: L0 %d nH, A2 %d nH (min-L axis %d deg el, mod 180), A1 %d nH, residual rms %d nH\r\n",
                   (int)(A0 * 1e9f), (int)(A2 * 1e9f), (int)((p2 + 0.5f * PI) * (180.0f / PI)) % 180,
                   (int)(A1 * 1e9f), (int)(res * 1e9f));
            printf("  -> Ld ~ %d nH, Lq ~ %d nH; cross-axis 2nd harmonic %d x1e-4 of the axis current\r\n",
                   (int)((A0 - A2) * 1e9f), (int)((A0 + A2) * 1e9f), (int)(X2 * 1e4f));
        }
    }
#endif

#if RUN_FOC_TEST
    printf("Starting FOC control...\r\n");
    // Speed test: 1500 rad/s el (~2050 rpm), stepped to 2500 (~3400 rpm) at t = 3 s; both below
    // ESMO_OMEGA_MAX. The speed loop takes over at the startup handover (FOC_Speed_Control).
    // foc.is_ref = 3.0f;   // Torque mode instead: stator current magnitude (A), runs away unloaded
    // Speeds and is_ref are relative to the direction (positive = in RUN_DIRECTION).
    // FOC_CALIBRATION keeps the bridge enabled (zero vector) after the last injection, and the
    // direction can only be set with the bridge off: go idle first (the ISR turns the outputs
    // off within one PWM period), then set the direction, then start.
    foc.foc_mode = FOC_IDLE;
    osDelay(2);
    if (!FOC_Set_Direction(&foc, RUN_DIRECTION))
        printf("!! FOC_Set_Direction refused (bridge energized)\r\n");
    foc.speed_ref = 1500.0f;
    foc.bldc.max_current = TEST_I_MAX;
    PI_Set_Output_Limit(&foc.speed_ctrl, TEST_I_MAX);
    foc.hfi_takeover = !HFI_SHADOW_TEST; // Test only (see HFI_SHADOW_TEST)
    foc.motor_mode = MOTOR_IDLE;
    foc.foc_mode = FOC_SPEED_CONTROL;

    // Log the drive state at ~50 Hz for the TEST_PROFILE duration (task context, never from the ISR). Integers only
    // (rad/s, mV, mA) to keep printf's stack use low on this task's 512 B stack.
    // brk = TIM1 break flag: the IHM08M1 over-current comparator tripped (outputs latched off).
    // ia/ib are updated every ISR (id/iq freeze once the outputs are off): after a coast they
    // must stay ~0, otherwise the bridge is still braking. moe = TIM1 main output enable.
    printf("Before run: brk = %d, moe = %d\r\n", (TIM1->SR & TIM_SR_BIF) ? 1 : 0, (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0);
    TIM1->SR = ~TIM_SR_BIF; // Clear a stale break flag (rc_w0)
    const uint32_t t0 = osKernelGetTickCount();
    isr_cyc_max = 0;
    irq_cyc_max = 0;
    // HFI columns: on = tracker running, ctl = HFI in control, smo_hfi = eSMO minus HFI angle (0.1 deg el;
    // where it grows while the speed falls is the eSMO's real low-speed limit), w_hfi = HFI speed,
    // zp = HF response |Zp| (mA), err = tracker error (0.1 deg), flt = HFI fault (coasted)
    printf("t_ms mode w_force w_ref w_est emf_mV lock cnt id_mA iq_mA ia_mA ib_mA vd_mV vq_mV vbus_mV brk moe isr_cyc irq_cyc on ctl smo_hfi w_hfi zp err flt\r\n");
    BLDC_Mode_t last_mode = foc.motor_mode;
#if TEST_PROFILE == 4
    for (int k = 0; k < 550; k++) // 11 s
    {
        uint32_t t_ms = osKernelGetTickCount() - t0;
        foc.speed_ref = (t_ms >= 8000) ? 1500.0f : (t_ms >= 6500) ? 150.0f : (t_ms >= 5000) ? 0.0f : 150.0f;
#elif TEST_PROFILE == 3
    for (int k = 0; k < 550; k++) // 11 s
    {
        uint32_t t_ms = osKernelGetTickCount() - t0;
        if (t_ms < 2500)
            foc.speed_ref = 1500.0f;
        else if (t_ms < 9500)
            foc.speed_ref = 800.0f - 0.1f * (float)(t_ms - 2500);
        else
            foc.speed_ref = 100.0f;
#elif TEST_PROFILE == 2
    for (int k = 0; k < 600; k++) // 12 s
    {
        uint32_t t_ms = osKernelGetTickCount() - t0;
        foc.speed_ref = (t_ms >= 10000) ? 1500.0f : (t_ms >= 8500) ? 0.0f : (t_ms >= 2500) ? 150.0f : 1500.0f;
#elif TEST_PROFILE == 1
    for (int k = 0; k < 400; k++) // 8 s
    {
        uint32_t t_ms = osKernelGetTickCount() - t0;
        // 1500 to 2.5 s; 500 rad/s^2 to 2000 at 3.5 s; hold to 4.5 s; 250 rad/s^2 to 2500 at 6.5 s; hold.
        // Slower than the library's SPEED_REF_SLEW, so the library ramp just follows.
        if (t_ms < 2500)
            foc.speed_ref = 1500.0f;
        else if (t_ms < 3500)
            foc.speed_ref = 1500.0f + 0.5f * (float)(t_ms - 2500);
        else if (t_ms < 4500)
            foc.speed_ref = 2000.0f;
        else if (t_ms < 6500)
            foc.speed_ref = 2000.0f + 0.25f * (float)(t_ms - 4500);
        else
            foc.speed_ref = 2500.0f;
#else
    for (int k = 0; k < 250; k++)
    {
        uint32_t t_ms = osKernelGetTickCount() - t0;
        if (t_ms >= 3000)
            foc.speed_ref = 2500.0f;
#endif
        // Handover message, printed here because FOC_Update runs in the ADC ISR (values are
        // up to one logger period after the handover)
        if (foc.motor_mode == MOTOR_RUNNING && last_mode != MOTOR_RUNNING)
            printf("*** Startup complete: handover to closed loop at speed %f rad/s, id = %f A, iq = %f A\r\n",
                   foc.omega, foc.id_ref, foc.iq_ref);
        if (foc.motor_mode != MOTOR_STARTUP_HFI && last_mode == MOTOR_STARTUP_HFI)
            printf("*** HFI start: probe %d mrad (expect ~-500), |Zp| +pulse %d mA, -pulse %d mA -> polarity %d "
                   "(1 kept, -1 flipped, 0 -> Align/I-f)\r\n",
                   (int)(foc.hfi_probe_e * 1000.0f), (int)(foc.hfi_zp_pos * 1000.0f),
                   (int)(foc.hfi_zp_neg * 1000.0f), (int)foc.hfi_pol);
        last_mode = foc.motor_mode;
        printf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
               (int)t_ms, (int)foc.motor_mode, (int)foc.startup_omega,
               (int)foc.speed_ref_ramp, (int)foc.smo.omega_est, (int)(foc.smo.emf_magnitude * 1000.0f), (int)foc.smo.locked,
               (int)foc.bemf_counter, (int)(foc.i_d * 1000.0f), (int)(foc.i_q * 1000.0f),
               (int)(foc.i_a * 1000.0f), (int)(foc.i_b * 1000.0f),
               (int)(foc.v_d * 1000.0f), (int)(foc.v_q * 1000.0f), (int)(foc.v_bus * 1000.0f),
               (TIM1->SR & TIM_SR_BIF) ? 1 : 0, (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0, (int)isr_cyc_last,
               (int)irq_cyc_last);
        // Separate call: fewer varargs per printf on this task's 512 B stack
        float smo_hfi = foc.smo.theta_est - foc.hfi_theta;
        if (smo_hfi > PI)
            smo_hfi -= TWO_PI;
        else if (smo_hfi < -PI)
            smo_hfi += TWO_PI;
        printf(" %d %d %d %d %d %d %d\r\n", (int)foc.hfi_on, (int)foc.hfi_control,
               (int)(smo_hfi * (1800.0f / PI)), (int)foc.hfi_omega, (int)(foc.hfi_zp * 1000.0f),
               (int)(foc.hfi_err * (1800.0f / PI)), (int)foc.hfi_fault);
        osDelay(20);
    }
    printf("ISR cycles: max %lu, budget %lu per period (%d Hz PWM)\r\n", (unsigned long)isr_cyc_max,
           (unsigned long)(SystemCoreClock / (uint32_t)PWM_FREQ), (int)PWM_FREQ);
    printf("IRQ cycles (handler entry to callback end): max %lu\r\n", (unsigned long)irq_cyc_max);
    printf("Stopping FOC control... (vbus_fault = %d, vbus_uv_fault = %d)\r\n", (int)foc.vbus_fault,
           (int)foc.vbus_uv_fault);
    foc.foc_mode = FOC_IDLE;
    foc.bldc.max_current = I_RATED;
    PI_Set_Output_Limit(&foc.speed_ctrl, I_RATED);
    // Coast check: the rotor is still spinning down here; ia/ib must stay ~0 with moe = 0
    for (int k = 0; k < 25; k++)
    {
        osDelay(20);
        printf("coast %d: mode %d ia_mA %d ib_mA %d moe %d\r\n", (k + 1) * 20, (int)foc.motor_mode,
               (int)(foc.i_a * 1000.0f), (int)(foc.i_b * 1000.0f), (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0);
    }
#else
    printf("FOC run skipped (RUN_FOC_TEST = 0)\r\n");
#endif

    /* Infinite loop */
    for (;;)
    {
        osDelay(5000);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// The speed loop runs inside FOC_Update (FOC_Speed_Control in foc_lib.c), not in a task.

// -----------------------------------------------------------
// Callback function for ADC injected conversion complete
// (FOC_FAST_CODE: runs from CCM SRAM with FOC_Update with this port's Inc/foc_lib_conf.h)
FOC_FAST_CODE void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) // Current sensor phase U
    {
        const uint32_t cyc0 = DWT->CYCCNT;
        GPIOC->BSRR = GPIO_PIN_8; // PC8 high for the callback's duration (scope timing)
        //  Measure ADC values for phase currents and bus voltage: injected ranks 1-4 are
        //  JDR1-4 (what HAL_ADCEx_InjectedGetValue returns, without the call and switch)
        foc.adc_v_a = ADC1->JDR1;
        foc.adc_v_b = ADC1->JDR2;
        foc.adc_v_c = ADC1->JDR3;
        foc.adc_v_bus = ADC1->JDR4;

        // Call FOC control algorithm
        FOC_Update(&foc);

        // Update PWM registers
        TIM1->CCR1 = foc.pwm_a;
        TIM1->CCR2 = foc.pwm_b;
        TIM1->CCR3 = foc.pwm_c;

        // HAL side of foc.outputs_enabled (gate pins, see Bridge_Outputs). MOE is left alone: an
        // over-current break clears it and, with AOE off, it stays latched off.
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
