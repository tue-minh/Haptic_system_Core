/**
  ******************************************************************************
  * @file           : tue_dc.c
  * @brief          : 4-Channel DC motor driver implementation for DRV8872 using
  *                   TIM3 and TIM4 PWM on STM32F407 (MM_Control).
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_dc.h"
#include "tim.h"
#include "gpio.h"

#include <string.h>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
#define TUE_DC_TIMER_CLOCK_HZ     84000000U   /* APB1 timer clock = 84 MHz     */

/* Channel to hardware mapping table */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t          tim_channel_in1;
    uint32_t          tim_channel_in2;
    GPIO_TypeDef      *fault_port;
    uint16_t          fault_pin;
} dc_hw_channel_t;

/* Private variables ---------------------------------------------------------*/
static bool                     s_initialized            = false;
static uint32_t                 s_pwm_freq_hz            = TUE_DC_DEFAULT_PWM_FREQ_HZ;
static uint32_t                 s_timer_arr              = 4199U; /* 84MHz / 20kHz - 1 */
static uint32_t                 s_timer_psc              = 0U;
static bool                     s_auto_fault_protection  = true;
static int16_t                  s_target_speed[TUE_DC_MOTOR_COUNT] = {0};
static uint16_t                 s_ramp_rate_per_s        = 0U; /* 0 = instantaneous */

static tue_dc_channel_status_t  s_channels[TUE_DC_MOTOR_COUNT];

/* Hardware pin and timer channel descriptors */
static const dc_hw_channel_t s_hw[TUE_DC_MOTOR_COUNT] = {
    /* Motor 1: TIM4 CH1 (PD12), TIM4 CH2 (PD13), nFAULT: PD8 (DC_F1) */
    {
        .htim            = &htim4,
        .tim_channel_in1 = TIM_CHANNEL_1,
        .tim_channel_in2 = TIM_CHANNEL_2,
        .fault_port      = GPIOD,
        .fault_pin       = DC_F1_Pin
    },
    /* Motor 2: TIM4 CH3 (PD14), TIM4 CH4 (PD15), nFAULT: PD9 (DC_F2) */
    {
        .htim            = &htim4,
        .tim_channel_in1 = TIM_CHANNEL_3,
        .tim_channel_in2 = TIM_CHANNEL_4,
        .fault_port      = GPIOD,
        .fault_pin       = DC_F2_Pin
    },
    /* Motor 3: TIM3 CH1 (PC6), TIM3 CH2 (PC7), nFAULT: PD10 (DC_F3) */
    {
        .htim            = &htim3,
        .tim_channel_in1 = TIM_CHANNEL_1,
        .tim_channel_in2 = TIM_CHANNEL_2,
        .fault_port      = GPIOD,
        .fault_pin       = DC_F3_Pin
    },
    /* Motor 4: TIM3 CH3 (PC8), TIM3 CH4 (PC9), nFAULT: PD11 (DC_F4) */
    {
        .htim            = &htim3,
        .tim_channel_in1 = TIM_CHANNEL_3,
        .tim_channel_in2 = TIM_CHANNEL_4,
        .fault_port      = GPIOD,
        .fault_pin       = DC_F4_Pin
    }
};

/* Private function prototypes -----------------------------------------------*/
static void dc_apply_hardware(uint8_t idx, uint32_t ccr_in1, uint32_t ccr_in2);
static void dc_apply_speed(uint8_t idx, int16_t speed);
static bool dc_read_fault_hardware(uint8_t idx);
static void dc_update_timer_base(uint32_t psc, uint32_t arr);
static void dc_check_channel_fault(uint8_t idx);
static void dc_service_channel_tick(uint8_t idx);

/* ========================================================================== */
/* Hardware Access Helpers                                                    */
/* ========================================================================== */

static void dc_apply_hardware(uint8_t idx, uint32_t ccr_in1, uint32_t ccr_in2)
{
    if (idx >= TUE_DC_MOTOR_COUNT) {
        return;
    }
    const dc_hw_channel_t *hw = &s_hw[idx];
    __HAL_TIM_SET_COMPARE(hw->htim, hw->tim_channel_in1, ccr_in1);
    __HAL_TIM_SET_COMPARE(hw->htim, hw->tim_channel_in2, ccr_in2);
}

static void dc_apply_speed(uint8_t idx, int16_t speed)
{
    if (idx >= TUE_DC_MOTOR_COUNT) {
        return;
    }
    uint16_t abs_duty = (uint16_t)abs(speed);
    uint32_t pulse = ((uint32_t)abs_duty * (s_timer_arr + 1U)) / 1000U;
    if (pulse > (s_timer_arr + 1U)) {
        pulse = s_timer_arr + 1U;
    }

    s_channels[idx].speed = speed;
    s_channels[idx].duty  = abs_duty;

    if (speed > 0) {
        /* Forward: IN1 = PWM, IN2 = 0 */
        s_channels[idx].mode = TUE_DC_MODE_FORWARD;
        dc_apply_hardware(idx, pulse, 0U);
    } else if (speed < 0) {
        /* Reverse: IN1 = 0, IN2 = PWM */
        s_channels[idx].mode = TUE_DC_MODE_REVERSE;
        dc_apply_hardware(idx, 0U, pulse);
    } else {
        /* Zero speed: Coast (IN1=0, IN2=0) */
        s_channels[idx].mode = TUE_DC_MODE_COAST;
        dc_apply_hardware(idx, 0U, 0U);
    }
}

static bool dc_read_fault_hardware(uint8_t idx)
{
    if (idx >= TUE_DC_MOTOR_COUNT) {
        return false;
    }
    /* DRV8872 nFAULT is active low: RESET (0) means fault active! */
    return (HAL_GPIO_ReadPin(s_hw[idx].fault_port, s_hw[idx].fault_pin) == GPIO_PIN_RESET);
}

static void dc_check_channel_fault(uint8_t idx)
{
    if (idx >= TUE_DC_MOTOR_COUNT) {
        return;
    }
    bool fault = dc_read_fault_hardware(idx);
    s_channels[idx].fault_active = fault;
    if (fault) {
        if (!s_channels[idx].fault_latched) {
            s_channels[idx].fault_count++;
            s_channels[idx].fault_latched = true;
        }
        if (s_auto_fault_protection && s_channels[idx].mode != TUE_DC_MODE_COAST) {
            /* Emergency shutdown on hardware fault */
            dc_apply_hardware(idx, 0U, 0U);
            s_channels[idx].mode        = TUE_DC_MODE_COAST;
            s_channels[idx].speed       = 0;
            s_channels[idx].duty        = 0U;
            s_target_speed[idx]         = 0;
        }
    }
}

static void dc_update_timer_base(uint32_t psc, uint32_t arr)
{
    s_timer_psc = psc;
    s_timer_arr = arr;

    /* Update TIM3 */
    __HAL_TIM_SET_PRESCALER(&htim3, (uint32_t)psc);
    __HAL_TIM_SET_AUTORELOAD(&htim3, (uint32_t)arr);
    htim3.Instance->EGR = TIM_EGR_UG;

    /* Update TIM4 */
    __HAL_TIM_SET_PRESCALER(&htim4, (uint32_t)psc);
    __HAL_TIM_SET_AUTORELOAD(&htim4, (uint32_t)arr);
    htim4.Instance->EGR = TIM_EGR_UG;
}

/* ========================================================================== */
/* Public API Implementation                                                  */
/* ========================================================================== */

tue_dc_status_t tue_dc_init(void)
{
    if (s_initialized) {
        return TUE_DC_OK;
    }

    /* 1. Ensure fault pins on GPIOD (PD8..PD11) are configured for EXTI falling edge with pull-up */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = DC_F1_Pin | DC_F2_Pin | DC_F3_Pin | DC_F4_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Enable EXTI NVIC interrupts for active-low DRV8872 fault lines */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* 2. Configure 20 kHz default PWM frequency (84 MHz / 4200 = 20 kHz) */
    s_pwm_freq_hz = TUE_DC_DEFAULT_PWM_FREQ_HZ;
    s_timer_psc   = 0U;
    s_timer_arr   = (TUE_DC_TIMER_CLOCK_HZ / s_pwm_freq_hz) - 1U; /* 4199 */
    dc_update_timer_base(s_timer_psc, s_timer_arr);

    /* 3. Initialize all compare outputs to 0 (all outputs LOW = COAST) */
    for (uint8_t i = 0U; i < TUE_DC_MOTOR_COUNT; i++) {
        dc_apply_hardware(i, 0U, 0U);
        s_channels[i].motor_id      = (uint8_t)(i + 1U);
        s_channels[i].mode          = TUE_DC_MODE_COAST;
        s_channels[i].speed         = 0;
        s_channels[i].duty          = 0U;
        s_channels[i].fault_active  = false;
        s_channels[i].fault_latched = false;
        s_channels[i].fault_count   = 0U;
        s_target_speed[i]           = 0;
    }

    /* 4. Start PWM output on all 8 channels */
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* 5. Enable Timer Update Interrupts on TIM3 and TIM4 for 1 kHz control & ramping */
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

    /* Initial fault read */
    for (uint8_t i = 0U; i < TUE_DC_MOTOR_COUNT; i++) {
        dc_check_channel_fault(i);
    }

    s_initialized = true;
    return TUE_DC_OK;
}

bool tue_dc_is_initialized(void)
{
    return s_initialized;
}

tue_dc_status_t tue_dc_set_speed(uint8_t motor_id, int16_t speed)
{
    if (!s_initialized) {
        tue_dc_init();
    }

    if ((motor_id < 1U) || (motor_id > TUE_DC_MOTOR_COUNT)) {
        return TUE_DC_ERR_PARAM;
    }

    uint8_t idx = (uint8_t)(motor_id - 1U);

    /* Clamp speed to [-1000, 1000] */
    if (speed > TUE_DC_SPEED_MAX)  speed = TUE_DC_SPEED_MAX;
    if (speed < TUE_DC_SPEED_MIN)  speed = TUE_DC_SPEED_MIN;

    /* Check fault before applying drive */
    dc_check_channel_fault(idx);
    if (s_auto_fault_protection && s_channels[idx].fault_active && (speed != 0)) {
        return TUE_DC_ERR_FAULT;
    }

    s_target_speed[idx] = speed;

    if (s_ramp_rate_per_s == 0U) {
        /* Instantaneous change */
        dc_apply_speed(idx, speed);
    } else {
        /* Smooth ramping via 1 kHz timer interrupt */
        if (s_channels[idx].speed == speed) {
            dc_apply_speed(idx, speed);
        }
    }

    return TUE_DC_OK;
}

tue_dc_status_t tue_dc_set_speed_all(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    tue_dc_status_t s1 = tue_dc_set_speed(1U, m1);
    tue_dc_status_t s2 = tue_dc_set_speed(2U, m2);
    tue_dc_status_t s3 = tue_dc_set_speed(3U, m3);
    tue_dc_status_t s4 = tue_dc_set_speed(4U, m4);

    if (s1 != TUE_DC_OK) return s1;
    if (s2 != TUE_DC_OK) return s2;
    if (s3 != TUE_DC_OK) return s3;
    if (s4 != TUE_DC_OK) return s4;
    return TUE_DC_OK;
}

tue_dc_status_t tue_dc_brake(uint8_t motor_id)
{
    if (!s_initialized) {
        tue_dc_init();
    }

    uint8_t start_idx = 0U;
    uint8_t end_idx   = TUE_DC_MOTOR_COUNT;

    if ((motor_id >= 1U) && (motor_id <= TUE_DC_MOTOR_COUNT)) {
        start_idx = (uint8_t)(motor_id - 1U);
        end_idx   = (uint8_t)(start_idx + 1U);
    } else if ((motor_id != TUE_DC_MOTOR_ALL) && (motor_id != 0U)) {
        return TUE_DC_ERR_PARAM;
    }

    /* Brake: IN1 = HIGH (100%), IN2 = HIGH (100%) */
    uint32_t full_on = s_timer_arr + 1U;
    for (uint8_t i = start_idx; i < end_idx; i++) {
        s_target_speed[i]   = 0;
        dc_apply_hardware(i, full_on, full_on);
        s_channels[i].mode  = TUE_DC_MODE_BRAKE;
        s_channels[i].speed = 0;
        s_channels[i].duty  = 1000U;
    }

    return TUE_DC_OK;
}

tue_dc_status_t tue_dc_coast(uint8_t motor_id)
{
    if (!s_initialized) {
        tue_dc_init();
    }

    uint8_t start_idx = 0U;
    uint8_t end_idx   = TUE_DC_MOTOR_COUNT;

    if ((motor_id >= 1U) && (motor_id <= TUE_DC_MOTOR_COUNT)) {
        start_idx = (uint8_t)(motor_id - 1U);
        end_idx   = (uint8_t)(start_idx + 1U);
    } else if ((motor_id != TUE_DC_MOTOR_ALL) && (motor_id != 0U)) {
        return TUE_DC_ERR_PARAM;
    }

    /* Coast: IN1 = LOW, IN2 = LOW */
    for (uint8_t i = start_idx; i < end_idx; i++) {
        s_target_speed[i]   = 0;
        dc_apply_hardware(i, 0U, 0U);
        s_channels[i].mode  = TUE_DC_MODE_COAST;
        s_channels[i].speed = 0;
        s_channels[i].duty  = 0U;
    }

    return TUE_DC_OK;
}

tue_dc_status_t tue_dc_stop(uint8_t motor_id)
{
    return tue_dc_coast(motor_id);
}

tue_dc_status_t tue_dc_get_channel_status(uint8_t motor_id, tue_dc_channel_status_t *out_status)
{
    if (!s_initialized) {
        tue_dc_init();
    }

    if ((motor_id < 1U) || (motor_id > TUE_DC_MOTOR_COUNT) || (out_status == NULL)) {
        return TUE_DC_ERR_PARAM;
    }

    uint8_t idx = (uint8_t)(motor_id - 1U);
    dc_check_channel_fault(idx);

    *out_status = s_channels[idx];
    return TUE_DC_OK;
}

tue_dc_status_t tue_dc_get_info(tue_dc_info_t *out_info)
{
    if (!s_initialized) {
        tue_dc_init();
    }
    if (out_info == NULL) {
        return TUE_DC_ERR_PARAM;
    }

    out_info->motor_count  = TUE_DC_MOTOR_COUNT;
    out_info->pwm_freq_khz = (uint8_t)(s_pwm_freq_hz / 1000U);
    out_info->max_duty     = (uint16_t)TUE_DC_SPEED_MAX;
    out_info->timer_arr    = (uint16_t)s_timer_arr;
    out_info->driver_type  = TUE_DC_DRIVER_TYPE_DRV8872;

    return TUE_DC_OK;
}

bool tue_dc_get_fault(uint8_t motor_id)
{
    if ((motor_id < 1U) || (motor_id > TUE_DC_MOTOR_COUNT)) {
        return false;
    }
    uint8_t idx = (uint8_t)(motor_id - 1U);
    dc_check_channel_fault(idx);
    return s_channels[idx].fault_active;
}

uint8_t tue_dc_get_fault_mask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < TUE_DC_MOTOR_COUNT; i++) {
        dc_check_channel_fault(i);
        if (s_channels[i].fault_active) {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

void tue_dc_clear_faults(void)
{
    for (uint8_t i = 0U; i < TUE_DC_MOTOR_COUNT; i++) {
        s_channels[i].fault_latched = false;
        s_channels[i].fault_count   = 0U;
        dc_check_channel_fault(i);
    }
}

tue_dc_status_t tue_dc_set_pwm_frequency(uint32_t freq_hz)
{
    if ((freq_hz < TUE_DC_MIN_PWM_FREQ_HZ) || (freq_hz > TUE_DC_MAX_PWM_FREQ_HZ)) {
        return TUE_DC_ERR_PARAM;
    }

    uint32_t psc = (TUE_DC_TIMER_CLOCK_HZ / (freq_hz * 65536U));
    uint32_t arr = (TUE_DC_TIMER_CLOCK_HZ / ((psc + 1U) * freq_hz)) - 1U;
    if (arr > 65535U) {
        arr = 65535U;
    }

    s_pwm_freq_hz = freq_hz;
    dc_update_timer_base(psc, arr);

    /* Reapply active speeds with new ARR */
    for (uint8_t i = 0U; i < TUE_DC_MOTOR_COUNT; i++) {
        if (s_channels[i].mode == TUE_DC_MODE_FORWARD || s_channels[i].mode == TUE_DC_MODE_REVERSE) {
            (void)tue_dc_set_speed((uint8_t)(i + 1U), s_channels[i].speed);
        } else if (s_channels[i].mode == TUE_DC_MODE_BRAKE) {
            (void)tue_dc_brake((uint8_t)(i + 1U));
        } else {
            (void)tue_dc_coast((uint8_t)(i + 1U));
        }
    }

    return TUE_DC_OK;
}

uint32_t tue_dc_get_pwm_frequency(void)
{
    return s_pwm_freq_hz;
}

void tue_dc_set_auto_fault_protection(bool enable)
{
    s_auto_fault_protection = enable;
}

bool tue_dc_get_auto_fault_protection(void)
{
    return s_auto_fault_protection;
}

/* ========================================================================== */
/* Interrupt Service & Periodic Control Ramping                              */
/* ========================================================================== */

static void dc_service_channel_tick(uint8_t idx)
{
    if (idx >= TUE_DC_MOTOR_COUNT) {
        return;
    }

    /* 1. Periodic fault check */
    dc_check_channel_fault(idx);
    if (s_auto_fault_protection && s_channels[idx].fault_active) {
        return;
    }

    /* 2. Ramping check */
    if (s_ramp_rate_per_s == 0U) {
        return;
    }

    /* Do not ramp if motor was intentionally braked and target speed is 0 */
    if (s_channels[idx].mode == TUE_DC_MODE_BRAKE && s_target_speed[idx] == 0) {
        return;
    }

    int16_t current = s_channels[idx].speed;
    int16_t target  = s_target_speed[idx];

    if (current == target) {
        return;
    }

    /* Step size per 1 ms tick (rate_per_s / 1000) */
    int16_t step = (int16_t)(s_ramp_rate_per_s / 1000U);
    if (step == 0) {
        step = 1;
    }

    int16_t next;
    if (current < target) {
        next = current + step;
        if (next > target) {
            next = target;
        }
    } else {
        next = current - step;
        if (next < target) {
            next = target;
        }
    }

    dc_apply_speed(idx, next);
}

void tue_dc_on_fault_exti(uint16_t GPIO_Pin)
{
    uint8_t idx = 0xFFU;
    if (GPIO_Pin == DC_F1_Pin) {
        idx = 0U;
    } else if (GPIO_Pin == DC_F2_Pin) {
        idx = 1U;
    } else if (GPIO_Pin == DC_F3_Pin) {
        idx = 2U;
    } else if (GPIO_Pin == DC_F4_Pin) {
        idx = 3U;
    }

    if (idx < TUE_DC_MOTOR_COUNT) {
        /* DRV8872 nFAULT is active low (0 = fault asserted) */
        if (HAL_GPIO_ReadPin(s_hw[idx].fault_port, s_hw[idx].fault_pin) == GPIO_PIN_RESET) {
            s_channels[idx].fault_active = true;
            if (!s_channels[idx].fault_latched) {
                s_channels[idx].fault_latched = true;
                s_channels[idx].fault_count++;
            }
            if (s_auto_fault_protection) {
                /* Sub-microsecond instantaneous shutdown */
                dc_apply_hardware(idx, 0U, 0U);
                s_channels[idx].mode         = TUE_DC_MODE_COAST;
                s_channels[idx].speed        = 0;
                s_channels[idx].duty         = 0U;
                s_target_speed[idx]          = 0;
            }
        }
    }
}

void tue_dc_on_timer_elapsed(TIM_HandleTypeDef *htim)
{
    if (htim == NULL || !s_initialized) {
        return;
    }

    /* Compute divider to achieve ~1 kHz service rate from PWM timer updates */
    uint32_t div_target = s_pwm_freq_hz / 1000U;
    if (div_target == 0U) {
        div_target = 1U;
    }

    if (htim->Instance == TIM4) {
        static uint32_t div4 = 0U;
        if (++div4 >= div_target) {
            div4 = 0U;
            dc_service_channel_tick(0U); /* Motor 1 */
            dc_service_channel_tick(1U); /* Motor 2 */
        }
    } else if (htim->Instance == TIM3) {
        static uint32_t div3 = 0U;
        if (++div3 >= div_target) {
            div3 = 0U;
            dc_service_channel_tick(2U); /* Motor 3 */
            dc_service_channel_tick(3U); /* Motor 4 */
        }
    }
}

void tue_dc_set_ramp_rate(uint16_t rate_per_s)
{
    s_ramp_rate_per_s = rate_per_s;
}

uint16_t tue_dc_get_ramp_rate(void)
{
    return s_ramp_rate_per_s;
}

/* ========================================================================== */
/* Hardware Diagnostic Test Functions                                         */
/* ========================================================================== */

/* Global diagnostic variables visible in GDB / IDE Live Expressions */
volatile uint8_t  g_dc_test_step        = 0U; /* 1=FWD, 2=BRK, 3=REV, 4=CST */
volatile uint8_t  g_dc_test_fault_mask  = 0U; /* Active fault bitmask (bit0=M1..bit3=M4) */
volatile uint32_t g_dc_test_cycle_count = 0U; /* Completed test cycles */

uint8_t tue_dc_test_run_all(int16_t speed_permille, uint32_t duration_ms)
{
    if (!s_initialized) {
        tue_dc_init();
    }

    /* Start all 4 channels at specified low speed */
    (void)tue_dc_set_speed_all(speed_permille, speed_permille, speed_permille, speed_permille);

    uint8_t fault_mask = 0U;
    uint32_t elapsed = 0U;
    const uint32_t step_ms = 50U;

    while (elapsed < duration_ms) {
        HAL_Delay(step_ms);
        elapsed += step_ms;

        uint8_t current_faults = tue_dc_get_fault_mask();
        if (current_faults != 0U) {
            fault_mask |= current_faults;
        }

        /* Toggle LED every 200 ms to indicate execution */
        if ((elapsed % 200U) < step_ms) {
            HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
        }
    }

    /* Return all channels to coast */
    (void)tue_dc_coast(TUE_DC_MOTOR_ALL);

    return fault_mask;
}

void tue_dc_test_low_speed_loop(void)
{
    // if (!s_initialized) {
    //     tue_dc_init();
    // }

    // /* 25.0% low speed duty cycle: safe for initial bench/unloaded testing */
    // const int16_t LOW_SPEED = 250;

    while (1) {
        HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
        HAL_Delay(500);
        HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
        HAL_Delay(500);
        // g_dc_test_cycle_count++;

        // /* 1. FORWARD (+25% duty) on all 4 channels for 3 seconds */
        // g_dc_test_step = 1U;
        // (void)tue_dc_set_speed_all(LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED);
        // for (int i = 0; i < 30; i++) {
        //     HAL_Delay(100);
        //     g_dc_test_fault_mask = tue_dc_get_fault_mask();
        //     HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
        // }

        // /* 2. ACTIVE DYNAMIC BRAKE on all 4 channels for 1 second */
        // // g_dc_test_step = 2U;
        // // (void)tue_dc_brake(TUE_DC_MOTOR_ALL);
        // // for (int i = 0; i < 10; i++) {
        // //     HAL_Delay(100);
        // //     g_dc_test_fault_mask = tue_dc_get_fault_mask();
        // // }

        // // /* 3. REVERSE (-25% duty) on all 4 channels for 3 seconds */
        // // g_dc_test_step = 3U;
        // // (void)tue_dc_set_speed_all(-LOW_SPEED, -LOW_SPEED, -LOW_SPEED, -LOW_SPEED);
        // // for (int i = 0; i < 30; i++) {
        // //     HAL_Delay(100);
        // //     g_dc_test_fault_mask = tue_dc_get_fault_mask();
        // //     HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
        // // }

        // // /* 4. COAST (0% duty, freewheel/sleep) on all 4 channels for 1 second */
        // // g_dc_test_step = 4U;
        // // (void)tue_dc_coast(TUE_DC_MOTOR_ALL);
        // // for (int i = 0; i < 10; i++) {
        // //     HAL_Delay(100);
        // //     g_dc_test_fault_mask = tue_dc_get_fault_mask();
        // // }

        // /* Diagnostic fault alert: rapid blink pattern if any channel triggered nFAULT */
        // if (g_dc_test_fault_mask != 0U) {
        //     for (int i = 0; i < 10; i++) {
        //         HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
        //         HAL_Delay(50);
        //     }
        // }
    }
}
