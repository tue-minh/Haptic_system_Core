/**
  ******************************************************************************
  * @file           : tue_dc.h
  * @brief          : 4-Channel DC motor driver interface for DRV8872 using
  *                   TIM3 and TIM4 PWM on STM32F407 (MM_Control).
  ******************************************************************************
  * @attention
  *
  * Hardware mapping:
  *   Motor 1:
  *     - IN1    : PD12 -> TIM4_CH1
  *     - IN2    : PD13 -> TIM4_CH2
  *     - nFAULT : PD8  -> DC_F1 (active-low, open-drain)
  *   Motor 2:
  *     - IN1    : PD14 -> TIM4_CH3
  *     - IN2    : PD15 -> TIM4_CH4
  *     - nFAULT : PD9  -> DC_F2 (active-low, open-drain)
  *   Motor 3:
  *     - IN1    : PC6  -> TIM3_CH1
  *     - IN2    : PC7  -> TIM3_CH2
  *     - nFAULT : PD10 -> DC_F3 (active-low, open-drain)
  *   Motor 4:
  *     - IN1    : PC8  -> TIM3_CH3
  *     - IN2    : PC9  -> TIM3_CH4
  *     - nFAULT : PD11 -> DC_F4 (active-low, open-drain)
  *
  * DRV8872 Logic:
  *   IN1=0, IN2=0 -> Coast / Hi-Z / Sleep (fast decay)
  *   IN1=1, IN2=0 -> Forward (PWM on IN1, IN2 held LOW)
  *   IN1=0, IN2=1 -> Reverse (IN1 held LOW, PWM on IN2)
  *   IN1=1, IN2=1 -> Active Brake (low-side slow decay)
  *   nFAULT       -> Driven LOW on OCP (overcurrent), TSD (overtemp), UVLO.
  *
  ******************************************************************************
  */

#ifndef TUE_DC_H_
#define TUE_DC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* Exported constants --------------------------------------------------------*/
#define TUE_DC_MOTOR_COUNT              4U

/* Motor channel IDs (1-based index) */
#define TUE_DC_MOTOR_1                  1U
#define TUE_DC_MOTOR_2                  2U
#define TUE_DC_MOTOR_3                  3U
#define TUE_DC_MOTOR_4                  4U
#define TUE_DC_MOTOR_ALL                0xFFU

/* Speed bounds (per-mille: -1000 to +1000, 0.1% resolution) */
#define TUE_DC_SPEED_MAX                1000
#define TUE_DC_SPEED_MIN               -1000

/* PWM frequency bounds (Hz) */
#define TUE_DC_DEFAULT_PWM_FREQ_HZ      20000U  /* 20 kHz ultrasonic (silent) */
#define TUE_DC_MIN_PWM_FREQ_HZ          100U
#define TUE_DC_MAX_PWM_FREQ_HZ          50000U  /* 50 kHz cap (DRV8872 max 100k) */

/* Fault bitmasks */
#define TUE_DC_FAULT_M1_MASK            (1U << 0)
#define TUE_DC_FAULT_M2_MASK            (1U << 1)
#define TUE_DC_FAULT_M3_MASK            (1U << 2)
#define TUE_DC_FAULT_M4_MASK            (1U << 3)

/* Driver hardware identifier */
#define TUE_DC_DRIVER_TYPE_DRV8872      0x8872U

/* Exported types ------------------------------------------------------------*/

/**
  * @brief Return status codes for DC motor driver operations.
  */
typedef enum {
    TUE_DC_OK             = 0,
    TUE_DC_ERR_PARAM      = 1,
    TUE_DC_ERR_FAULT      = 2,
    TUE_DC_ERR_HAL        = 3,
    TUE_DC_ERR_BUSY       = 4
} tue_dc_status_t;

/**
  * @brief Operating mode of a DC motor channel.
  */
typedef enum {
    TUE_DC_MODE_COAST     = 0,  /* Both inputs LOW: Hi-Z coast / low power */
    TUE_DC_MODE_FORWARD   = 1,  /* Forward rotation (IN1=PWM, IN2=0) */
    TUE_DC_MODE_REVERSE   = 2,  /* Reverse rotation (IN1=0, IN2=PWM) */
    TUE_DC_MODE_BRAKE     = 3   /* Active brake (IN1=1, IN2=1) */
} tue_dc_mode_t;

/**
  * @brief Telemetry status for an individual DC motor channel.
  */
typedef struct {
    uint8_t       motor_id;       /* 1..4 */
    tue_dc_mode_t mode;           /* Current mode */
    int16_t       speed;          /* Current commanded speed (-1000..+1000) */
    uint16_t      duty;           /* Absolute duty cycle (0..1000 per-mille) */
    bool          fault_active;   /* Current raw nFAULT pin state (true = fault active / low) */
    bool          fault_latched;  /* Sticky fault flag since last clear */
    uint32_t      fault_count;    /* Number of fault occurrences */
} tue_dc_channel_status_t;

/**
  * @brief Controller capabilities and active configuration info.
  */
typedef struct {
    uint8_t  motor_count;         /* Number of channels (4) */
    uint8_t  pwm_freq_khz;        /* Active PWM frequency in kHz (e.g. 20) */
    uint16_t max_duty;            /* Full scale duty value (1000) */
    uint16_t timer_arr;           /* Active Timer ARR register value */
    uint16_t driver_type;         /* Hardware driver identifier (0x8872) */
} tue_dc_info_t;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialise the 4-channel DC motor subsystem.
  *         Configures TIM3 and TIM4 for 20 kHz PWM, initializes all duty cycles
  *         to 0 (COAST), arms the GPIO fault inputs (with internal pull-up),
  *         and starts PWM output generation on all 8 channels.
  * @retval TUE_DC_OK on success, or error code.
  */
tue_dc_status_t tue_dc_init(void);

/**
  * @brief  Check if the DC motor subsystem has been initialised.
  * @retval true if initialised.
  */
bool tue_dc_is_initialized(void);

/**
  * @brief  Set speed / duty cycle for a single motor channel.
  * @param  motor_id Channel identifier (1..4).
  * @param  speed    Signed speed (-1000 to +1000):
  *                  > 0: Forward rotation (duty = speed / 10) %
  *                  < 0: Reverse rotation (duty = -speed / 10) %
  *                  = 0: Coast / stop (or brake if configured)
  * @retval TUE_DC_OK on success, TUE_DC_ERR_PARAM if motor_id out of range.
  */
tue_dc_status_t tue_dc_set_speed(uint8_t motor_id, int16_t speed);

/**
  * @brief  Set speed for all 4 motor channels simultaneously in a single operation.
  * @param  m1 Speed for Motor 1 (-1000..+1000).
  * @param  m2 Speed for Motor 2 (-1000..+1000).
  * @param  m3 Speed for Motor 3 (-1000..+1000).
  * @param  m4 Speed for Motor 4 (-1000..+1000).
  * @retval TUE_DC_OK on success.
  */
tue_dc_status_t tue_dc_set_speed_all(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/**
  * @brief  Engage active dynamic braking (IN1=HIGH, IN2=HIGH) on a channel or all channels.
  * @param  motor_id 1..4 for specific channel, or TUE_DC_MOTOR_ALL (0xFF) / 0 for all channels.
  * @retval TUE_DC_OK on success.
  */
tue_dc_status_t tue_dc_brake(uint8_t motor_id);

/**
  * @brief  Put motor into Hi-Z coast / idle (IN1=LOW, IN2=LOW) on a channel or all channels.
  * @param  motor_id 1..4 for specific channel, or TUE_DC_MOTOR_ALL (0xFF) / 0 for all channels.
  * @retval TUE_DC_OK on success.
  */
tue_dc_status_t tue_dc_coast(uint8_t motor_id);

/**
  * @brief  Stop motor (alias for coast).
  * @param  motor_id 1..4 or TUE_DC_MOTOR_ALL / 0.
  * @retval TUE_DC_OK on success.
  */
tue_dc_status_t tue_dc_stop(uint8_t motor_id);

/**
  * @brief  Query full telemetry status of a specific motor channel.
  * @param  motor_id   Channel 1..4.
  * @param  out_status Pointer to output structure.
  * @retval TUE_DC_OK on success, TUE_DC_ERR_PARAM if invalid channel or NULL ptr.
  */
tue_dc_status_t tue_dc_get_channel_status(uint8_t motor_id, tue_dc_channel_status_t *out_status);

/**
  * @brief  Read subsystem capabilities and timing info.
  * @param  out_info Pointer to output structure.
  * @retval TUE_DC_OK on success.
  */
tue_dc_status_t tue_dc_get_info(tue_dc_info_t *out_info);

/**
  * @brief  Read the instantaneous fault pin (nFAULT) state for a given motor channel.
  * @param  motor_id Channel 1..4.
  * @retval true if fault is currently active (nFAULT is LOW).
  */
bool tue_dc_get_fault(uint8_t motor_id);

/**
  * @brief  Read the combined fault bitmask for all 4 channels.
  * @retval Bitmask where bit0=M1, bit1=M2, bit2=M3, bit3=M4. (1 = Fault active).
  */
uint8_t tue_dc_get_fault_mask(void);

/**
  * @brief  Clear latched sticky fault flags and reset fault counters.
  */
void tue_dc_clear_faults(void);

/**
  * @brief  Dynamically reconfigure PWM switching frequency for TIM3 and TIM4.
  * @param  freq_hz Frequency in Hertz (100 Hz to 50000 Hz).
  * @retval TUE_DC_OK on success, TUE_DC_ERR_PARAM if frequency out of range.
  */
tue_dc_status_t tue_dc_set_pwm_frequency(uint32_t freq_hz);

/**
  * @brief  Get the currently configured PWM frequency in Hertz.
  * @retval Active PWM frequency in Hz.
  */
uint32_t tue_dc_get_pwm_frequency(void);

/**
  * @brief  Enable or disable automatic hardware protection shut-off on fault.
  *         When enabled, if nFAULT is asserted active, the affected channel
  *         is automatically commanded to COAST (0 duty).
  * @param  enable true to enable auto-shut-off, false to disable.
  */
void tue_dc_set_auto_fault_protection(bool enable);

/**
  * @brief  Get the status of automatic fault protection.
  * @retval true if auto-protection is enabled.
  */
bool tue_dc_get_auto_fault_protection(void);

/* Interrupt Service Callbacks -----------------------------------------------*/

/**
  * @brief  EXTI Falling-edge ISR handler for DRV8872 active-low nFAULT pins.
  *         Invoked immediately from HAL_GPIO_EXTI_Callback() when an overcurrent
  *         (OCP) or overtemperature (TSD) event pulls nFAULT low on PD8..PD11.
  *         Instantly halts PWM on the faulted channel in sub-microsecond time.
  * @param  GPIO_Pin Pin mask that triggered the EXTI interrupt.
  */
void tue_dc_on_fault_exti(uint16_t GPIO_Pin);

/**
  * @brief  Timer update period elapsed ISR handler for TIM3 and TIM4.
  *         Invoked from HAL_TIM_PeriodElapsedCallback() on timer overflow.
  *         Provides a 1 kHz periodic service tick for smooth speed ramping
  *         and hardware watchdog supervision.
  * @param  htim Timer handle.
  */
void tue_dc_on_timer_elapsed(TIM_HandleTypeDef *htim);

/**
  * @brief  Configure velocity acceleration slew rate (ramp).
  * @param  rate_per_s Acceleration in per-mille per second (e.g. 2000 = 0.5s from 0 to 100%).
  *                     Set to 0 for instantaneous response (no ramping).
  */
void tue_dc_set_ramp_rate(uint16_t rate_per_s);

/**
  * @brief  Get the active velocity acceleration slew rate.
  * @retval Active rate in per-mille per second (0 = instantaneous).
  */
uint16_t tue_dc_get_ramp_rate(void);

/* Hardware Diagnostic Test Functions ----------------------------------------*/

/**
  * @brief  Standalone diagnostic test loop for all 4 DC motor channels.
  *         Runs all 4 channels at low speed (25% duty = 250 per-mille), cycling:
  *           1. Forward at +25% duty (3 seconds)
  *           2. Active Dynamic Brake (1 second)
  *           3. Reverse at -25% duty (3 seconds)
  *           4. Coast / Standby (1 second)
  *         Continuously checks nFAULT pins (DC_F1..DC_F4) and toggles LED_MCU.
  *         Does NOT return (infinite loop for bench testing before FreeRTOS).
  */
void tue_dc_test_low_speed_loop(void);

/**
  * @brief  Single-pass test: runs all 4 channels at low speed for a given duration.
  * @param  speed_permille Low speed value (-1000..+1000, e.g. 250 for 25% duty).
  * @param  duration_ms    Duration to run in milliseconds.
  * @retval Bitmask of any faults detected during the run (0 = no faults).
  */
uint8_t tue_dc_test_run_all(int16_t speed_permille, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* TUE_DC_H_ */
