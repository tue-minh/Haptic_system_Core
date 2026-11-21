/**
  ******************************************************************************
  * @file    tue_ps2.h
  * @brief   PS2 wireless controller receiver driver (SPI3 master).
  ******************************************************************************
  * @attention
  *
  * The STM32F407 acts as the SPI master on SPI3 (PC10 SCK, PC11 MISO,
  * PC12 MOSI) with PD4 (SPI3_CS1) as the active-LOW ATT line, replacing the
  * PlayStation console. A background FreeRTOS task polls the receiver board
  * every TUE_PS2_POLL_PERIOD_MS, keeps a thread-safe snapshot for the host and
  * drives the controller's two vibration motors.
  *
  * SPI3 is dedicated to this link: tue_spi_adc never touches it.
  *
  * See PS2_CONTROLLER_GUIDE.md for wiring and protocol details.
  ******************************************************************************
  */

#ifndef TUE_PS2_H
#define TUE_PS2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Active-HIGH button masks (1 = pressed). */
#define TUE_PS2_BTN_SELECT    (1U << 0)
#define TUE_PS2_BTN_L3        (1U << 1)
#define TUE_PS2_BTN_R3        (1U << 2)
#define TUE_PS2_BTN_START     (1U << 3)
#define TUE_PS2_BTN_UP        (1U << 4)
#define TUE_PS2_BTN_RIGHT     (1U << 5)
#define TUE_PS2_BTN_DOWN      (1U << 6)
#define TUE_PS2_BTN_LEFT      (1U << 7)
#define TUE_PS2_BTN_L2        (1U << 8)
#define TUE_PS2_BTN_R2        (1U << 9)
#define TUE_PS2_BTN_L1        (1U << 10)
#define TUE_PS2_BTN_R1        (1U << 11)
#define TUE_PS2_BTN_TRIANGLE  (1U << 12)
#define TUE_PS2_BTN_CIRCLE    (1U << 13)
#define TUE_PS2_BTN_CROSS     (1U << 14)
#define TUE_PS2_BTN_SQUARE    (1U << 15)

#define TUE_PS2_POLL_PERIOD_MS  20U

/* Device mode bytes reported by the receiver (rx[1]). */
#define TUE_PS2_MODE_DIGITAL    0x41U
#define TUE_PS2_MODE_ANALOG     0x73U
#define TUE_PS2_MODE_ANALOG_PR  0x79U

typedef struct {
    bool     connected;      /* true when the last reply carried rx[2] == 0x5A */
    uint8_t  mode;           /* 0x41 digital, 0x73 analog, 0x79 analog+pressure */
    uint16_t buttons;        /* current active-HIGH button bitmask (A/B 2-byte) */
    uint16_t buttons_prev;   /* previous poll bitmask (for edge detection) */
    uint8_t  lx, ly;         /* left stick  (0..255, center ~127) */
    uint8_t  rx, ry;         /* right stick (0..255, center ~127) */
} tue_ps2_state_t;

/**
  * @brief  Start the PS2 polling task on SPI3.
  *         Call once after the RTOS scheduler is running. The task configures
  *         the controller for analog mode and polls it at 50 Hz.
  */
void tue_ps2_init(void);

/**
  * @brief  Copy the latest controller snapshot (thread-safe).
  */
void tue_ps2_get_state(tue_ps2_state_t *out);

/**
  * @brief  Set the vibration motors used by the next poll.
  * @param  small  small motor, 0 = off, non-zero = on
  * @param  large  large motor intensity, 0x00 = off, 0x40..0xFF = spinning
  */
void tue_ps2_set_motors(uint8_t small, uint8_t large);

/**
  * @brief  Read back the motor values currently applied.
  */
void tue_ps2_get_motors(uint8_t *small, uint8_t *large);

#ifdef __cplusplus
}
#endif

#endif /* TUE_PS2_H */
