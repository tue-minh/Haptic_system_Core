/**
  ******************************************************************************
  * @file    tue_ps2.c
  * @brief   PS2 wireless controller receiver driver (SPI3 master + poll task).
  ******************************************************************************
  * @attention
  *
  * Wire format: SPI mode 3 (CPOL=1, CPHA=2Edge), LSB first, <= 500 kHz.
  * ATT is PD4 (SPI3_CS1), held HIGH between packets. Each packet is sent one
  * byte at a time with ~20 us between bytes because the wireless receiver
  * needs time to prepare the next response byte.
  ******************************************************************************
  */

#include "tue_ps2.h"

#include <string.h>

#include "cmsis_os2.h"
#include "main.h"
#include "spi.h"

/* ATT (chip select) on SPI3_CS1 / PD4. */
#define PS2_ATT_PORT           SPI3_CS1_GPIO_Port
#define PS2_ATT_PIN            SPI3_CS1_Pin

#define PS2_BYTE_DELAY_US      20U
#define PS2_ATT_DELAY_US       15U
#define PS2_SPI_TIMEOUT_MS     5U
/* Pairing retry while the receiver/controller is absent. */
#define PS2_RETRY_CONFIG_MS    1000U
/* Re-send the analog-mode sequence at most this often if the handheld drops
 * back to digital (0x41) after sleeping. */
#define PS2_REANALOG_MS        500U

static bool                    s_initialized = false;
static osThreadId_t            s_task        = NULL;
static volatile uint8_t        s_small_motor = 0U;
static volatile uint8_t        s_large_motor = 0U;
static uint32_t                s_last_config_tick = 0U;

/* Snapshot published to the USB command handlers. Guarded by __disable_irq(). */
static tue_ps2_state_t         s_state;

/* Private function prototypes -----------------------------------------------*/
static void ps2_task(void *argument);
static void ps2_configure_analog(void);
static bool ps2_poll_once(tue_ps2_state_t *st, uint8_t small, uint8_t large);

/* Microsecond pacing -------------------------------------------------------*/

static void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

/* SPI framing --------------------------------------------------------------*/

static void ps2_transact(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    HAL_GPIO_WritePin(PS2_ATT_PORT, PS2_ATT_PIN, GPIO_PIN_RESET);
    delay_us(PS2_ATT_DELAY_US);

    for (uint8_t i = 0U; i < len; i++) {
        uint8_t tx_b = tx[i];
        uint8_t rx_b = 0xFFU;

        (void)HAL_SPI_TransmitReceive(&hspi3, &tx_b, &rx_b, 1U,
                                      PS2_SPI_TIMEOUT_MS);
        rx[i] = rx_b;
        delay_us(PS2_BYTE_DELAY_US);
    }

    HAL_GPIO_WritePin(PS2_ATT_PORT, PS2_ATT_PIN, GPIO_PIN_SET);
    delay_us(PS2_ATT_DELAY_US);
}

/*
 * Enter configuration mode, lock analog mode, map the vibration motors and
 * exit configuration mode. Replies are ignored on purpose: a missing
 * controller simply leaves the link in the disconnected state.
 */
static void ps2_configure_analog(void)
{
    uint8_t rx[9];

    const uint8_t cmd_poll[5]   = {0x01, 0x42, 0x00, 0x00, 0x00};
    const uint8_t cmd_enter[5]  = {0x01, 0x43, 0x00, 0x01, 0x00};
    const uint8_t cmd_analog[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    const uint8_t cmd_motors[9] = {0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t cmd_exit[9]   = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

    ps2_transact(cmd_poll, rx, sizeof(cmd_poll));
    delay_us(200U);
    ps2_transact(cmd_enter, rx, sizeof(cmd_enter));
    delay_us(200U);
    ps2_transact(cmd_analog, rx, sizeof(cmd_analog));
    delay_us(200U);
    ps2_transact(cmd_motors, rx, sizeof(cmd_motors));
    delay_us(200U);
    ps2_transact(cmd_exit, rx, sizeof(cmd_exit));

    s_last_config_tick = HAL_GetTick();
}

static bool ps2_poll_once(tue_ps2_state_t *st, uint8_t small, uint8_t large)
{
    uint8_t tx[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[9] = {0};

    tx[3] = (small != 0U) ? 0xFFU : 0x00U;
    tx[4] = large;

    ps2_transact(tx, rx, sizeof(tx));

    if (rx[2] != 0x5AU) {
        st->connected = false;
        st->mode      = rx[1];
        return false;
    }

    st->connected    = true;
    st->mode         = rx[1];
    st->buttons_prev = st->buttons;
    st->buttons      = (uint16_t)((uint8_t)(~rx[3]) |
                                  ((uint16_t)(uint8_t)(~rx[4]) << 8));

    if ((rx[1] & 0xF0U) == 0x70U) {
        st->lx = rx[7];
        st->ly = rx[8];
        st->rx = rx[5];
        st->ry = rx[6];
    } else {
        st->lx = 0x7FU;
        st->ly = 0x7FU;
        st->rx = 0x7FU;
        st->ry = 0x7FU;
    }

    return true;
}

/* Poll task ----------------------------------------------------------------*/

static void ps2_task(void *argument)
{
    (void)argument;
    tue_ps2_state_t live;

    memset(&live, 0, sizeof(live));
    live.lx = 0x7FU;
    live.ly = 0x7FU;
    live.rx = 0x7FU;
    live.ry = 0x7FU;

    osDelay(50U);
    ps2_configure_analog();

    for (;;) {
        uint8_t small = s_small_motor;
        uint8_t large = s_large_motor;

        (void)ps2_poll_once(&live, small, large);

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_state = live;
        __set_PRIMASK(primask);

        uint32_t now = HAL_GetTick();
        if (!live.connected) {
            if ((now - s_last_config_tick) >= PS2_RETRY_CONFIG_MS) {
                ps2_configure_analog();
            }
        } else if (live.mode == TUE_PS2_MODE_DIGITAL) {
            if ((now - s_last_config_tick) >= PS2_REANALOG_MS) {
                ps2_configure_analog();
            }
        }

        osDelay(TUE_PS2_POLL_PERIOD_MS);
    }
}

/* Public API ---------------------------------------------------------------*/

void tue_ps2_init(void)
{
    if (s_initialized) {
        return;
    }

    dwt_delay_init();
    HAL_GPIO_WritePin(PS2_ATT_PORT, PS2_ATT_PIN, GPIO_PIN_SET);

    memset(&s_state, 0, sizeof(s_state));
    s_state.lx = 0x7FU;
    s_state.ly = 0x7FU;
    s_state.rx = 0x7FU;
    s_state.ry = 0x7FU;

    s_small_motor = 0U;
    s_large_motor = 0U;

    const osThreadAttr_t task_attr = {
        .name = "tue_ps2",
        .stack_size = 1024U,
        .priority = (osPriority_t)osPriorityNormal,
    };
    s_task = osThreadNew(ps2_task, NULL, &task_attr);
    s_initialized = (s_task != NULL);
}

void tue_ps2_get_state(tue_ps2_state_t *out)
{
    if (out == NULL) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = s_state;
    __set_PRIMASK(primask);
}

void tue_ps2_set_motors(uint8_t small, uint8_t large)
{
    s_small_motor = (small != 0U) ? 0xFFU : 0x00U;
    s_large_motor = large;
}

void tue_ps2_get_motors(uint8_t *small, uint8_t *large)
{
    if (small != NULL) {
        *small = s_small_motor;
    }
    if (large != NULL) {
        *large = s_large_motor;
    }
}
