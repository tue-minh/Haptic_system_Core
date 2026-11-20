/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief Wire frame sent to the SPI master (14 bytes, little-endian).
 *
 * No CRC on purpose: chip-select frames the transfer, and exactly one frame is
 * clocked per CS assertion, so its start and length are already unambiguous.
 * The SOF marker is kept purely as a sanity check.
 */
typedef struct __attribute__((packed))
{
  uint8_t  sof0;
  uint8_t  sof1;
  uint8_t  seq;
  uint8_t  status;
  uint16_t adc_raw[5];
} ItvSpiFrame_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITV_CH_COUNT          5U
#define ADC_REF_MV            3300U
#define ADC_MAX_COUNT         4095U
#define ADC_OFFSET_MV         200U
#define ADC_OFFSET_COUNTS     (((ADC_OFFSET_MV * ADC_MAX_COUNT) + (ADC_REF_MV / 2U)) / ADC_REF_MV)
#define ADC_FILTER_LEN        8U
#define ADC_SAMPLE_PERIOD_MS  10U
#define DEFAULT_LOG_PERIOD_MS 50U
#define MIN_LOG_PERIOD_MS     20U
#define MAX_LOG_PERIOD_MS     10000U
#define UART_TX_BUF_SIZE      192U
#define UART_RX_BUF_SIZE      64U
#define SPI_FRAME_SIZE        ((uint16_t)sizeof(ItvSpiFrame_t))
#define SPI_FRAME_SOF0        0xAAU
#define SPI_FRAME_SOF1        0x55U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint16_t adc_dma_buf[ITV_CH_COUNT];
static volatile uint16_t adc_latest[ITV_CH_COUNT];
static uint16_t          adc_ring[ITV_CH_COUNT][ADC_FILTER_LEN];
static uint32_t          adc_sum[ITV_CH_COUNT];
static uint8_t           adc_ring_idx = 0U;
static uint8_t           adc_ring_primed = 0U;
static volatile uint8_t  adc_busy = 0U;
static volatile uint8_t  adc_data_ready = 0U;
static volatile uint8_t  adc_valid = 0U;
static volatile uint8_t  adc_seq = 0U;
static volatile uint32_t adc_sample_count = 0U;
static volatile uint32_t adc_error_count = 0U;
static volatile uint8_t  adc_last_error = 0U;

static uint8_t           spi_tx_buf[SPI_FRAME_SIZE];
static uint8_t           spi_rx_buf[SPI_FRAME_SIZE];
static uint8_t           spi_frame_shadow[2][SPI_FRAME_SIZE];
static volatile uint8_t  spi_frame_front = 0U;
static volatile uint8_t  spi_adc_dirty = 0U;
static volatile uint8_t  spi_cs_active = 0U;
static volatile uint8_t  spi_cs_restart_pending = 0U;
static volatile uint16_t spi_tx_idx = 0U;
static volatile uint16_t spi_rx_idx = 0U;
static volatile uint32_t spi_tx_count = 0U;
static volatile uint32_t spi_error_count = 0U;
static volatile uint32_t spi_resync_count = 0U;

static uint8_t           uart_tx_buf[UART_TX_BUF_SIZE];
static uint8_t           uart_rx_dma_buf[UART_RX_BUF_SIZE];
static char              uart_cmd_buf[UART_RX_BUF_SIZE];
static volatile uint16_t uart_cmd_len = 0U;
static volatile uint8_t  uart_cmd_ready = 0U;
static volatile uint8_t  uart_tx_busy = 0U;
static volatile uint32_t uart_tx_count = 0U;
static volatile uint32_t uart_rx_count = 0U;
static volatile uint32_t uart_error_count = 0U;

static uint8_t           stream_enabled = 1U;
static uint32_t          log_period_ms = DEFAULT_LOG_PERIOD_MS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SPI1_PackFrame(uint8_t *buf);
static void SPI1_RearmSlaveDMA(void);
static void SPI1_ServiceCompletedFrame(void);
static void UART1_StartRxDMA(void);
static uint8_t UART1_SendDMA(const char *msg, uint32_t wait_timeout_ms);
static void ITV_LogSample(const uint16_t raw[ITV_CH_COUNT], uint32_t timestamp_ms, uint8_t wait_tx);
static void ITV_ProcessCommand(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SPI1_PackFrame(uint8_t *buf)
{
  ItvSpiFrame_t frame;
  frame.sof0 = SPI_FRAME_SOF0;
  frame.sof1 = SPI_FRAME_SOF1;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  frame.seq = adc_seq;
  frame.status = (adc_valid != 0U) ? 0x01U : 0x00U;
  if (adc_last_error != 0U)
  {
    frame.status |= 0x02U;
  }
  for (uint32_t i = 0U; i < ITV_CH_COUNT; i++)
  {
    frame.adc_raw[i] = adc_latest[i];
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  memcpy(buf, &frame, sizeof(frame));
}

static void SPI1_RearmSlaveDMA(void)
{
  if ((spi_cs_active != 0U) && (hspi1.State != HAL_SPI_STATE_READY))
  {
    return;
  }
  if (hspi1.State != HAL_SPI_STATE_READY)
  {
    (void)HAL_SPI_Abort(&hspi1);
  }
  /* Hardware-reset SPI1 when idle (CS HIGH) so any stray SCK/NSS glitch
   * that shifted the internal bit counter cannot wedge subsequent frames. */
  if (spi_cs_active == 0U)
  {
    __HAL_RCC_SPI1_FORCE_RESET();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __HAL_RCC_SPI1_RELEASE_RESET();
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
      spi_error_count++;
      return;
    }
  }
  __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
  memcpy(spi_tx_buf, spi_frame_shadow[spi_frame_front], SPI_FRAME_SIZE);
  spi_adc_dirty = 0U;

  if (HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, SPI_FRAME_SIZE) != HAL_OK)
  {
    spi_error_count++;
  }
}

/*
 * Re-arm after a completed frame.
 *
 * Runs in DMA/SPI interrupt context, so it must not block. HAL's completion
 * handler has already waited for BSY and set State back to READY by the time
 * this is called; if it did NOT (a stale error), arming again here would fail
 * and silently wedge the link, so fall back to the deferred restart in the
 * main loop instead.
 */
static void SPI1_ServiceCompletedFrame(void)
{
  if (hspi1.State != HAL_SPI_STATE_READY)
  {
    spi_cs_restart_pending = 1U;
    return;
  }
  /* If CS is still LOW (one-frame-per-CS protocol), wait for CS rising edge
   * before arming the next frame so we never have to HAL_SPI_Abort() a
   * pre-armed transfer when CS deasserts. */
  if (spi_cs_active != 0U)
  {
    return;
  }
  __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
  memcpy(spi_tx_buf, spi_frame_shadow[spi_frame_front], SPI_FRAME_SIZE);
  spi_adc_dirty = 0U;

  if (HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, SPI_FRAME_SIZE) != HAL_OK)
  {
    spi_error_count++;
    spi_cs_restart_pending = 1U;
  }
}

static uint32_t ITV_RawToMillivolts(uint16_t raw)
{
  return ((uint32_t)raw * ADC_REF_MV + (ADC_MAX_COUNT / 2U)) / ADC_MAX_COUNT;
}

static void UART1_StartRxDMA(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buf, UART_RX_BUF_SIZE) == HAL_OK)
  {
    if (huart1.hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
  }
}

static uint8_t UART1_SendDMA(const char *msg, uint32_t wait_timeout_ms)
{
  size_t len = strlen(msg);
  if (len == 0U)
  {
    return 1U;
  }
  if (len >= UART_TX_BUF_SIZE)
  {
    len = UART_TX_BUF_SIZE - 1U;
  }

  uint32_t start = HAL_GetTick();
  while (uart_tx_busy != 0U)
  {
    if ((wait_timeout_ms == 0U) || ((HAL_GetTick() - start) >= wait_timeout_ms))
    {
      return 0U;
    }
  }

  memcpy(uart_tx_buf, msg, len);
  uart_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, (uint16_t)len) != HAL_OK)
  {
    uart_tx_busy = 0U;
    return 0U;
  }
  return 1U;
}

static void ITV_LogSample(const uint16_t raw[ITV_CH_COUNT], uint32_t timestamp_ms, uint8_t wait_tx)
{
  char line[UART_TX_BUF_SIZE];
  uint32_t mv[ITV_CH_COUNT];
  for (uint32_t i = 0U; i < ITV_CH_COUNT; i++)
  {
    mv[i] = ITV_RawToMillivolts(raw[i]);
  }

  (void)snprintf(
      line, sizeof(line),
      "[t=%lums] RAW:%u,%u,%u,%u,%u | mV:%lu,%lu,%lu,%lu,%lu\r\n",
      (unsigned long)timestamp_ms,
      (unsigned int)raw[0], (unsigned int)raw[1], (unsigned int)raw[2],
      (unsigned int)raw[3], (unsigned int)raw[4],
      (unsigned long)mv[0], (unsigned long)mv[1], (unsigned long)mv[2],
      (unsigned long)mv[3], (unsigned long)mv[4]);

  (void)UART1_SendDMA(line, wait_tx ? 50U : 0U);
}

static void ITV_ProcessCommand(void)
{
  char cmd[UART_RX_BUF_SIZE];
  uint16_t len = 0U;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (uart_cmd_ready != 0U)
  {
    len = uart_cmd_len;
    if (len >= UART_RX_BUF_SIZE)
    {
      len = UART_RX_BUF_SIZE - 1U;
    }
    memcpy(cmd, uart_cmd_buf, len);
    cmd[len] = '\0';
    uart_cmd_ready = 0U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (len == 0U)
  {
    return;
  }

  while ((len > 0U) && ((cmd[len - 1U] == '\r') || (cmd[len - 1U] == '\n') ||
                        (cmd[len - 1U] == ' ') || (cmd[len - 1U] == '\t')))
  {
    cmd[--len] = '\0';
  }

  char *p = cmd;
  while ((*p == ' ') || (*p == '\t'))
  {
    p++;
  }
  if (*p == '\0')
  {
    return;
  }

  char reply[UART_TX_BUF_SIZE];
  if ((strcmp(p, "help") == 0) || (strcmp(p, "?") == 0))
  {
    (void)snprintf(reply, sizeof(reply),
                   "CMD: start | stop | read | rate <20..10000ms> | stat\r\n");
    (void)UART1_SendDMA(reply, 50U);
  }
  else if (strcmp(p, "start") == 0)
  {
    stream_enabled = 1U;
    (void)snprintf(reply, sizeof(reply), "OK: stream ON (%lu ms)\r\n",
                   (unsigned long)log_period_ms);
    (void)UART1_SendDMA(reply, 50U);
  }
  else if (strcmp(p, "stop") == 0)
  {
    stream_enabled = 0U;
    (void)UART1_SendDMA("OK: stream OFF\r\n", 50U);
  }
  else if (strcmp(p, "read") == 0)
  {
    uint16_t snapshot[ITV_CH_COUNT];
    primask = __get_PRIMASK();
    __disable_irq();
    for (uint32_t i = 0U; i < ITV_CH_COUNT; i++)
    {
      snapshot[i] = adc_latest[i];
    }
    if (primask == 0U)
    {
      __enable_irq();
    }
    ITV_LogSample(snapshot, HAL_GetTick(), 1U);
  }
  else if (strncmp(p, "rate ", 5) == 0)
  {
    long req = strtol(p + 5, NULL, 10);
    if ((req >= (long)MIN_LOG_PERIOD_MS) && (req <= (long)MAX_LOG_PERIOD_MS))
    {
      log_period_ms = (uint32_t)req;
      (void)snprintf(reply, sizeof(reply), "OK: rate=%lu ms\r\n",
                     (unsigned long)log_period_ms);
    }
    else
    {
      (void)snprintf(reply, sizeof(reply), "ERR: rate must be 20..10000 ms\r\n");
    }
    (void)UART1_SendDMA(reply, 50U);
  }
  else if (strcmp(p, "stat") == 0)
  {
    (void)snprintf(
        reply, sizeof(reply),
        "STAT: adc=%lu adc_err=%lu spi=%lu spi_err=%lu resync=%lu tx=%lu rx=%lu u_err=%lu\r\n",
        (unsigned long)adc_sample_count,
        (unsigned long)adc_error_count,
        (unsigned long)spi_tx_count,
        (unsigned long)spi_error_count,
        (unsigned long)spi_resync_count,
        (unsigned long)uart_tx_count,
        (unsigned long)uart_rx_count,
        (unsigned long)uart_error_count);
    (void)UART1_SendDMA(reply, 50U);
  }
  else
  {
    (void)snprintf(reply, sizeof(reply), "ERR: unknown cmd '%s' (try help)\r\n", p);
    (void)UART1_SendDMA(reply, 50U);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  UART1_StartRxDMA();
  (void)UART1_SendDMA(
      "\r\nITV_SLAVE_READY: ADC1 5ch(PA0-PA4) | USART1 115200 | SPI1 Slave 16B DMA+IRQ\r\n",
      50U);

  SPI1_PackFrame(spi_frame_shadow[0]);
  spi_frame_front = 0U;
  /* Keep SPI1 disabled (SPE = 0) while CS is HIGH so SCK glitches cannot
   * shift bits between frames; EXTI falling edge enables it per transfer. */
  SPI1->CR2 = 0U;
  SPI1->CR1 = SPI_CR1_SSM;

  uint32_t last_sample_tick = HAL_GetTick();
  uint32_t last_log_tick = HAL_GetTick();

  adc_busy = 1U;
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ITV_CH_COUNT) != HAL_OK)
  {
    adc_busy = 0U;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (uart_cmd_ready != 0U)
    {
      ITV_ProcessCommand();
    }

    if ((adc_busy == 0U) && ((now - last_sample_tick) >= ADC_SAMPLE_PERIOD_MS))
    {
      last_sample_tick = now;
      adc_busy = 1U;
      HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
      if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ITV_CH_COUNT) != HAL_OK)
      {
        adc_busy = 0U;
        adc_error_count++;
        adc_last_error = 1U;
      }
    }

    if (spi_cs_restart_pending != 0U)
    {
      spi_cs_restart_pending = 0U;
      if (spi_cs_active == 0U)
      {
        if (hspi1.State != HAL_SPI_STATE_READY)
        {
          spi_resync_count++;
        }
        SPI1_RearmSlaveDMA();
      }
      else if (hspi1.State == HAL_SPI_STATE_READY)
      {
        SPI1_RearmSlaveDMA();
      }
    }

    if ((stream_enabled != 0U) && (adc_data_ready != 0U) &&
        ((now - last_log_tick) >= log_period_ms) && (uart_tx_busy == 0U))
    {
      uint16_t snapshot[ITV_CH_COUNT];
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      for (uint32_t i = 0U; i < ITV_CH_COUNT; i++)
      {
        snapshot[i] = adc_latest[i];
      }
      adc_data_ready = 0U;
      if (primask == 0U)
      {
        __enable_irq();
      }

      last_log_tick = now;
      ITV_LogSample(snapshot, now, 0U);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    if (adc_ring_primed == 0U)
    {
      for (uint32_t i = 0U; i < ITV_CH_COUNT; i++)
      {
        uint16_t raw = adc_dma_buf[i];
        uint16_t val = (raw > ADC_OFFSET_COUNTS) ? (uint16_t)(raw - ADC_OFFSET_COUNTS) : 0U;
        for (uint32_t k = 0U; k < ADC_FILTER_LEN; k++)
        {
          adc_ring[i][k] = val;
        }
        adc_sum[i] = (uint32_t)val * ADC_FILTER_LEN;
        adc_latest[i] = val;
      }
      adc_ring_primed = 1U;
      adc_ring_idx = 0U;
    }
    else
    {
      for (uint32_t i = 0U; i < ITV_CH_COUNT; i++)
      {
        uint16_t raw = adc_dma_buf[i];
        uint16_t val = (raw > ADC_OFFSET_COUNTS) ? (uint16_t)(raw - ADC_OFFSET_COUNTS) : 0U;
        adc_sum[i] -= adc_ring[i][adc_ring_idx];
        adc_ring[i][adc_ring_idx] = val;
        adc_sum[i] += val;
        adc_latest[i] = (uint16_t)((adc_sum[i] + (ADC_FILTER_LEN / 2U)) / ADC_FILTER_LEN);
      }
      adc_ring_idx = (uint8_t)((adc_ring_idx + 1U) % ADC_FILTER_LEN);
    }

    adc_seq++;
    adc_valid = 1U;
    adc_last_error = 0U;
    adc_sample_count++;
    adc_data_ready = 1U;

    uint8_t back = (uint8_t)(spi_frame_front ^ 1U);
    SPI1_PackFrame(spi_frame_shadow[back]);
    spi_frame_front = back;
    spi_adc_dirty = 1U;
    adc_busy = 0U;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_error_count++;
    adc_last_error = 1U;
    adc_busy = 0U;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    spi_tx_count++;
    SPI1_ServiceCompletedFrame();
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    spi_error_count++;
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    if (hspi->State == HAL_SPI_STATE_READY)
    {
      SPI1_ServiceCompletedFrame();
    }
    else
    {
      spi_cs_restart_pending = 1U;
    }
  }
}

void SPI1_Slave_IRQHandler(void)
{
  uint32_t sr = SPI1->SR;
  if ((sr & SPI_SR_RXNE) != 0U)
  {
    uint8_t rx = (uint8_t)SPI1->DR;
    if (spi_rx_idx < SPI_FRAME_SIZE)
    {
      spi_rx_buf[spi_rx_idx++] = rx;
    }
  }
  if ((sr & SPI_SR_TXE) != 0U)
  {
    if (spi_tx_idx < SPI_FRAME_SIZE)
    {
      SPI1->DR = spi_tx_buf[spi_tx_idx++];
    }
    else
    {
      SPI1->DR = 0x00U;
      SPI1->CR2 &= ~SPI_CR2_TXEIE;
    }
  }
  if ((sr & SPI_SR_OVR) != 0U)
  {
    (void)SPI1->DR;
    (void)SPI1->SR;
    spi_error_count++;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SPI1_NSS_Pin)
  {
    if (HAL_GPIO_ReadPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin) == GPIO_PIN_RESET)
    {
      spi_cs_active = 1U;
      /* Hard-reset SPI1 at CS falling edge so any prior glitch is cleared */
      __HAL_RCC_SPI1_FORCE_RESET();
      __NOP();
      __NOP();
      __HAL_RCC_SPI1_RELEASE_RESET();
      SPI1->CR1 = SPI_CR1_SSM;
      SPI1->CR2 = 0U;
      memcpy(spi_tx_buf, spi_frame_shadow[spi_frame_front], SPI_FRAME_SIZE);
      spi_tx_idx = 1U;
      spi_rx_idx = 0U;
      SPI1->DR = spi_tx_buf[0];
      SPI1->CR2 = SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE;
      SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SPE;
    }
    else
    {
      spi_cs_active = 0U;
      SPI1->CR2 = 0U;
      SPI1->CR1 = SPI_CR1_SSM;
      if (spi_rx_idx >= SPI_FRAME_SIZE)
      {
        spi_tx_count++;
      }
      else if (spi_rx_idx > 0U)
      {
        spi_resync_count++;
      }
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_tx_count++;
    uart_tx_busy = 0U;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    if ((Size > 0U) && (uart_cmd_ready == 0U))
    {
      uint16_t copy_len = (Size < UART_RX_BUF_SIZE) ? Size : (UART_RX_BUF_SIZE - 1U);
      memcpy(uart_cmd_buf, uart_rx_dma_buf, copy_len);
      uart_cmd_len = copy_len;
      uart_cmd_ready = 1U;
    }
    uart_rx_count++;
    UART1_StartRxDMA();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_error_count++;
    uart_tx_busy = 0U;
    UART1_StartRxDMA();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin); HAL_Delay(50);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
