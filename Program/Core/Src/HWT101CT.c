/**
 * @file HWT101CT.c
 * @brief HWT101CT-TTL USART1 DMA+IDLE ???
 *
 * ?: 0x55 | TYPE | 8B | SUM
 *   TYPE=0x52 ?????  Wz @ DATA3  -> /32768*2000 ??/s
 *   TYPE=0x53 ???    Yaw @ DATA3 -> /32768*180 ??
 */
#include "HWT101CT.h"
#include "GM65.h"
#include <string.h>

#define HWT_RX_BUF_SIZE  64U
#define HWT_FRAME_LEN    11U

static uint8_t s_rx_dma[HWT_RX_BUF_SIZE] __attribute__((aligned(32)));
static volatile float    s_yaw;
static volatile float    s_wz;
static volatile uint16_t s_ver;
static volatile uint32_t s_last_ms;

volatile float   hwt_zangle = 0.f;  /* Keil Debug: HWT101 航向角 (°) */
volatile float pos_z = 0.f;
volatile uint8_t hwt_online = 0U;   /* Keil Debug: 500ms 内有数据则为 1 */

static int16_t rd_i16(const uint8_t *p)
{
  return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static uint8_t checksum_ok(const uint8_t *f)
{
  uint8_t sum = 0;
  uint8_t i;
  for (i = 0; i < 10U; i++)
  {
    sum = (uint8_t)(sum + f[i]);
  }
  return (sum == f[10]) ? 1U : 0U;
}

static void parse_frame(const uint8_t *f)
{
  if (f[0] != 0x55U || checksum_ok(f) == 0U)
  {
    return;
  }

  if (f[1] == 0x52U)
  {
    /* ?????: Wx Wy Wz T ???? Wz ?? offset 6 */
    s_wz = (float)rd_i16(&f[6]) / 32768.f * 2000.f;
    s_last_ms = HAL_GetTick();
    hwt_online = 1U;
  }
  else if (f[1] == 0x53U)
  {
    /* ???: Roll Pitch Yaw Ver ???? Yaw ?? offset 6 */
    s_yaw = (float)rd_i16(&f[6]) / 32768.f * 180.f;
    hwt_zangle = s_yaw;
    pos_z = s_yaw;
    s_ver = (uint16_t)rd_i16(&f[8]);
    s_last_ms = HAL_GetTick();
    hwt_online = 1U;
  }
}

static void feed_bytes(const uint8_t *data, uint16_t len)
{
  static uint8_t buf[HWT_FRAME_LEN];
  static uint8_t idx;
  uint16_t i;

  for (i = 0; i < len; i++)
  {
    uint8_t b = data[i];
    if (idx == 0U)
    {
      if (b != 0x55U)
      {
        continue;
      }
      buf[idx++] = b;
    }
    else
    {
      buf[idx++] = b;
      if (idx >= HWT_FRAME_LEN)
      {
        parse_frame(buf);
        idx = 0U;
      }
    }
  }
}

static void hwt_start_rx(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx_dma, HWT_RX_BUF_SIZE) == HAL_OK)
  {
    if (huart1.hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
  }
}

void HWT101_Init(void)
{
  s_yaw = 0.f;
  s_wz = 0.f;
  s_ver = 0;
  s_last_ms = 0;
  hwt_zangle = 0.f;
  pos_z = 0.f;
  hwt_online = 0U;
  memset(s_rx_dma, 0, sizeof(s_rx_dma));
  hwt_start_rx();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    if (Size > 0U && Size <= HWT_RX_BUF_SIZE)
    {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
      SCB_InvalidateDCache_by_Addr((uint32_t *)s_rx_dma,
                                   (int32_t)(((Size + 31U) / 32U) * 32U));
#endif
      feed_bytes(s_rx_dma, Size);
    }
    hwt_start_rx();
  }
  else if (huart->Instance == UART7)
  {
    GM65_OnRxEvent(Size);
  }
}

uint8_t HWT101_IsOnline(void)
{
  if (s_last_ms == 0U)
  {
    hwt_online = 0U;
    return 0U;
  }
  hwt_online = ((HAL_GetTick() - s_last_ms) < 500U) ? 1U : 0U;
  return hwt_online;
}

float HWT101_GetYaw(void)
{
  return s_yaw;
}

float HWT101_GetWz(void)
{
  return s_wz;
}

uint16_t HWT101_GetVersion(void)
{
  return s_ver;
}

static void wit_write(uint8_t reg, uint16_t val)
{
  uint8_t cmd[5];
  cmd[0] = 0xFF;
  cmd[1] = 0xAA;
  cmd[2] = reg;
  cmd[3] = (uint8_t)(val & 0xFFU);
  cmd[4] = (uint8_t)(val >> 8);
  (void)HAL_UART_Transmit(&huart1, cmd, 5, 50);
}

void HWT101_CaliYaw(void)
{
  wit_write(0x69U, 0xB588U); /* ???? KEY */
  HAL_Delay(5);
  wit_write(0x76U, 0x0000U); /* CALIYAW */
}
