/**
 * @file GY614.c
 * @brief GY-614 UART �����ϱ����� �� ��ͷ���� BO/100
 *
 * ֡: 0xA4 | 0x03 | start_reg | len | data... | checksum
 * start_reg==0x07 ʱ: e,toH,toL,taH,taL,boH,boL
 * BO = (boH*256+boL)/100
 */
#include "GY614.h"
#include <string.h>

#define GY_ADDR       0xA4U
#define GY_CMD        0x03U
#define GY_REG_TEMP   0x07U
#define GY_BUF_MAX    24U
#define GY_LPF_ALPHA  0.2f   /* 0~1：越大越跟手，越小越稳 */

static uint8_t s_it_byte;
static uint8_t s_rebuf[GY_BUF_MAX];
static uint8_t s_i;
static volatile float s_temp_c;
static volatile float s_temp_lpf;
static uint8_t s_lpf_inited;

static void gy_feed(uint8_t b)
{
  uint8_t i;
  uint8_t sum;
  uint8_t need;

  s_rebuf[s_i++] = b;

  if (s_rebuf[0] != GY_ADDR)
  {
    s_i = 0;
    return;
  }
  if ((s_i == 2U) && (s_rebuf[1] != GY_CMD))
  {
    s_i = 0;
    return;
  }
  if ((s_i == 3U) && (s_rebuf[2] > 16U))
  {
    s_i = 0;
    return;
  }
  if ((s_i == 4U) && (s_rebuf[3] > 16U))
  {
    s_i = 0;
    return;
  }

  if (s_i <= 3U)
  {
    return;
  }

  need = (uint8_t)(s_rebuf[3] + 5U); /* len + ͷ4 + У��1 */
  if (need > GY_BUF_MAX)
  {
    s_i = 0;
    return;
  }

  if (s_i < need)
  {
    return;
  }

  /* ����һ֡ */
  sum = 0;
  for (i = 0; i < (uint8_t)(need - 1U); i++)
  {
    sum = (uint8_t)(sum + s_rebuf[i]);
  }
  if (sum == s_rebuf[need - 1U])
  {
    if (s_rebuf[2] == GY_REG_TEMP && s_rebuf[3] >= 7U)
    {
      /* bo @ [9],[10]：data 为 e,toH,toL,taH,taL,boH,boL */
      uint16_t bo = (uint16_t)(((uint16_t)s_rebuf[9] << 8) | s_rebuf[10]);
      float t = (float)bo / 100.0f;
      s_temp_c = t;
      if (s_lpf_inited == 0U)
      {
        s_temp_lpf = t;
        s_lpf_inited = 1U;
      }
      else
      {
        s_temp_lpf = GY_LPF_ALPHA * t + (1.0f - GY_LPF_ALPHA) * s_temp_lpf;
      }
    }
  }
  s_i = 0;
}

void GY614_Init(void)
{
  s_i = 0;
  s_temp_c = 0.f;
  s_temp_lpf = 0.f;
  s_lpf_inited = 0U;
  memset(s_rebuf, 0, sizeof(s_rebuf));
  (void)HAL_UART_AbortReceive_IT(&huart8);
  (void)HAL_UART_Receive_IT(&huart8, &s_it_byte, 1);
}

void GY614_OnUartRxCplt(void)
{
  gy_feed(s_it_byte);
  (void)HAL_UART_Receive_IT(&huart8, &s_it_byte, 1);
}

float getTemp(void)
{
  return s_temp_c;
}

float getTempLPF(void)
{
  return s_temp_lpf;
}
