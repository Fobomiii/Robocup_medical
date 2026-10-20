/**
 * @file GM65.c
 * @brief GM65 UART7 DMA+IDLE �� ɨ��ɹ������ ASCII ���
 */
#include "GM65.h"
#include <string.h>

#define GM65_RX_BUF_SIZE  256U

static uint8_t s_rx_dma[GM65_RX_BUF_SIZE] __attribute__((aligned(32)));
static char s_code[GM65_CODE_MAX];
static volatile uint16_t s_code_len;
static volatile uint8_t s_frame_ready;

static void gm65_start_rx(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart7, s_rx_dma, GM65_RX_BUF_SIZE) == HAL_OK)
  {
    if (huart7.hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
  }
}

static void gm65_store(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint16_t n = 0;

  if (len == 0U)
  {
    return;
  }

  for (i = 0; i < len && n < (GM65_CODE_MAX - 1U); i++)
  {
    uint8_t c = data[i];
    /* ȥ�������������������ɼ����� */
    if (c == '\r' || c == '\n' || c == '\0')
    {
      continue;
    }
    s_code[n++] = (char)c;
  }
  s_code[n] = '\0';
  s_code_len = n;
  if (n > 0U)
  {
    s_frame_ready = 1U;
  }
}

void GM65_Init(void)
{
  s_code[0] = '\0';
  s_code_len = 0;
  s_frame_ready = 0;
  memset(s_rx_dma, 0, sizeof(s_rx_dma));
  gm65_start_rx();
}

void GM65_OnRxEvent(uint16_t Size)
{
  if (Size > 0U && Size <= GM65_RX_BUF_SIZE)
  {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_rx_dma,
                                 (int32_t)(((Size + 31U) / 32U) * 32U));
#endif
    gm65_store(s_rx_dma, Size);
  }
  gm65_start_rx();
}

uint8_t GM65_FrameReady(void)
{
  return s_frame_ready;
}

void GM65_ClearFrameReady(void)
{
  s_frame_ready = 0;
}

const char *GM65_GetLastCode(void)
{
  return s_code;
}

uint16_t GM65_GetLastLen(void)
{
  return s_code_len;
}
