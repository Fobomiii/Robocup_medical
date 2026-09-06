/**
 * @file OPS.c
 * @brief OPS UART driver for STM32H7 HAL — RC_Old frame compatible
 *
 * Frame: 0x0D 0x0A + 6x float(24B) + 0x0A 0x0D
 * Stored into OPS like Master DMA layout (floats at ActVal[1..6])
 */
#include "OPS.h"
#include "cmsis_os.h"
#include "MAX30102.h"
#include "GY614.h"
#include <string.h>

Union_OPS OPS;

static uint8_t s_rx_byte;
static volatile uint8_t s_frame_ready;
static volatile uint32_t s_last_frame_ms;

/* RX state machine */
static uint8_t s_count;
static uint8_t s_i;
static union {
  uint8_t data[24];
  float ActVal[6];
} s_payload;

static void OPS_SendBytes(const uint8_t *buf, uint16_t len)
{
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 100);
}

static void OPS_SendString(const char *str)
{
  while (*str)
  {
    uint8_t c = (uint8_t)(*str++);
    OPS_SendBytes(&c, 1);
  }
}

static void OPS_ApplyPayload(void)
{
  /* Align with Master Location.h / DMA layout */
  OPS.data[2] = 0x0D;
  OPS.data[3] = 0x0A;
  memcpy(&OPS.data[4], s_payload.data, 24);
  OPS.data[28] = 0x0A;
  OPS.data[29] = 0x0D;

  /* ActVal[1..6] overlay data[4..27] after little-endian float layout */
  OPS.ActVal[1] = s_payload.ActVal[0];
  OPS.ActVal[2] = s_payload.ActVal[1];
  OPS.ActVal[3] = s_payload.ActVal[2];
  OPS.ActVal[4] = s_payload.ActVal[3];
  OPS.ActVal[5] = s_payload.ActVal[4];
  OPS.ActVal[6] = s_payload.ActVal[5];

  s_last_frame_ms = HAL_GetTick();
  s_frame_ready = 1;
}

static void OPS_FeedByte(uint8_t ch)
{
  switch (s_count)
  {
  case 0:
    if (ch == 0x0D)
      s_count = 1;
    else
      s_count = 0;
    break;
  case 1:
    if (ch == 0x0A)
    {
      s_i = 0;
      s_count = 2;
    }
    else if (ch == 0x0D)
      s_count = 1;
    else
      s_count = 0;
    break;
  case 2:
    s_payload.data[s_i++] = ch;
    if (s_i >= 24)
    {
      s_i = 0;
      s_count = 3;
    }
    break;
  case 3:
    if (ch == 0x0A)
      s_count = 4;
    else
      s_count = 0;
    break;
  case 4:
    if (ch == 0x0D)
      OPS_ApplyPayload();
    s_count = 0;
    break;
  default:
    s_count = 0;
    break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    OPS_FeedByte(s_rx_byte);
    (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
  }
  else if (huart->Instance == UART4)
  {
    MAX30102_OnUartRxCplt();
  }
  else if (huart->Instance == UART8)
  {
    GY614_OnUartRxCplt();
  }
}

void OPS_Init(void)
{
  memset(&OPS, 0, sizeof(OPS));
  s_count = 0;
  s_i = 0;
  s_frame_ready = 0;
  s_last_frame_ms = 0;

  /* USART3 已由 main 中 MX_USART3_UART_Init() 初始化 */
  (void)HAL_UART_AbortReceive_IT(&huart3);
  (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);

  /* Wait first frame (max ~2s) then cali like RC_Old */
  {
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 2000U)
    {
      if (s_frame_ready)
        break;
      osDelay(10);
    }
  }

  OPS_Cali();
  osDelay(50);
  s_frame_ready = 0;
}

void OPS_Task(void)
{
  OPS_Init();

  for (;;)
  {
    /* 位姿由 USART3 RX 中断解帧写入 OPS；本任务负责周期读取/维护 */
    if (OPS_FrameReady())
    {
      /* 此处可读 pos_x/pos_y/zangle，供本任务逻辑或通知其它任务 */
      (void)OPS_GetX();
      (void)OPS_GetY();
      (void)OPS_GetYaw();
      OPS_ClearFrameReady();
    }
    (void)OPS_IsOnline();
    osDelay(10);
  }
}

uint8_t OPS_IsOnline(void)
{
  if (s_last_frame_ms == 0)
    return 0;
  return (HAL_GetTick() - s_last_frame_ms) < 200U ? 1U : 0U;
}

uint8_t OPS_FrameReady(void)
{
  return s_frame_ready;
}

void OPS_ClearFrameReady(void)
{
  s_frame_ready = 0;
}

float OPS_GetX(void) { return pos_x; }
float OPS_GetY(void) { return pos_y; }
float OPS_GetYaw(void) { return zangle; }
float OPS_GetWz(void) { return w_z; }

void OPS_Cali(void)
{
  OPS.ActVal[1] = 0;
  OPS.ActVal[2] = 0;
  OPS.ActVal[3] = 0;
  OPS.ActVal[4] = 0;
  OPS.ActVal[5] = 0;
  OPS.ActVal[6] = 0;
  OPS_SendString("ACT0");
}

static void OPS_UpdateAxis(const char *cmd4, float v)
{
  uint8_t pkt[8];
  union {
    float f;
    uint8_t b[4];
  } u;
  int i;

  pkt[0] = (uint8_t)cmd4[0];
  pkt[1] = (uint8_t)cmd4[1];
  pkt[2] = (uint8_t)cmd4[2];
  pkt[3] = (uint8_t)cmd4[3];
  u.f = v;
  for (i = 0; i < 4; i++)
    pkt[4 + i] = u.b[i];
  OPS_SendBytes(pkt, 8);
}

void OPS_UpdateX(float posx) { OPS_UpdateAxis("ACTX", posx); }
void OPS_UpdateY(float posy) { OPS_UpdateAxis("ACTY", posy); }
void OPS_UpdateZ(float posz) { OPS_UpdateAxis("ACTJ", posz); }
