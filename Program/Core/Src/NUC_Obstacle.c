/**
 * @file NUC_Obstacle.c
 * @brief NUC obstacle detection UART receiver
 *
 * Frame: AA BB | XH XL | YH YL | CHK | 55
 * Coordinate: X forward (mm), Y lateral (mm, positive=left)
 */
#include "NUC_Obstacle.h"
#include <string.h>

#define NUC_FRAME_HEADER1  0xAA
#define NUC_FRAME_HEADER2  0xBB
#define NUC_FRAME_TAIL     0x55
#define NUC_FRAME_LEN      8U
#define NUC_TIMEOUT_MS     200U
#define NUC_FRAME_GAP_MS   50U
#define NUC_HANDSHAKE_GAP_MS 100U
#define NUC_HANDSHAKE_LEN    4U

static uint8_t s_it_byte;
static uint8_t s_buf[NUC_FRAME_LEN];
static uint8_t s_idx;
static uint8_t s_pong_idx;
static volatile int16_t s_obstacle_x;
static volatile int16_t s_obstacle_y;
static volatile uint32_t s_last_valid_ms;
static const uint8_t s_ping[NUC_HANDSHAKE_LEN] = {0xA5U, 0x5AU, 0x01U, 0x00U};
static const uint8_t s_pong[NUC_HANDSHAKE_LEN] = {0xA5U, 0x5AU, 0x01U, 0x01U};

/* Debug: expose frame buffer for inspection */
volatile uint8_t g_nuc_debug_buf[8];
volatile uint8_t g_nuc_parse_fail_reason = 0; // 1=header seen, 2=tail, 3=checksum, 0xFF=success
volatile uint32_t g_nuc_rx_count = 0; // Total bytes received
volatile uint8_t g_nuc_last_byte = 0; // Last received byte
volatile uint32_t g_nuc_uart_error_count = 0;
volatile uint32_t g_nuc_last_uart_error = 0;
volatile uint8_t g_nuc_online = 0;

/* Debug: circular buffer for last 16 bytes */
#define RX_HISTORY_SIZE 16
volatile uint8_t g_nuc_rx_history[RX_HISTORY_SIZE];
volatile uint8_t g_nuc_history_idx = 0;

static uint32_t s_last_rx_ms;

static void nuc_feed_handshake_byte(uint8_t b)
{
  if (b == s_pong[s_pong_idx])
  {
    s_pong_idx++;
    if (s_pong_idx >= NUC_HANDSHAKE_LEN)
    {
      g_nuc_online = 1U;
      s_pong_idx = 0U;
    }
  }
  else if (b == s_pong[0])
  {
    s_pong_idx = 1U;
  }
  else
  {
    s_pong_idx = 0U;
  }
}

static void nuc_parse_frame(void)
{
  uint8_t chk_calc;
  uint8_t chk_recv;
  int16_t x, y;

  /* Copy frame for debugging */
  for (uint8_t i = 0; i < NUC_FRAME_LEN; i++)
  {
    g_nuc_debug_buf[i] = s_buf[i];
  }

  /* Verify header and tail */
  if (s_buf[0] != NUC_FRAME_HEADER1 || s_buf[1] != NUC_FRAME_HEADER2)
  {
    g_nuc_parse_fail_reason = 1;
    return;
  }
  if (s_buf[7] != NUC_FRAME_TAIL)
  {
    g_nuc_parse_fail_reason = 2;
    return;
  }

  /* Verify checksum */
  chk_calc = (uint8_t)(s_buf[2] + s_buf[3] + s_buf[4] + s_buf[5]);
  chk_recv = s_buf[6];
  if (chk_calc != chk_recv)
  {
    g_nuc_parse_fail_reason = 3;
    return;
  }

  /* Parse X and Y (big-endian int16) */
  x = (int16_t)(((uint16_t)s_buf[2] << 8) | s_buf[3]);
  y = (int16_t)(((uint16_t)s_buf[4] << 8) | s_buf[5]);

  s_obstacle_x = x;
  s_obstacle_y = y;
  s_last_valid_ms = HAL_GetTick();
  g_nuc_parse_fail_reason = 0xFF; // Success
}

static void nuc_feed_byte(uint8_t b)
{
  uint32_t now = HAL_GetTick();

  /* A partial frame cannot span a long bus gap; resync before consuming b. */
  if (s_idx != 0U && (now - s_last_rx_ms) > NUC_FRAME_GAP_MS)
  {
    s_idx = 0;
  }
  s_last_rx_ms = now;

  /* State 0: wait for header1 */
  if (s_idx == 0)
  {
    if (b == NUC_FRAME_HEADER1)
    {
      g_nuc_parse_fail_reason = 1;
      s_buf[s_idx++] = b;
    }
    return;
  }

  /* State 1: wait for header2 */
  if (s_idx == 1)
  {
    if (b == NUC_FRAME_HEADER2)
    {
      s_buf[s_idx++] = b;
    }
    else if (b == NUC_FRAME_HEADER1)
    {
      g_nuc_parse_fail_reason = 1;
      s_buf[0] = b;
      s_idx = 1;
    }
    else
    {
      s_idx = 0;
    }
    return;
  }

  /* State 2-7: collect remaining bytes */
  s_buf[s_idx++] = b;
  if (s_idx >= NUC_FRAME_LEN)
  {
    nuc_parse_frame();
    s_idx = 0;
  }
}

void NUC_Obstacle_Init(void)
{
  s_idx = 0;
  s_pong_idx = 0;
  s_last_rx_ms = 0;
  s_obstacle_x = 0;
  s_obstacle_y = 0;
  s_last_valid_ms = 0;
  g_nuc_parse_fail_reason = 0;
  g_nuc_rx_count = 0;
  g_nuc_last_byte = 0;
  g_nuc_uart_error_count = 0;
  g_nuc_last_uart_error = 0;
  g_nuc_online = 0;
  memset((void *)g_nuc_debug_buf, 0, sizeof(g_nuc_debug_buf));
  memset((void *)g_nuc_rx_history, 0, sizeof(g_nuc_rx_history));
  g_nuc_history_idx = 0;
  memset(s_buf, 0, sizeof(s_buf));
  (void)HAL_UART_AbortReceive_IT(&huart8);
  (void)HAL_UART_Receive_IT(&huart8, &s_it_byte, 1);
}

HAL_StatusTypeDef NUC_Obstacle_SendPing(void)
{
  return HAL_UART_Transmit(&huart8, s_ping, NUC_HANDSHAKE_LEN, 10U);
}

uint8_t NUC_IsOnline(void)
{
  return g_nuc_online;
}

uint8_t NUC_Obstacle_WaitForOnline(uint32_t timeout_ms)
{
  uint32_t start_ms = HAL_GetTick();
  uint32_t last_ping_ms = start_ms - NUC_HANDSHAKE_GAP_MS;

  g_nuc_online = 0U;
  s_pong_idx = 0U;

  while ((HAL_GetTick() - start_ms) < timeout_ms)
  {
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_ping_ms) >= NUC_HANDSHAKE_GAP_MS)
    {
      (void)NUC_Obstacle_SendPing();
      last_ping_ms = now_ms;
    }

    if (NUC_IsOnline() != 0U)
    {
      return 1U;
    }

    HAL_Delay(1U);
  }

  return NUC_IsOnline();
}

void NUC_Obstacle_OnUartError(void)
{
  g_nuc_uart_error_count++;
  g_nuc_last_uart_error = huart8.ErrorCode;

  /* ORE/RTO abort interrupt reception; restart it after HAL marks the
     receive state ready. Frame/noise errors are recoverable in HAL and keep
     the current one-byte reception active. */
  if ((huart8.ErrorCode & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_RTO)) != 0U)
  {
    (void)HAL_UART_Receive_IT(&huart8, &s_it_byte, 1);
  }
}

void NUC_Obstacle_OnUartRxCplt(void)
{
  g_nuc_rx_count++;
  g_nuc_last_byte = s_it_byte;

  // Store in circular history buffer
  g_nuc_rx_history[g_nuc_history_idx] = s_it_byte;
  g_nuc_history_idx = (g_nuc_history_idx + 1) % RX_HISTORY_SIZE;

  nuc_feed_handshake_byte(s_it_byte);
  nuc_feed_byte(s_it_byte);
  (void)HAL_UART_Receive_IT(&huart8, &s_it_byte, 1);
}

int16_t NUC_GetObstacleX(void)
{
  if (!NUC_IsObstacleValid())
  {
    return 0;
  }
  return s_obstacle_x;
}

int16_t NUC_GetObstacleY(void)
{
  if (!NUC_IsObstacleValid())
  {
    return 0;
  }
  return s_obstacle_y;
}

uint8_t NUC_IsObstacleValid(void)
{
  if (s_last_valid_ms == 0)
  {
    return 0;
  }
  return (HAL_GetTick() - s_last_valid_ms) < NUC_TIMEOUT_MS ? 1U : 0U;
}
