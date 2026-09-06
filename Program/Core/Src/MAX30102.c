/**
 * @file MAX30102.c
 * @brief MAX30102 AT+HEART + �������ڣ����� A��|��|>3 ������
 */
#include "MAX30102.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_RX_RING      256U
#define MAX_LINE_SIZE    128U
#define MAX_WIN_SIZE     8U
#define MAX_DELTA_REJECT 20   /* |��| > 3 ���� */
#define MAX_TIMEOUT_MS   800U

static uint8_t s_it_byte;
static volatile uint8_t s_ring[MAX_RX_RING];
static volatile uint16_t s_ring_h;
static volatile uint16_t s_ring_t;

static char s_line_acc[MAX_LINE_SIZE];
static uint16_t s_line_len;

static int s_win[MAX_WIN_SIZE];
static uint8_t s_win_n;
static int s_last_accepted;
static int s_bpm_smooth;
static int s_bpm_raw;

static void ring_reset(void)
{
  s_ring_h = 0;
  s_ring_t = 0;
}

static void ring_push(uint8_t b)
{
  uint16_t next = (uint16_t)((s_ring_h + 1U) % MAX_RX_RING);
  if (next == s_ring_t)
  {
    /* ������� */
    s_ring_t = (uint16_t)((s_ring_t + 1U) % MAX_RX_RING);
  }
  s_ring[s_ring_h] = b;
  s_ring_h = next;
}

static uint8_t ring_pop(uint8_t *out)
{
  if (s_ring_t == s_ring_h)
  {
    return 0U;
  }
  *out = s_ring[s_ring_t];
  s_ring_t = (uint16_t)((s_ring_t + 1U) % MAX_RX_RING);
  return 1U;
}

static void win_clear(void)
{
  s_win_n = 0;
  s_last_accepted = 0;
  s_bpm_smooth = 0;
}

static void win_recalc(void)
{
  uint8_t i;
  int sum = 0;
  if (s_win_n == 0U)
  {
    s_bpm_smooth = 0;
    return;
  }
  for (i = 0; i < s_win_n; i++)
  {
    sum += s_win[i];
  }
  s_bpm_smooth = (sum + (int)s_win_n / 2) / (int)s_win_n;
}

static void win_push(int bpm)
{
  if (s_win_n < MAX_WIN_SIZE)
  {
    s_win[s_win_n++] = bpm;
  }
  else
  {
    uint8_t i;
    for (i = 1; i < MAX_WIN_SIZE; i++)
    {
      s_win[i - 1U] = s_win[i];
    }
    s_win[MAX_WIN_SIZE - 1U] = bpm;
  }
  s_last_accepted = bpm;
  win_recalc();
}

static void filter_on_sample(int bpm, uint8_t is_null)
{
  if (is_null)
  {
    s_bpm_raw = 0;
    win_clear();
    return;
  }

  s_bpm_raw = bpm;

  if (s_win_n == 0U)
  {
    win_push(bpm);
    return;
  }

  {
    int d = bpm - s_last_accepted;
    if (d < 0)
    {
      d = -d;
    }
    if (d > MAX_DELTA_REJECT)
    {
      /* ���� A�����������ڱ����������㣨������䣩 */
      win_recalc();
      return;
    }
  }
  win_push(bpm);
}

static void line_reset(void)
{
  s_line_len = 0;
  s_line_acc[0] = '\0';
}

static int wait_line(char *out, uint16_t out_size, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint8_t ch;

  if (out == NULL || out_size < 2U)
  {
    return -1;
  }
  out[0] = '\0';

  while ((HAL_GetTick() - start) < timeout_ms)
  {
    if (ring_pop(&ch) == 0U)
    {
      continue;
    }
    if (((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n')) && (s_line_len == 0U))
    {
      continue;
    }
    if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n'))
    {
      s_line_acc[s_line_len] = '\0';
      strncpy(out, s_line_acc, out_size - 1U);
      out[out_size - 1U] = '\0';
      line_reset();
      return 0;
    }
    if (s_line_len < (MAX_LINE_SIZE - 1U))
    {
      s_line_acc[s_line_len++] = (char)ch;
    }
    else
    {
      line_reset();
    }
  }
  return -1;
}

static int transact_heart(char *resp, uint16_t resp_size)
{
  char line[MAX_LINE_SIZE];
  uint32_t start;
  uint16_t used = 0;
  const char *cmd = "AT+HEART\r\n";

  ring_reset();
  line_reset();
  resp[0] = '\0';

  (void)HAL_UART_Transmit(&huart4, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);

  start = HAL_GetTick();
  while ((HAL_GetTick() - start) < MAX_TIMEOUT_MS)
  {
    if (wait_line(line, sizeof(line), 50U) != 0)
    {
      continue;
    }

    if (used > 0U && used < (resp_size - 1U))
    {
      resp[used++] = '\n';
      resp[used] = '\0';
    }
    {
      uint16_t n = (uint16_t)strlen(line);
      if ((used + n) >= (resp_size - 1U))
      {
        n = (uint16_t)(resp_size - 1U - used);
      }
      memcpy(&resp[used], line, n);
      used = (uint16_t)(used + n);
      resp[used] = '\0';
    }

    if (strstr(line, "OK") != NULL)
    {
      return 0;
    }
  }
  return (used > 0U) ? 0 : -1;
}

static int parse_heart(const char *resp, int *bpm_out, uint8_t *is_null)
{
  const char *p;
  *is_null = 0;
  *bpm_out = 0;

  if (strstr(resp, "+HEART=NULL") != NULL)
  {
    *is_null = 1U;
    return 0;
  }
  p = strstr(resp, "+HEART=");
  if (p == NULL)
  {
    return -1;
  }
  *bpm_out = atoi(p + 7);
  if (*bpm_out <= 0)
  {
    *is_null = 1U;
  }
  return 0;
}

void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

void MAX30102_Init(void)
{
  ring_reset();
  line_reset();
  win_clear();
  s_bpm_raw = 0;
  (void)HAL_UART_AbortReceive_IT(&huart4);
  (void)HAL_UART_Receive_IT(&huart4, &s_it_byte, 1);
}

void MAX30102_OnUartRxCplt(void)
{
  ring_push(s_it_byte);
  (void)HAL_UART_Receive_IT(&huart4, &s_it_byte, 1);
}

void MAX30102_Update(void)
{
  char resp[MAX_LINE_SIZE];
  int bpm = 0;
  uint8_t is_null = 0;

  if (transact_heart(resp, sizeof(resp)) != 0)
  {
    return;
  }
  if (parse_heart(resp, &bpm, &is_null) != 0)
  {
    return;
  }
  filter_on_sample(bpm, is_null);
}

int getBPM(void)
{
  return s_bpm_smooth;
}

int getBPMRaw(void)
{
  return s_bpm_raw;
}
