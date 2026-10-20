/**
 * @file CN_TTS.c
 * @brief CN-TTS GBK text driver on the former GM65 UART7 connection.
 */
#include "CN_TTS.h"

#include "main.h"

#include <stddef.h>

extern UART_HandleTypeDef huart7;

#define CN_TTS_TX_TIMEOUT_MS 100U

static uint8_t s_rx_byte;
static volatile uint8_t s_busy;

/* GBK byte streams for the bed 1 and bed 3 medicine announcements. */
static const uint8_t s_bed1_announcement[] = {
    0xD2U, 0xBBU, 0xB4U, 0xB2U, 0xB2U, 0xA1U, 0xC8U,
    0xCBU, 0xC7U, 0xEBU, 0xC8U, 0xA1U, 0xD2U, 0xA9U};
static const uint8_t s_bed3_announcement[] = {
    0xC8U, 0xFDU, 0xB4U, 0xB2U, 0xB2U, 0xA1U, 0xC8U,
    0xCBU, 0xC7U, 0xEBU, 0xC8U, 0xA1U, 0xD2U, 0xA9U};

static HAL_StatusTypeDef cn_tts_send(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U) ||
      (length > CN_TTS_MAX_TEXT_BYTES))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(&huart7, (uint8_t *)data, length,
                           CN_TTS_TX_TIMEOUT_MS);
}

static HAL_StatusTypeDef cn_tts_send_setting(uint8_t command, uint8_t value)
{
  uint8_t frame[4] = {'<', command, '>', value};

  return cn_tts_send(frame, sizeof(frame));
}

void CN_TTS_Init(void)
{
  s_busy = 0U;
  s_rx_byte = 0U;
  (void)HAL_UART_Receive_IT(&huart7, &s_rx_byte, 1U);
}

HAL_StatusTypeDef CN_TTS_SpeakGBK(const uint8_t *text, uint16_t length)
{
  HAL_StatusTypeDef status;

  s_busy = 1U;
  status = cn_tts_send(text, length);
  if (status != HAL_OK)
  {
    s_busy = 0U;
  }
  return status;
}

HAL_StatusTypeDef CN_TTS_SetVolume(uint8_t volume)
{
  if ((volume < 1U) || (volume > 4U))
  {
    return HAL_ERROR;
  }
  return cn_tts_send_setting((uint8_t)'V', (uint8_t)('0' + volume));
}

HAL_StatusTypeDef CN_TTS_SetSpeed(uint8_t speed)
{
  if ((speed < 1U) || (speed > 3U))
  {
    return HAL_ERROR;
  }
  return cn_tts_send_setting((uint8_t)'S', (uint8_t)('0' + speed));
}

HAL_StatusTypeDef CN_TTS_PlaySound(uint8_t sound)
{
  if (sound > 7U)
  {
    return HAL_ERROR;
  }
  return cn_tts_send_setting((uint8_t)'Z', (uint8_t)('0' + sound));
}

HAL_StatusTypeDef CN_TTS_AnnounceBed1(void)
{
  return CN_TTS_SpeakGBK(s_bed1_announcement,
                         (uint16_t)sizeof(s_bed1_announcement));
}

HAL_StatusTypeDef CN_TTS_AnnounceBed3(void)
{
  return CN_TTS_SpeakGBK(s_bed3_announcement,
                         (uint16_t)sizeof(s_bed3_announcement));
}

uint8_t CN_TTS_IsBusy(void)
{
  return s_busy;
}

void CN_TTS_OnUartRxCplt(void)
{
  if (s_rx_byte == (uint8_t)'A')
  {
    s_busy = 1U;
  }
  else if (s_rx_byte == (uint8_t)'O')
  {
    s_busy = 0U;
  }

  (void)HAL_UART_Receive_IT(&huart7, &s_rx_byte, 1U);
}

void CN_TTS_OnUartError(void)
{
  s_busy = 0U;
  (void)HAL_UART_Receive_IT(&huart7, &s_rx_byte, 1U);
}
