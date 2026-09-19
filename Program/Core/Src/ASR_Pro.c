/**
 * @file ASR_Pro.c
 * @brief STM32 to ASR Pro announcement commands over UART4.
 */
#include "ASR_Pro.h"

#include "main.h"

extern UART_HandleTypeDef huart4;

#define ASR_PRO_FRAME_HEADER_0 0xAAU
#define ASR_PRO_FRAME_HEADER_1 0x55U
#define ASR_PRO_COMMAND_BED1   0x01U
#define ASR_PRO_COMMAND_BED3   0x03U
#define ASR_PRO_TX_TIMEOUT_MS  20U

static HAL_StatusTypeDef asr_pro_send_command(uint8_t command)
{
  uint8_t frame[4];

  frame[0] = ASR_PRO_FRAME_HEADER_0;
  frame[1] = ASR_PRO_FRAME_HEADER_1;
  frame[2] = command;
  frame[3] = (uint8_t)(frame[0] ^ frame[1] ^ frame[2]);

  return HAL_UART_Transmit(&huart4, frame, sizeof(frame), ASR_PRO_TX_TIMEOUT_MS);
}

HAL_StatusTypeDef ASR_Pro_AnnounceBed1(void)
{
  return asr_pro_send_command(ASR_PRO_COMMAND_BED1);
}

HAL_StatusTypeDef ASR_Pro_AnnounceBed3(void)
{
  return asr_pro_send_command(ASR_PRO_COMMAND_BED3);
}
