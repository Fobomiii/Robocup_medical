/**
 * @file Board.c
 * @brief Board-level I/O helpers.
 */
#include "Board.h"

void BUZZ_On(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

void BUZZ_Off(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void BUZZ_Beep(uint32_t duration_ms)
{
  BUZZ_On();
  HAL_Delay(duration_ms);
  BUZZ_Off();
}
