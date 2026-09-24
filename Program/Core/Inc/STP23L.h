#ifndef STP23L_H
#define STP23L_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void STP23L_Init(void);
void STP23L_OnUartRxCplt(UART_HandleTypeDef *huart);
void STP23L_OnUartError(UART_HandleTypeDef *huart);

uint16_t STP32_getA(void);
uint16_t STP32_getB(void);
uint16_t STP32_getC(void);

uint8_t STP23L_IsOnlineA(void);
uint8_t STP23L_IsOnlineB(void);
uint8_t STP23L_IsOnlineC(void);

uint8_t STP23L_GetSampleA(uint16_t *distance_mm, uint32_t *frame_sequence);
uint8_t STP23L_GetSampleB(uint16_t *distance_mm, uint32_t *frame_sequence);
uint8_t STP23L_GetSampleC(uint16_t *distance_mm, uint32_t *frame_sequence);

#ifdef __cplusplus
}
#endif

#endif
