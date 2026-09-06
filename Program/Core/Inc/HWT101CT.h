/**
 * @file HWT101CT.h
 * @brief 维特 HWT101CT-TTL @ USART1 PA9=TX PA10=RX  115200  DMA+IDLE
 *
 * ����:
 *   main ��: MX_USART1_UART_Init(); HWT101_Init();
 *   ��������: HWT101_GetYaw() / HWT101_GetWz() / HWT101_IsOnline()
 *   ��������: HWT101_CaliYaw();
 */
#ifndef HWT101CT_H
#define HWT101CT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/** Keil Debug Watch: HWT101 航向角 (°) / 在线标志 */
extern volatile float   hwt_zangle;
extern volatile float   pos_z;
extern volatile uint8_t hwt_online;

void HWT101_Init(void);

uint8_t  HWT101_IsOnline(void);
float    HWT101_GetYaw(void);
float    HWT101_GetWz(void);
uint16_t HWT101_GetVersion(void);

/** Z ��ǶȻ��㣨�Ƚ�����д CALIYAW�� */
void HWT101_CaliYaw(void);

#ifdef __cplusplus
}
#endif

#endif /* HWT101CT_H */
