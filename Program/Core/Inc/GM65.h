/**
 * @file GM65.h
 * @brief GM65 ��ά��ģ�� @ UART7 PE8=TX PE7=RX  9600  DMA+IDLE
 *
 * ɨ��ɹ����ϱ��ַ������޹̶����ڷ��͡�
 *
 * ����:
 *   main: MX_UART7_Init(); GM65_Init();
 *   ����: if (GM65_FrameReady()) { const char *s = GM65_GetLastCode(); GM65_ClearFrameReady(); }
 */
#ifndef GM65_H
#define GM65_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define GM65_CODE_MAX  128U

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;

void GM65_Init(void);

/** �ڲ����� HAL_UARTEx_RxEventCallback �ַ����� */
void GM65_OnRxEvent(uint16_t Size);

uint8_t     GM65_FrameReady(void);
void        GM65_ClearFrameReady(void);
const char *GM65_GetLastCode(void);
uint16_t    GM65_GetLastLen(void);

#ifdef __cplusplus
}
#endif

#endif /* GM65_H */
