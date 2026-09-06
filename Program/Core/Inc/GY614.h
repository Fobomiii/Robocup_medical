/**
 * @file GY614.h
 * @brief GY-614 ������� @ UART8 PE1=TX PE0=RX  9600  �ֽ��ж���֡
 *
 * ��ͷ���� BO = (H*256+L)/100 ��  ��֡�� bo ��/���ֽڣ�
 *
 * 调用:
 *   main: MX_UART8_Init(); GY614_Init();
 *   其它处: float t = getTemp();  float tf = getTempLPF();
 */
#ifndef GY614_H
#define GY614_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

extern UART_HandleTypeDef huart8;

void GY614_Init(void);

/** 原始体表温度 ℃（最新一帧 BO/100）；无有效帧时为 0 */
float getTemp(void);

/** 一阶低通后的体表温度 ℃（在收帧时更新） */
float getTempLPF(void);

/** 内部：UART8 RX 中断完成，由 HAL_UART_RxCpltCallback 分发 */
void GY614_OnUartRxCplt(void);

#ifdef __cplusplus
}
#endif

#endif /* GY614_H */
