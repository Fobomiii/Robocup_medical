/**
 * @file MAX30102.h
 * @brief MAX30102 ģ�� AT ���� @ UART4 PD1=TX PD0=RX  57600
 *
 * �˲����� A��|��ֵ-�ϴ��ѽ���| > 3 ������NULL ��մ��ڡ�
 *
 * ����:
 *   main: MX_UART4_Init(); MAX30102_Init();
 *   ���� 1Hz: MAX30102_Update();
 *   ���⴦: getBPM() / getBPMRaw();
 */
#ifndef MAX30102_H
#define MAX30102_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

extern UART_HandleTypeDef huart4;

void MAX30102_Init(void);

/** �� AT+HEART�����������������˲�������������ÿ 1s ��һ�Σ� */
void MAX30102_Update(void);

/** ƽ��������� bpm����Ч/�崰��Ϊ 0 */
int getBPM(void);

/** ���һ�ν�������ԭʼ bpm�������޳������䣩��NULL ʱΪ 0 */
int getBPMRaw(void);

/** �ڲ���UART4 RX �ж���ɻص����� OPS HAL_UART_RxCpltCallback �ַ��� */
void MAX30102_OnUartRxCplt(void);

#ifdef __cplusplus
}
#endif

#endif /* MAX30102_H */
