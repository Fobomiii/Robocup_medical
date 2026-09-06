/**
 * @file OPS.h
 * @brief OPS全场定位串口驱动 (协议对齐 RC_Old Master OPS)
 *
 * 硬件: USART3  PC10=TX  PC11=RX  115200 8N1
 *
 * 调用:
 *   StartOPSUartTask 内: OPS_Task() —— 初始化 + 周期维护
 *   其它任务读位姿: pos_x / pos_y / zangle 或 OPS_GetX/Y/Yaw()
 *   清零: OPS_Cali()
 *   写坐标: OPS_UpdateX/Y/Z()
 */
#ifndef __OPS_H
#define __OPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef union {
  uint8_t data[32];
  float ActVal[8];
} Union_OPS;

extern Union_OPS OPS;
extern UART_HandleTypeDef huart3;

/** 与 RC_Old Location.h 一致（注意负号） */
#define pos_x   (-OPS.ActVal[4])
#define pos_y   (-OPS.ActVal[5])
#define zangle  (-OPS.ActVal[1])
#define xangle  (-OPS.ActVal[2])
#define yangle  (-OPS.ActVal[3])
#define w_z     (-OPS.ActVal[6])

/** 初始化 UART 中断收帧 + ACT0 清零（在 OPSUartTask 里由 OPS_Task 调用） */
void OPS_Init(void);

/** OPSUartTask 入口：Init 后循环读取/维护（永不返回） */
void OPS_Task(void);

/** 是否在线（近期收到完整帧） */
uint8_t OPS_IsOnline(void);

/** 自上次查询后是否有新帧 */
uint8_t OPS_FrameReady(void);
void OPS_ClearFrameReady(void);

float OPS_GetX(void);
float OPS_GetY(void);
float OPS_GetYaw(void);
float OPS_GetWz(void);

void OPS_Cali(void);
void OPS_UpdateX(float posx);
void OPS_UpdateY(float posy);
void OPS_UpdateZ(float posz);

#ifdef __cplusplus
}
#endif

#endif /* __OPS_H */
