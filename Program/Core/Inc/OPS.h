/**
 * @file OPS.h
 * @brief OPS全场定位串口驱动 (协议对齐 RC_Old Master OPS)
 *
 * 硬件: USART3  PC10=TX  PC11=RX  115200 8N1
 *
 * 调用:
 *   main（osKernelStart 前）: OPS_Init() —— 死等首帧 + ACT0 + 稳定性检查
 *   StartOPSUartTask: OPS_Task() —— 周期维护
 *   其它任务读位姿: 使用 OPS_GetX/Y/Yaw() 或 OPS_GetPose()
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

/* 由 USART3 接收中断更新；业务代码请通过 OPS_Get* 接口读取。 */
extern volatile Union_OPS OPS;
extern UART_HandleTypeDef huart3;

/**
 * 初始化：死等首帧 → ACT0 → 1s 航向稳定性检查（稳则短鸣，不稳则蜂鸣卡死）。
 * 在 main、创建 RTOS 任务之前调用（内部用 HAL_Delay）。
 */
void OPS_Init(void);

/** OPSUartTask 入口：周期读取/维护（永不返回；假定 OPS_Init 已完成） */
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

/**
 * 一次性读取同一帧中的 X/Y/OPS 航向。
 * 返回 1 表示 OPS 在线且输出有效，返回 0 表示尚未收到有效帧或已超时。
 */
uint8_t OPS_GetPose(float *pos_x, float *pos_y, float *yaw);

void OPS_Cali(void);
void OPS_UpdateX(float pos_x);
void OPS_UpdateY(float pos_y);
void OPS_UpdateZ(float pos_z);

#ifdef __cplusplus
}
#endif

#endif /* __OPS_H */
