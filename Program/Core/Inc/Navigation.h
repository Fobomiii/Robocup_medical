/**
 * @file Navigation.h
 * @brief 机器人导航：FSM + 侧向避障(OA)
 *
 * 坐标：全局 +Y 前、+X 右、航向顺时针(°)；与 OPS/HWT 一致。
 * RDK 障碍为车体相对：BX(右+) BY(前+)，经 OA_SetObstacleBody 写入。
 *
 * 周期：Nav_Update() = FSM_Update → OA_Update → ChassisCtrl_Update
 */
#ifndef NAVIGATION_H
#define NAVIGATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

void MapPos_Init(void);
void FSM_Update(void);

/**
 * @brief 写入车体障碍点。
 * @param bx 车体右向相对距离（与 OPS 同单位）
 * @param by 车体前向相对距离
 * @param valid 0=本帧无障碍；非0=有效
 */
void OA_SetObstacleBody(float bx, float by, uint8_t valid);

/** 1=允许避障侧移；0=清偏置并忽略障碍（扫码/放药等） */
void OA_Enable(uint8_t enable);
void OA_Reset(void);
void OA_Update(void);

void Nav_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_H */
