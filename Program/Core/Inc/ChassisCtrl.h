/**
 * @file ChassisCtrl.h
 * @brief 底盘位姿闭环：OPS 位置 + HWT101 航向 → PID → DJI_Chassis_SetCommand。
 *
 * 与 DJIMotorCtrlSTM32 内 C++ class CHASSIS（电机速度环）分层：
 *   本模块 = 外环位姿；DJI 库 = 内环 3508 速度。
 *
 * 坐标：目标/反馈 X/Y 为 OPS 场坐标；航向为 HWT101 角度 (°)。
 * +Y 前进，+X 右移，+W 顺时针。
 * 避障：OA 以世界系偏置加在 target 上，Update 跟踪 track = target + oa。
 */
#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/********设置目标点**********/
void ChassisCtrl_MoveTarget(float x_tgt, float y_tgt, float z_tgt,
                            float pos_x, float pos_y, float pos_z);
/**********使能设置***********/
void ChassisCtrl_Enable(bool enable);
/*************后台运行************/
bool ChassisCtrl_Update(float pos_x, float pos_y, float pos_z);
/********获取到达标志 */
bool ChassisCtrl_ReachFlag(void);

/** 世界系避障偏置（加在 target 上）；Clear 等价于 Set(0,0) */
void ChassisCtrl_SetOAOffset(float dx, float dy);
void ChassisCtrl_ClearOA(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CTRL_H */
