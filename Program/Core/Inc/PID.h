/**
 * @file PID.h
 * @brief 三轴位姿 PID（X/Y 位置 + Z 航向），extern "C" 接口，C/C++ 均可调用。
 *
 * Update 返回控制量（如 vx/vy/角速度），不是滤波后的 measurement。
 * Z 轴 target/measurement 单位：度（与 HWT101 hwt_zangle 一致）。
 */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

void PID_SetX(float Kp, float Ki, float Kd, float out_min, float out_max, float i_max);
void PID_SetY(float Kp, float Ki, float Kd, float out_min, float out_max, float i_max);
void PID_SetZ(float Kp, float Ki, float Kd, float out_min, float out_max, float i_max);

float PID_UpdateX(float target, float measurement);
float PID_UpdateY(float target, float measurement);
float PID_UpdateZ(float target, float measurement);

void PID_Reset(void);
void PID_ResetXY(void);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
