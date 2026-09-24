#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

void PID_SetX(float Kp, float Ki, float Kd, float out_min, float out_max,
              float i_max);
void PID_SetY(float Kp, float Ki, float Kd, float out_min, float out_max,
              float i_max);
void PID_SetZ(float Kp, float Ki, float Kd, float out_min, float out_max,
              float i_max);

float PID_UpdateX(float target, float measurement);
float PID_UpdateY(float target, float measurement);
float PID_UpdateZ(float target, float measurement);

void PID_Reset(void);
void PID_ResetXY(void);

#ifdef __cplusplus
}
#endif

#endif
