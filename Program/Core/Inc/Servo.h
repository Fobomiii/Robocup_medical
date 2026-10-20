/**
 * @file Servo.h
 * @brief SG90 PWM servo on TIM3 CH1 (PA6) / CH2 (PA7), 50Hz
 */
#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

void SERVO_Init(void);
void SERVO1_ANGLE(float angle_deg);
void SERVO2_ANGLE(float angle_deg);
void SERVO1_OPEN(void);
void SERVO1_CLOSE(void);
void SERVO2_OPEN(void);
void SERVO2_CLOSE(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
