/**
 * @file DJIMotorCtrlSTM32.h
 * @brief DJI C620/C610 motor control for STM32 (ESP-style cascade PID).
 *
 * CHASSIS: M3508+P19, CAN ID 1~4 = Front/Left/Rear/Right
 *   SetCommand / Update(Vx,Vy,W): +Y forward, +X right, +W clockwise.
 * M2006Motor: M2006+P36, default CAN ID 5
 *   ctrlAngle(deg) / ctrlAngle(deg, time_s): time>0 uses trapezoid accel-cruise-decel.
 *   update() runs pos/speed loop (call from armTask).
 */
#ifndef DJI_MOTOR_CTRL_STM32_H
#define DJI_MOTOR_CTRL_STM32_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus

class CHASSIS {
public:
  explicit CHASSIS(FDCAN_HandleTypeDef* hfdcan);

  /** Init bus + set control frequency. Default 1000 Hz. */
  void begin(uint16_t frq_hz = 1000);

  /**
   * One control step: kinematics + ESP speed loop + send 0x200.
   * Call at begin() frequency from chassisTask.
   */
  void Update(float Vx, float Vy, float W);

  uint16_t freq() const { return frq_; }

private:
  FDCAN_HandleTypeDef* can_;
  uint16_t frq_;
  bool started_;
};

class M2006Motor {
public:
  M2006Motor(FDCAN_HandleTypeDef* hfdcan, uint8_t id = 5, float gear_ratio = 36.f);

  /** Init bus + set control frequency. Default 1000 Hz. */
  void begin(uint16_t frq_hz = 1000);

  /** Jump target immediately (may be abrupt). */
  void ctrlAngle(float deg);

  /**
   * Move to angle_deg within time_s using trapezoid velocity
   * (accel → cruise → decel). time_s<=0 same as ctrlAngle(deg).
   */
  void ctrlAngle(float deg, float time_s);

  /**
   * One control step: trajectory + position + speed PID + send 0x1FF.
   * Call at begin() frequency from armTask.
   */
  void update();

  uint16_t freq() const { return frq_; }

  /** Current output-shaft angle estimate (deg). */
  float getAngleDeg();

private:
  void planTrapezoid(float start_deg, float final_deg, float time_s);
  float trajEval(float t) const;
  float encPerOutDeg() const;
  void sendCurrent(int16_t current);

  FDCAN_HandleTypeDef* can_;
  uint8_t id_;
  float gear_;
  uint16_t frq_;
  bool started_;

  volatile float target_deg_; /* final goal */
  float cmd_deg_;             /* instantaneous setpoint for PID */

  /* Trapezoid trajectory (output-shaft degrees, seconds) */
  bool traj_active_;
  uint32_t traj_t0_us_;
  float traj_T_;
  float traj_start_;
  float traj_final_;
  float traj_Ta_;
  float traj_Tc_;
  float traj_a_; /* |accel| deg/s^2 */
  float traj_v_; /* |peak speed| deg/s */
  float traj_dir_; /* +1 / -1 */
};

extern CHASSIS chassis;
extern M2006Motor arm;

#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif

void DJI_Motor_ChassisTask(void);
void DJI_Motor_ArmStart(void);

void DJI_Chassis_SetCommand(float vx, float vy, float w);

/** Immediate angle (deg). */
void DJI_Arm_CtrlAngle(float deg);

/** Trapezoid move to deg in time_s seconds. */
void DJI_Arm_CtrlAngleTimed(float deg, float time_s);

#ifdef __cplusplus
}
#endif

#endif /* DJI_MOTOR_CTRL_STM32_H */
