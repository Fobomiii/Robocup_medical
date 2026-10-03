#include "ChassisCtrl.h"

#include "DJIMotorCtrlSTM32.h"
#include "PID.h"

#include <math.h>

#define CHASSIS_CTRL_REACH_X_MM 5.0f
#define CHASSIS_CTRL_REACH_Y_MM 5.0f
#define CHASSIS_CTRL_REACH_YAW_DEG 0.5f
#define CHASSIS_CTRL_XY_SLEW_RPM_PER_S 300.0f
#define CHASSIS_CTRL_YAW_SLEW_RPM_PER_S 600.0f
#define CHASSIS_CTRL_DEFAULT_DT_S 0.01f
#define CHASSIS_CTRL_MAX_DT_S 0.05f
#define CHASSIS_CTRL_PI 3.14159265358979323846f

static float s_target_x;
static float s_target_y;
static float s_target_yaw;
static bool s_enabled;
static bool s_reached;
static float s_last_body_right_output;
static float s_last_body_forward_output;
static float s_last_yaw_output;
static uint32_t s_last_update_ms;

static float chassis_ctrl_slew(float target, float current, float max_delta)
{
  float delta = target - current;

  if (delta > max_delta)
  {
    return current + max_delta;
  }
  if (delta < -max_delta)
  {
    return current - max_delta;
  }
  return target;
}

static void chassis_ctrl_reset_output(void)
{
  s_last_body_right_output = 0.0f;
  s_last_body_forward_output = 0.0f;
  s_last_yaw_output = 0.0f;
  s_last_update_ms = 0U;
}

static float chassis_ctrl_yaw_error(float target, float measurement)
{
  float error = target - measurement;

  while (error > 180.0f)
  {
    error -= 360.0f;
  }
  while (error < -180.0f)
  {
    error += 360.0f;
  }
  return error;
}

void ChassisCtrl_MoveTarget(float x_target, float y_target, float yaw_target,
                            float pos_x, float pos_y, float yaw)
{
  (void)pos_x;
  (void)pos_y;
  (void)yaw;
  s_target_x = x_target;
  s_target_y = y_target;
  s_target_yaw = yaw_target;
  s_reached = false;
  PID_Reset();
}

void ChassisCtrl_Enable(bool enable)
{
  if (s_enabled == enable)
  {
    if (!enable)
    {
      DJI_Chassis_SetCommandImmediate(0.0f, 0.0f, 0.0f);
    }
    return;
  }

  s_enabled = enable;
  PID_Reset();
  chassis_ctrl_reset_output();
  if (!enable)
  {
    DJI_Chassis_SetCommandImmediate(0.0f, 0.0f, 0.0f);
  }
}

bool ChassisCtrl_Update(float pos_x, float pos_y, float yaw)
{
  float field_x_output;
  float field_y_output;
  float yaw_output;
  float cosine;
  float sine;
  float body_right_output;
  float body_forward_output;
  float dt;
  uint32_t now_ms;

  if (!s_enabled)
  {
    DJI_Chassis_SetCommandImmediate(0.0f, 0.0f, 0.0f);
    return false;
  }

  if ((fabsf(s_target_x - pos_x) < CHASSIS_CTRL_REACH_X_MM) &&
      (fabsf(s_target_y - pos_y) < CHASSIS_CTRL_REACH_Y_MM) &&
      (fabsf(chassis_ctrl_yaw_error(s_target_yaw, yaw)) <
       CHASSIS_CTRL_REACH_YAW_DEG))
  {
    DJI_Chassis_SetCommandImmediate(0.0f, 0.0f, 0.0f);
    PID_Reset();
    chassis_ctrl_reset_output();
    s_enabled = false;
    s_reached = true;
    return true;
  }

  field_x_output = PID_UpdateX(s_target_x, pos_x);
  field_y_output = PID_UpdateY(s_target_y, pos_y);
  yaw_output = PID_UpdateZ(s_target_yaw, yaw);
  cosine = cosf(yaw * CHASSIS_CTRL_PI / 180.0f);
  sine = sinf(yaw * CHASSIS_CTRL_PI / 180.0f);
  body_right_output = field_x_output * cosine - field_y_output * sine;
  body_forward_output = field_x_output * sine + field_y_output * cosine;

  now_ms = HAL_GetTick();
  dt = (s_last_update_ms == 0U)
           ? CHASSIS_CTRL_DEFAULT_DT_S
           : 0.001f * (float)(now_ms - s_last_update_ms);
  if ((dt <= 0.0f) || (dt > CHASSIS_CTRL_MAX_DT_S))
  {
    dt = CHASSIS_CTRL_DEFAULT_DT_S;
  }
  s_last_update_ms = now_ms;

  body_right_output = chassis_ctrl_slew(
      body_right_output, s_last_body_right_output,
      CHASSIS_CTRL_XY_SLEW_RPM_PER_S * dt);
  body_forward_output = chassis_ctrl_slew(
      body_forward_output, s_last_body_forward_output,
      CHASSIS_CTRL_XY_SLEW_RPM_PER_S * dt);
  yaw_output = chassis_ctrl_slew(
      yaw_output, s_last_yaw_output,
      CHASSIS_CTRL_YAW_SLEW_RPM_PER_S * dt);
  s_last_body_right_output = body_right_output;
  s_last_body_forward_output = body_forward_output;
  s_last_yaw_output = yaw_output;

  DJI_Chassis_SetCommand(body_right_output, body_forward_output, yaw_output);
  s_reached = false;
  return false;
}

bool ChassisCtrl_ReachFlag(void)
{
  return s_reached;
}
