#include "ChassisCtrl.h"

#include "DJIMotorCtrlSTM32.h"
#include "PID.h"

#include <math.h>

#define CHASSIS_CTRL_REACH_X_MM 5.0f
#define CHASSIS_CTRL_REACH_Y_MM 5.0f
#define CHASSIS_CTRL_REACH_YAW_DEG 0.5f
#define CHASSIS_CTRL_PI 3.14159265358979323846f

#define NAV_YAW_HOLD_TARGET_DEG 0.0f
#define NAV_YAW_HOLD_KP 2.2f
#define NAV_YAW_HOLD_KD 0.45f
#define NAV_YAW_HOLD_DEADBAND_DEG 1.0f
#define NAV_YAW_HOLD_MAX_RAD_S 0.40f
#define NAV_YAW_HOLD_RATE_DEADBAND_RAD_S 0.02f

static float s_target_x;
static float s_target_y;
static float s_target_yaw;
static bool s_enabled;
static bool s_reached;

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

static float chassis_ctrl_clamp(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

float ChassisCtrl_NavYawUpdate(float hwt_yaw_deg,
                               float hwt_yaw_rate_deg_s)
{
  float error_deg = chassis_ctrl_yaw_error(NAV_YAW_HOLD_TARGET_DEG,
                                            hwt_yaw_deg);
  float error_rad = error_deg * CHASSIS_CTRL_PI / 180.0f;
  float yaw_rate_rad_s = hwt_yaw_rate_deg_s * CHASSIS_CTRL_PI / 180.0f;
  float damping_rad_s = 0.0f;
  float output_rad_s;

  if (fabsf(yaw_rate_rad_s) > NAV_YAW_HOLD_RATE_DEADBAND_RAD_S)
  {
    damping_rad_s = -NAV_YAW_HOLD_KD * yaw_rate_rad_s;
  }
  if ((fabsf(error_deg) <= NAV_YAW_HOLD_DEADBAND_DEG) &&
      (damping_rad_s == 0.0f))
  {
    return 0.0f;
  }

  output_rad_s = NAV_YAW_HOLD_KP *
                     ((fabsf(error_deg) <= NAV_YAW_HOLD_DEADBAND_DEG)
                          ? 0.0f
                          : error_rad) +
                 damping_rad_s;
  output_rad_s = chassis_ctrl_clamp(output_rad_s,
                                    -NAV_YAW_HOLD_MAX_RAD_S,
                                    NAV_YAW_HOLD_MAX_RAD_S);
  return output_rad_s * 18000.0f / CHASSIS_CTRL_PI;
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

  /* Send the transformed docking PID outputs without a velocity ramp. */
  DJI_Chassis_SetCommand(body_right_output, body_forward_output, yaw_output);
  s_reached = false;
  return false;
}

bool ChassisCtrl_ReachFlag(void)
{
  return s_reached;
}
