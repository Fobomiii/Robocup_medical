/**
 * @file PID.cpp
 * @brief 单轴 PID 实现 + 全局 X/Y/Z 实例，DWT 计时，Z 轴航向归一化。
 */
#include "PID.h"
#include "main.h"

#include <math.h>

namespace {

void dwt_init_once(void)
{
  static uint8_t inited = 0U;
  if (inited != 0U) {
    return;
  }
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  inited = 1U;
}

uint32_t micros_u32(void)
{
  return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

float normalize_deg(float angle)
{
  while (angle > 180.f) {
    angle -= 360.f;
  }
  while (angle < -180.f) {
    angle += 360.f;
  }
  return angle;
}

float resolve_i_max(float out_min, float out_max, float i_max)
{
  if (i_max > 0.f) {
    return i_max;
  }
  {
    const float a = fabsf(out_min);
    const float b = fabsf(out_max);
    return (a > b) ? a : b;
  }
}

class PIDAxis {
public:
  PIDAxis()
    : kp_(0.f),
      ki_(0.f),
      kd_(0.f),
      out_min_(0.f),
      out_max_(0.f),
      i_max_(0.f),
      integral_(0.f),
      prev_error_(0.f),
      last_us_(0U),
      normalize_angle_(0)
  {
  }

  void setParams(float kp, float ki, float kd, float out_min, float out_max, float i_max)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    out_min_ = out_min;
    out_max_ = out_max;
    if (out_min_ > out_max_) {
      const float tmp = out_min_;
      out_min_ = out_max_;
      out_max_ = tmp;
    }
    i_max_ = resolve_i_max(out_min_, out_max_, i_max);
    reset();
  }

  void setNormalizeAngle(uint8_t enable)
  {
    normalize_angle_ = enable;
  }

  void reset()
  {
    integral_ = 0.f;
    prev_error_ = 0.f;
    last_us_ = 0U;
  }

  float update(float target, float measurement)
  {
    float error = target - measurement;
    if (normalize_angle_ != 0U) {
      error = normalize_deg(error);
    }

    dwt_init_once();
    const uint32_t now_us = micros_u32();
    float dt = 1e-6f * (float)(now_us - last_us_);
    if (last_us_ == 0U || dt <= 0.f || dt > 0.05f) {
      dt = 0.001f;
    }
    last_us_ = now_us;

    const float p_out = kp_ * error;

    if (ki_ != 0.f) {
      integral_ += dt * ki_ * error;
      if (integral_ > i_max_) {
        integral_ = i_max_;
      } else if (integral_ < -i_max_) {
        integral_ = -i_max_;
      }
    } else {
      integral_ = 0.f;
    }

    float d_out = 0.f;
    if (kd_ != 0.f) {
      d_out = kd_ * (error - prev_error_) / dt;
    }
    prev_error_ = error;

    float out = p_out + integral_ + d_out;
    if (out > out_max_) {
      return out_max_;
    }
    if (out < out_min_) {
      return out_min_;
    }
    return out;
  }

private:
  float kp_;
  float ki_;
  float kd_;
  float out_min_;
  float out_max_;
  float i_max_;
  float integral_;
  float prev_error_;
  uint32_t last_us_;
  uint8_t normalize_angle_;
};

PIDAxis s_pid_x;
PIDAxis s_pid_y;
PIDAxis s_pid_z;

} /* namespace */

extern "C" {//暴露给外部的API 便于c也能使用

void PID_SetX(float Kp, float Ki, float Kd, float out_min, float out_max, float i_max)
{
  s_pid_x.setNormalizeAngle(0U);
  s_pid_x.setParams(Kp, Ki, Kd, out_min, out_max, i_max);
}

void PID_SetY(float Kp, float Ki, float Kd, float out_min, float out_max, float i_max)
{
  s_pid_y.setNormalizeAngle(0U);
  s_pid_y.setParams(Kp, Ki, Kd, out_min, out_max, i_max);
}

void PID_SetZ(float Kp, float Ki, float Kd, float out_min, float out_max, float i_max)
{
  s_pid_z.setNormalizeAngle(1U);
  s_pid_z.setParams(Kp, Ki, Kd, out_min, out_max, i_max);
}

float PID_UpdateX(float target, float measurement)
{
  return s_pid_x.update(target, measurement);
}

float PID_UpdateY(float target, float measurement)
{
  return s_pid_y.update(target, measurement);
}

float PID_UpdateZ(float target, float measurement)
{
  return s_pid_z.update(target, measurement);
}

void PID_Reset(void)
{
  s_pid_x.reset();
  s_pid_y.reset();
  s_pid_z.reset();
}

void PID_ResetXY(void)
{
  s_pid_x.reset();
  s_pid_y.reset();
}

} /* extern "C" */
