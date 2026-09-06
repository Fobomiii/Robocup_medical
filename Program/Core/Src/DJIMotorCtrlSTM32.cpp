/**
 * @file DJIMotorCtrlSTM32.cpp
 * @brief DJI motor control — ESP-style PID (ARMCC5 / C++03 compatible).
 */
#include "DJIMotorCtrlSTM32.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* PID                                                                        */
/* -------------------------------------------------------------------------- */
struct PidParam {
  float kp;
  float ki;
  float kd;
  float dead_zone;
  float max_out;
};

class PidControl {
public:
  PidControl()
    : kp_(0.f), ki_(0.f), kd_(0.f), dead_(0.f), max_(10000.f),
      integral_(0.f), prev_err_(0.f), last_us_(0)
  {
  }

  void setParam(const PidParam& p)
  {
    kp_ = p.kp;
    ki_ = p.ki;
    kd_ = p.kd;
    dead_ = p.dead_zone;
    max_ = p.max_out;
    reset();
  }

  void reset()
  {
    integral_ = 0.f;
    prev_err_ = 0.f;
    last_us_ = 0;
  }

  float control(float error, uint32_t now_us)
  {
    if (fabsf(error) < dead_) {
      error = 0.f;
    }

    float dt = 1e-6f * (float)(now_us - last_us_);
    if (last_us_ == 0 || dt <= 0.f || dt > 0.05f) {
      dt = 0.001f;
    }

    float p_out = kp_ * error;
    integral_ += dt * ki_ * error;
    if (integral_ > max_) {
      integral_ = max_;
    } else if (integral_ < -max_) {
      integral_ = -max_;
    }
    if (fabsf(p_out) > max_) {
      integral_ = 0.f;
    }

    float d_out = kd_ * (error - prev_err_) / dt;
    prev_err_ = error;
    last_us_ = now_us;

    float out = p_out + integral_ + d_out;
    if (out > max_) {
      return max_;
    }
    if (out < -max_) {
      return -max_;
    }
    return out;
  }

private:
  float kp_;
  float ki_;
  float kd_;
  float dead_;
  float max_;
  float integral_;
  float prev_err_;
  uint32_t last_us_;
};

/* -------------------------------------------------------------------------- */
/* Time / DWT                                                                 */
/* -------------------------------------------------------------------------- */
static void dwt_init_once(void)
{
  static uint8_t inited = 0;
  if (inited) {
    return;
  }
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  inited = 1;
}

static uint32_t micros_u32(void)
{
  return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* -------------------------------------------------------------------------- */
/* Per-motor feedback                                                         */
/* -------------------------------------------------------------------------- */
struct MotorFb {
  int16_t angle;
  int16_t speed;
  int16_t current;
  int16_t last_angle;
  int64_t location;
  uint32_t last_rx_us;
  uint8_t angle_inited;

  float target_speed;
  float speed_loc_target;
  float last_cmd_speed;
  int16_t set_current;
  PidControl speed_pid;
  uint8_t enable;

  MotorFb()
    : angle(0), speed(0), current(0), last_angle(0), location(0), last_rx_us(0),
      angle_inited(0), target_speed(0.f), speed_loc_target(0.f), last_cmd_speed(0.f),
      set_current(0), enable(0)
  {
  }

  bool online() const
  {
    if (last_rx_us == 0) {
      return false;
    }
    return (micros_u32() - last_rx_us) < 100000U;
  }

  void pushRx(const uint8_t data[8])
  {
    angle = (int16_t)((data[0] << 8) | data[1]);
    speed = (int16_t)((data[2] << 8) | data[3]);
    current = (int16_t)((data[4] << 8) | data[5]);

    if (!angle_inited) {
      last_angle = angle;
      location = angle;
      speed_loc_target = (float)location;
      angle_inited = 1;
      last_rx_us = micros_u32();
      return;
    }

    int diff = (int)angle - (int)last_angle;
    int delta;
    if (diff > 4096) {
      delta = diff - 8192;
    } else if (diff < -4096) {
      delta = diff + 8192;
    } else {
      delta = diff;
    }
    location += delta;
    last_angle = angle;
    last_rx_us = micros_u32();
  }
};

/* -------------------------------------------------------------------------- */
/* Shared FDCAN bus                                                           */
/* -------------------------------------------------------------------------- */
class DjiCanBus {
public:
  DjiCanBus() : h_(NULL), ready_(false) {}

  void attach(FDCAN_HandleTypeDef* h)
  {
    if (h_ != NULL) {
      return;
    }
    h_ = h;
    dwt_init_once();

    PidParam speed_default;
    speed_default.kp = 5.f;
    speed_default.ki = 1.f;
    speed_default.kd = 0.01f;
    speed_default.dead_zone = 1.f;
    speed_default.max_out = 10000.f;
    for (int i = 0; i < 8; ++i) {
      motors_[i].speed_pid.setParam(speed_default);
    }

    FDCAN_FilterTypeDef flt;
    memset(&flt, 0, sizeof(flt));
    flt.IdType = FDCAN_STANDARD_ID;
    flt.FilterIndex = 0;
    flt.FilterType = FDCAN_FILTER_MASK;
    flt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    flt.FilterID1 = 0x200;
    flt.FilterID2 = 0x7F8;
    (void)HAL_FDCAN_ConfigFilter(h_, &flt);
    (void)HAL_FDCAN_ConfigGlobalFilter(h_, FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    (void)HAL_FDCAN_Start(h_);
    (void)HAL_FDCAN_ActivateNotification(h_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    ready_ = true;
  }

  bool ready() const { return ready_; }

  MotorFb& motor(uint8_t id)
  {
    if (id < 1) {
      id = 1;
    }
    if (id > 8) {
      id = 8;
    }
    return motors_[id - 1];
  }

  void onRxFifo0(FDCAN_HandleTypeDef* h)
  {
    if (h != h_) {
      return;
    }
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    while (HAL_FDCAN_GetRxMessage(h_, FDCAN_RX_FIFO0, &hdr, data) == HAL_OK) {
      if (hdr.IdType != FDCAN_STANDARD_ID) {
        continue;
      }
      if (hdr.Identifier >= 0x201 && hdr.Identifier <= 0x208) {
        motor((uint8_t)(hdr.Identifier - 0x200)).pushRx(data);
      }
    }
  }

  void sendGroup200(int16_t i1, int16_t i2, int16_t i3, int16_t i4)
  {
    sendRaw(0x200, i1, i2, i3, i4);
  }

  void sendGroup1FF(int16_t i5, int16_t i6, int16_t i7, int16_t i8)
  {
    sendRaw(0x1FF, i5, i6, i7, i8);
  }

  int16_t speedLoopStep(MotorFb& m, float target_rpm, float loc_k, uint32_t now_us, float dt)
  {
    if (!m.enable) {
      m.set_current = 0;
      return 0;
    }

    if (m.online()) {
      m.speed_loc_target += 8192.f * target_rpm * dt / 60.f;
    }

    float err = (target_rpm - (float)m.speed)
                + loc_k * (m.speed_loc_target - (float)m.location) / 8192.f;

    float cru = m.speed_pid.control(m.online() ? err : 0.f, now_us);
    if (cru > 16384.f) {
      cru = 16384.f;
    }
    if (cru < -16384.f) {
      cru = -16384.f;
    }
    m.set_current = (int16_t)cru;
    m.target_speed = target_rpm;
    return m.set_current;
  }

private:
  void sendRaw(uint16_t std_id, int16_t a, int16_t b, int16_t c, int16_t d)
  {
    if (h_ == NULL) {
      return;
    }
    FDCAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.Identifier = std_id;
    hdr.IdType = FDCAN_STANDARD_ID;
    hdr.TxFrameType = FDCAN_DATA_FRAME;
    hdr.DataLength = FDCAN_DLC_BYTES_8;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = FDCAN_BRS_OFF;
    hdr.FDFormat = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker = 0;

    uint8_t data[8];
    data[0] = (uint8_t)(a >> 8);
    data[1] = (uint8_t)(a & 0xFF);
    data[2] = (uint8_t)(b >> 8);
    data[3] = (uint8_t)(b & 0xFF);
    data[4] = (uint8_t)(c >> 8);
    data[5] = (uint8_t)(c & 0xFF);
    data[6] = (uint8_t)(d >> 8);
    data[7] = (uint8_t)(d & 0xFF);
    (void)HAL_FDCAN_AddMessageToTxFifoQ(h_, &hdr, data);
  }

  FDCAN_HandleTypeDef* h_;
  MotorFb motors_[8];
  bool ready_;
};

static DjiCanBus g_bus;

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
  (void)RxFifo0ITs;
  g_bus.onRxFifo0(hfdcan);
}

/* -------------------------------------------------------------------------- */
/* CHASSIS                                                                    */
/* -------------------------------------------------------------------------- */
static const float kGear3508 = 19.f;
static const float kSpeedLocK = 1000.f;

CHASSIS::CHASSIS(FDCAN_HandleTypeDef* hfdcan)
  : can_(hfdcan), frq_(1000), started_(false)
{
}

void CHASSIS::begin(uint16_t frq_hz)
{
  if (frq_hz < 50) {
    frq_hz = 50;
  }
  if (frq_hz > 1000) {
    frq_hz = 1000;
  }
  frq_ = frq_hz;
  g_bus.attach(can_);
  for (uint8_t id = 1; id <= 4; ++id) {
    MotorFb& m = g_bus.motor(id);
    m.enable = 1;
    m.speed_loc_target = (float)m.location;
    m.speed_pid.reset();
  }
  started_ = true;
}

void CHASSIS::Update(float Vx, float Vy, float W)
{
  if (!started_) {
    return;
  }

  float out_rpm[4];
  out_rpm[0] = Vx + W;
  out_rpm[1] = Vy + W;
  out_rpm[2] = -Vx + W;
  out_rpm[3] = -Vy + W;

  static uint32_t last_us = 0;
  uint32_t now = micros_u32();
  float dt = 1e-6f * (float)(now - last_us);
  if (last_us == 0 || dt <= 0.f || dt > 0.05f) {
    dt = 1.f / (float)frq_;
  }
  last_us = now;

  int16_t cur[4];
  for (int i = 0; i < 4; ++i) {
    float rotor = out_rpm[i] * kGear3508;
    cur[i] = g_bus.speedLoopStep(g_bus.motor((uint8_t)(i + 1)), rotor, kSpeedLocK, now, dt);
  }
  g_bus.sendGroup200(cur[0], cur[1], cur[2], cur[3]);
}

/* -------------------------------------------------------------------------- */
/* M2006Motor                                                                 */
/* -------------------------------------------------------------------------- */
static PidControl s_arm_pos_pid;
static uint32_t s_arm_last_us = 0;

M2006Motor::M2006Motor(FDCAN_HandleTypeDef* hfdcan, uint8_t id, float gear_ratio)
  : can_(hfdcan),
    id_(id),
    gear_(gear_ratio),
    frq_(1000),
    started_(false),
    target_deg_(0.f),
    cmd_deg_(0.f),
    traj_active_(false),
    traj_t0_us_(0),
    traj_T_(0.f),
    traj_start_(0.f),
    traj_final_(0.f),
    traj_Ta_(0.f),
    traj_Tc_(0.f),
    traj_a_(0.f),
    traj_v_(0.f),
    traj_dir_(1.f)
{
  if (id_ < 1) {
    id_ = 1;
  }
  if (id_ > 8) {
    id_ = 8;
  }
  if (gear_ < 1.f) {
    gear_ = 1.f;
  }
}

float M2006Motor::encPerOutDeg() const
{
  return (8192.f * gear_) / 360.f;
}

float M2006Motor::getAngleDeg()
{
  MotorFb& m = g_bus.motor(id_);
  return (float)m.location / encPerOutDeg();
}

void M2006Motor::sendCurrent(int16_t current)
{
  if (id_ >= 1 && id_ <= 4) {
    int16_t c1 = 0, c2 = 0, c3 = 0, c4 = 0;
    if (id_ == 1) {
      c1 = current;
    } else if (id_ == 2) {
      c2 = current;
    } else if (id_ == 3) {
      c3 = current;
    } else {
      c4 = current;
    }
    g_bus.sendGroup200(c1, c2, c3, c4);
  } else {
    int16_t c5 = 0, c6 = 0, c7 = 0, c8 = 0;
    if (id_ == 5) {
      c5 = current;
    } else if (id_ == 6) {
      c6 = current;
    } else if (id_ == 7) {
      c7 = current;
    } else {
      c8 = current;
    }
    g_bus.sendGroup1FF(c5, c6, c7, c8);
  }
}

void M2006Motor::begin(uint16_t frq_hz)
{
  if (frq_hz < 50) {
    frq_hz = 50;
  }
  if (frq_hz > 1000) {
    frq_hz = 1000;
  }
  frq_ = frq_hz;
  g_bus.attach(can_);

  PidParam pos_default;
  pos_default.kp = 0.1f;
  pos_default.ki = 0.1f;
  pos_default.kd = 0.f;
  pos_default.dead_zone = 2000.f;
  pos_default.max_out = 3000.f;
  s_arm_pos_pid.setParam(pos_default);

  MotorFb& m = g_bus.motor(id_);
  m.enable = 1;
  m.speed_loc_target = (float)m.location;
  m.speed_pid.reset();
  s_arm_last_us = 0;

  cmd_deg_ = getAngleDeg();
  target_deg_ = cmd_deg_;
  traj_active_ = false;
  started_ = true;
}

void M2006Motor::ctrlAngle(float deg)
{
  traj_active_ = false;
  target_deg_ = deg;
  cmd_deg_ = deg;
}

void M2006Motor::planTrapezoid(float start_deg, float final_deg, float time_s)
{
  float S = final_deg - start_deg;
  float Sabs = fabsf(S);

  traj_start_ = start_deg;
  traj_final_ = final_deg;
  traj_dir_ = (S >= 0.f) ? 1.f : -1.f;
  traj_T_ = time_s;
  traj_t0_us_ = micros_u32();

  if (Sabs < 1e-3f || time_s < 1e-3f) {
    traj_active_ = false;
    cmd_deg_ = final_deg;
    target_deg_ = final_deg;
    return;
  }

  float Ta = time_s / 3.f;
  float Tc = time_s / 3.f;
  float v = Sabs / (time_s - Ta);
  float a = v / Ta;

  if (Tc < 1e-4f) {
    Ta = time_s * 0.5f;
    Tc = 0.f;
    v = 2.f * Sabs / time_s;
    a = v / Ta;
  }

  traj_Ta_ = Ta;
  traj_Tc_ = Tc;
  traj_a_ = a;
  traj_v_ = v;
  traj_active_ = true;
  target_deg_ = final_deg;
  cmd_deg_ = start_deg;
}

float M2006Motor::trajEval(float t) const
{
  if (t <= 0.f) {
    return traj_start_;
  }
  if (t >= traj_T_) {
    return traj_final_;
  }

  float Ta = traj_Ta_;
  float Tc = traj_Tc_;
  float a = traj_a_;
  float v = traj_v_;
  float dir = traj_dir_;
  float s_abs;

  if (t <= Ta) {
    s_abs = 0.5f * a * t * t;
  } else if (t <= Ta + Tc) {
    float s_acc = 0.5f * a * Ta * Ta;
    s_abs = s_acc + v * (t - Ta);
  } else {
    float te = traj_T_ - t;
    float Sabs = fabsf(traj_final_ - traj_start_);
    s_abs = Sabs - 0.5f * a * te * te;
  }

  return traj_start_ + dir * s_abs;
}

void M2006Motor::ctrlAngle(float deg, float time_s)
{
  if (time_s <= 0.f) {
    ctrlAngle(deg);
    return;
  }
  float start = traj_active_ ? cmd_deg_ : getAngleDeg();
  planTrapezoid(start, deg, time_s);
}

void M2006Motor::update()
{
  if (!started_) {
    return;
  }

  if (traj_active_) {
    float t = 1e-6f * (float)(micros_u32() - traj_t0_us_);
    cmd_deg_ = trajEval(t);
    if (t >= traj_T_) {
      cmd_deg_ = traj_final_;
      traj_active_ = false;
    }
  } else {
    cmd_deg_ = target_deg_;
  }

  MotorFb& m = g_bus.motor(id_);
  uint32_t now = micros_u32();
  float dt = 1e-6f * (float)(now - s_arm_last_us);
  if (s_arm_last_us == 0 || dt <= 0.f || dt > 0.05f) {
    dt = 1.f / (float)frq_;
  }
  s_arm_last_us = now;

  float target_loc = cmd_deg_ * encPerOutDeg();
  float pos_err = target_loc - (float)m.location;
  float rotor_rpm = s_arm_pos_pid.control(pos_err, now);

  int16_t cur = g_bus.speedLoopStep(m, rotor_rpm, kSpeedLocK, now, dt);
  sendCurrent(cur);
}

/* -------------------------------------------------------------------------- */
/* Globals + C API                                                            */
/* -------------------------------------------------------------------------- */
extern FDCAN_HandleTypeDef hfdcan1;

CHASSIS chassis(&hfdcan1);
/* M2006 + P36, CAN ID 5 */
M2006Motor arm(&hfdcan1, 5, 36.f);

static volatile float s_cmd_vx = 0.f;
static volatile float s_cmd_vy = 0.f;
static volatile float s_cmd_w = 0.f;

extern "C" void DJI_Chassis_SetCommand(float vx, float vy, float w)
{
  s_cmd_vx = vx;
  s_cmd_vy = vy;
  s_cmd_w = w;
}

extern "C" void DJI_Arm_CtrlAngle(float deg)
{
  arm.ctrlAngle(deg);
}

extern "C" void DJI_Arm_CtrlAngleTimed(float deg, float time_s)
{
  arm.ctrlAngle(deg, time_s);
}

extern "C" void DJI_Motor_ChassisTask(void)
{
  chassis.begin(1000);
  uint32_t tick = osKernelGetTickCount();
  uint32_t period = 1000U / chassis.freq();
  for (;;) {
    chassis.Update(s_cmd_vx, s_cmd_vy, s_cmd_w);
    tick += (period == 0) ? 1U : period;
    osDelayUntil(tick);
  }
}

extern "C" void DJI_Motor_ArmStart(void)
{
  arm.begin(1000);
  uint32_t tick = osKernelGetTickCount();
  uint32_t period = 1000U / arm.freq();
  for (;;) {
    arm.update();
    tick += (period == 0) ? 1U : period;
    osDelayUntil(tick);
  }
}
