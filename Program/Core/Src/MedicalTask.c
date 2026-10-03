/**
 * @file MedicalTask.c
 * @brief Scanner/medicine workflow adapter for the Nav2 navigation stack.
 */
#include "MedicalTask.h"

#include "Board.h"
#include "ChassisCtrl.h"
#include "DJIMotorCtrlSTM32.h"
#include "NUC_Obstacle.h"
#include "Servo.h"
#include "STP23L.h"
#include "main.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MEDICAL_SCAN_BEEP_MS 200U
#define MEDICAL_DOCK_TIMEOUT_BEEP_FIRST_MS 200U
#define MEDICAL_DOCK_TIMEOUT_BEEP_GAP_MS 100U
#define MEDICAL_DOCK_TIMEOUT_BEEP_SECOND_MS 300U
#define MEDICAL_ORDER_HANDOFF_STOP_MS 500U
/* Keep the selected compartment open long enough to release the medicine. */
#define MEDICAL_DISPENSE_HOLD_MS 2500U
#define MEDICAL_DISPENSE_ANNOUNCE_DELAY_MS 1000U
#define MEDICAL_DISPENSE_TTS_TIMEOUT_MS 10000U
#define MEDICAL_BOX_COUNT 2U

/*
 * Final docking targets are sensor-face distances.  The field geometry uses
 * 1300 mm from the vehicle centre to the side reference and 500 mm to the
 * front reference; the STP23L heads are 153 mm from the vehicle centre.
 */
#define MEDICAL_DOCK_FRONT_TARGET_MM 347
#define MEDICAL_DOCK_SIDE_TARGET_MM 1147
#define MEDICAL_DOCK_SIDE_SPLIT_THRESHOLD_MM 1600U
#define MEDICAL_DOCK_TOLERANCE_MM 35
#define MEDICAL_DOCK_SAMPLE_COUNT 8U
#define MEDICAL_DOCK_SAMPLE_TRIM_COUNT 1U
#define MEDICAL_DOCK_SAMPLE_SETTLE_MS 100U
#define MEDICAL_DOCK_SAMPLE_TIMEOUT_MS 1500U
#define MEDICAL_DOCK_TRACK_MIN_SAMPLES 5U
#define MEDICAL_DOCK_TRACK_SPREAD_MM 40.0f
#define MEDICAL_DOCK_TRACK_RADIUS_MM 350.0f
#define MEDICAL_DOCK_TRACK_YAW_DEG 3.0f
#define MEDICAL_DOCK_HANDOFF_MAX_SPEED_MM_S 450.0f
#define MEDICAL_DOCK_HANDOFF_STABLE_MS 150U
#define MEDICAL_DOCK_SAFE_FORWARD_HALF_WIDTH_MM 300.0f
#define MEDICAL_DOCK_SAFE_APPROACH_DEPTH_MM 350.0f
#define MEDICAL_DOCK_SAFE_OVERSHOOT_MM 40.0f
#define MEDICAL_DOCK_LASER_MAX_CORRECTION_MM 400.0f
#define MEDICAL_DOCK_MAX_CORRECTIONS 2U
#define MEDICAL_DOCK_TIMEOUT_MS 3000U
#define MEDICAL_DOCK_SPEED_LOC_K 1000.0f
#define MEDICAL_DOCK_PI 3.14159265358979323846f
#define MEDICAL_BED1_TARGET_X_MM -2200.0f
#define MEDICAL_BED3_TARGET_X_MM 2200.0f
#define MEDICAL_BED_TARGET_Y_MM 5400.0f
#define MEDICAL_ARM_TRAVEL_DEG 750.0f
#define MEDICAL_ARM_DEPLOY_TIME_S 1.5f
#define MEDICAL_ARM_RETRACT_TIME_S 1.5f
#define MEDICAL_START_BUTTON_DEBOUNCE_MS 50U
/*
 * Bench/field navigation test mode:
 *   nurse -> bed1 -> bed3 -> home
 * without waiting for any GM65 barcode.
 *
 * Set this to 0 before the formal medicine-delivery run to restore the
 * four-read order and bedside barcode checks below.
 */
#define MEDICAL_TEST_AUTO_SKIP_SCAN 0U

typedef enum {
  MEDICINE_BOX_LEFT = 0,
  MEDICINE_BOX_RIGHT = 1
} MedicineBox;

typedef enum {
  MEDICAL_DOCK_HANDOFF = 0,
  MEDICAL_DOCK_MOVE_TRACKED = 1,
  MEDICAL_DOCK_MOVE_TRACKED_CORRECTION = 2,
  MEDICAL_DOCK_SAMPLE_INITIAL = 3,
  MEDICAL_DOCK_MOVE_COMBINED = 4,
  MEDICAL_DOCK_SAMPLE_VERIFY = 5,
  MEDICAL_DOCK_MOVE_SPLIT_SIDE = 6,
  MEDICAL_DOCK_SAMPLE_SPLIT_FRONT = 7,
  MEDICAL_DOCK_MOVE_SPLIT_FRONT = 8
} MedicalDockPhase;

typedef enum {
  MEDICAL_BUZZER_IDLE = 0,
  MEDICAL_BUZZER_SCAN_ON = 1,
  MEDICAL_BUZZER_DOCK_TIMEOUT_FIRST_ON = 2,
  MEDICAL_BUZZER_DOCK_TIMEOUT_GAP = 3,
  MEDICAL_BUZZER_DOCK_TIMEOUT_SECOND_ON = 4
} MedicalBuzzerPhase;

static MedicalTaskState s_state = MEDICAL_TASK_INIT;
static uint32_t s_state_enter_ms;
static uint8_t s_first_bed;
static uint8_t s_second_bed;
static uint8_t s_current_bed;
static uint8_t s_delivered_count;
static MedicineBox s_bed1_box;
static MedicineBox s_bed3_box;
static char s_last_scan[MEDICAL_TASK_SCAN_CODE_MAX];
static char s_bed1_scan[MEDICAL_TASK_SCAN_CODE_MAX];
static char s_bed3_scan[MEDICAL_TASK_SCAN_CODE_MAX];
static MedicalBuzzerPhase s_buzzer_phase;
static uint32_t s_buzzer_phase_start_ms;
static MedicalDockPhase s_dock_phase;
static float s_dock_front_target_samples[MEDICAL_DOCK_SAMPLE_COUNT];
static float s_dock_side_target_samples[MEDICAL_DOCK_SAMPLE_COUNT];
static uint8_t s_dock_front_target_count;
static uint8_t s_dock_side_target_count;
static uint32_t s_dock_target_last_front_sequence;
static uint32_t s_dock_target_last_side_sequence;
static uint16_t s_dock_front_samples[MEDICAL_DOCK_SAMPLE_COUNT];
static uint16_t s_dock_side_samples[MEDICAL_DOCK_SAMPLE_COUNT];
static uint8_t s_dock_front_sample_count;
static uint8_t s_dock_side_sample_count;
static uint8_t s_dock_correction_count;
static uint32_t s_dock_last_front_sequence;
static uint32_t s_dock_last_side_sequence;
static uint32_t s_dock_phase_enter_ms;
static uint32_t s_dock_handoff_slow_since_ms;
static float s_arm_origin_deg;
static uint8_t s_arm_origin_valid;
static uint8_t s_arm_target_deployed;
static uint8_t s_arm_commanded_deployed;
static uint8_t s_start_requested;
static uint8_t s_start_button_down;
static uint8_t s_start_button_armed;
static uint32_t s_start_button_change_ms;
static uint32_t s_box_open_ms[MEDICAL_BOX_COUNT];
static uint8_t s_box_close_pending[MEDICAL_BOX_COUNT];
static uint32_t s_dispense_start_ms;
static uint8_t s_dispense_announcement_started;

static void medical_set_state(MedicalTaskState next);
static void medical_docking_timeout_beep_start(void);

static uint8_t medical_start_button_event(void)
{
  uint8_t down = (HAL_GPIO_ReadPin(BTN_C_GPIO_Port, BTN_C_Pin) == GPIO_PIN_RESET)
                     ? 1U
                     : 0U;
  uint32_t now = HAL_GetTick();

  if (down != s_start_button_down)
  {
    s_start_button_down = down;
    s_start_button_change_ms = now;
    if (down == 0U)
    {
      s_start_button_armed = 1U;
    }
  }

  if ((down != 0U) && (s_start_button_armed != 0U) &&
      ((now - s_start_button_change_ms) >= MEDICAL_START_BUTTON_DEBOUNCE_MS))
  {
    s_start_button_armed = 0U;
    return 1U;
  }
  return 0U;
}

static void medical_arm_update(void)
{
  if (s_arm_origin_valid == 0U)
  {
    if (DJI_Arm_IsOnline() == 0U)
    {
      return;
    }
    s_arm_origin_deg = DJI_Arm_GetAngleDeg();
    s_arm_origin_valid = 1U;
    s_arm_commanded_deployed = 0U;
  }

  if (s_arm_target_deployed == s_arm_commanded_deployed)
  {
    return;
  }

  DJI_Arm_CtrlAngleTimed(
      (s_arm_target_deployed != 0U)
          ? (s_arm_origin_deg - MEDICAL_ARM_TRAVEL_DEG)
          : s_arm_origin_deg,
      (s_arm_target_deployed != 0U)
          ? MEDICAL_ARM_DEPLOY_TIME_S
          : MEDICAL_ARM_RETRACT_TIME_S);
  s_arm_commanded_deployed = s_arm_target_deployed;
}

static void medical_arm_set_deployed(uint8_t deployed)
{
  s_arm_target_deployed = (deployed != 0U) ? 1U : 0U;
  medical_arm_update();
}

static uint8_t medical_is_docking_state(void)
{
  return (s_state == MEDICAL_TASK_DOCK_BED1 ||
          s_state == MEDICAL_TASK_DOCK_BED3) ? 1U : 0U;
}

static uint8_t medical_is_bed_navigation_state(void)
{
  return (s_state == MEDICAL_TASK_NAV_BED1 ||
          s_state == MEDICAL_TASK_NAV_BED3) ? 1U : 0U;
}

static uint8_t medical_dock_get_side_sample(uint16_t *distance_mm,
                                            uint32_t *frame_sequence)
{
  if (s_current_bed == 1U)
  {
    return STP23L_GetSampleC(distance_mm, frame_sequence);
  }
  return STP23L_GetSampleA(distance_mm, frame_sequence);
}

static float medical_docking_clamp(float value, float minimum, float maximum)
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

static float medical_docking_abs_yaw(float yaw_clockwise_deg)
{
  while (yaw_clockwise_deg > 180.0f)
  {
    yaw_clockwise_deg -= 360.0f;
  }
  while (yaw_clockwise_deg < -180.0f)
  {
    yaw_clockwise_deg += 360.0f;
  }
  return fabsf(yaw_clockwise_deg);
}

static uint8_t medical_docking_in_safe_corridor(
    float pos_x,
    float pos_y,
    float yaw_clockwise_deg)
{
  float nominal_x;
  float side_progress;

  if ((s_current_bed != 1U) && (s_current_bed != 3U))
  {
    return 0U;
  }

  nominal_x = (s_current_bed == 1U)
                  ? MEDICAL_BED1_TARGET_X_MM
                  : MEDICAL_BED3_TARGET_X_MM;
  side_progress = (s_current_bed == 1U)
                      ? -(pos_x - nominal_x)
                      : (pos_x - nominal_x);
  return ((side_progress >= -MEDICAL_DOCK_SAFE_APPROACH_DEPTH_MM) &&
          (side_progress <= MEDICAL_DOCK_SAFE_OVERSHOOT_MM) &&
          (fabsf(pos_y - MEDICAL_BED_TARGET_Y_MM) <=
           MEDICAL_DOCK_SAFE_FORWARD_HALF_WIDTH_MM) &&
          (medical_docking_abs_yaw(yaw_clockwise_deg) <=
           MEDICAL_DOCK_TRACK_YAW_DEG))
             ? 1U
             : 0U;
}

static void medical_docking_target_samples_reset(void)
{
  uint16_t distance_mm;

  s_dock_front_target_count = 0U;
  s_dock_side_target_count = 0U;
  s_dock_target_last_front_sequence = 0U;
  s_dock_target_last_side_sequence = 0U;
  (void)STP23L_GetSampleB(&distance_mm,
                          &s_dock_target_last_front_sequence);
  (void)medical_dock_get_side_sample(
      &distance_mm, &s_dock_target_last_side_sequence);
}

static void medical_docking_append_target(float *samples,
                                          uint8_t *sample_count,
                                          float target)
{
  if (*sample_count < MEDICAL_DOCK_SAMPLE_COUNT)
  {
    samples[*sample_count] = target;
    (*sample_count)++;
    return;
  }

  memmove(&samples[0], &samples[1],
          (MEDICAL_DOCK_SAMPLE_COUNT - 1U) * sizeof(samples[0]));
  samples[MEDICAL_DOCK_SAMPLE_COUNT - 1U] = target;
}

static void medical_docking_capture_targets(float pos_x,
                                            float pos_y,
                                            float yaw_clockwise_deg)
{
  uint16_t distance_mm;
  uint32_t frame_sequence;
  float nominal_x = (s_current_bed == 1U)
                        ? MEDICAL_BED1_TARGET_X_MM
                        : MEDICAL_BED3_TARGET_X_MM;
  float delta_x = pos_x - nominal_x;
  float delta_y = pos_y - MEDICAL_BED_TARGET_Y_MM;
  float yaw_rad;
  float distance_error;
  float right_delta;

  if (((s_current_bed != 1U) && (s_current_bed != 3U)) ||
      ((delta_x * delta_x + delta_y * delta_y) >
       (MEDICAL_DOCK_TRACK_RADIUS_MM * MEDICAL_DOCK_TRACK_RADIUS_MM)) ||
      (medical_docking_in_safe_corridor(
           pos_x, pos_y, yaw_clockwise_deg) == 0U))
  {
    return;
  }

  yaw_rad = yaw_clockwise_deg * MEDICAL_DOCK_PI / 180.0f;
  if ((STP23L_GetSampleB(&distance_mm, &frame_sequence) != 0U) &&
      (frame_sequence != s_dock_target_last_front_sequence))
  {
    s_dock_target_last_front_sequence = frame_sequence;
    distance_error = (float)distance_mm - MEDICAL_DOCK_FRONT_TARGET_MM;
    if (fabsf(distance_error) <= MEDICAL_DOCK_LASER_MAX_CORRECTION_MM)
    {
      medical_docking_append_target(
          s_dock_front_target_samples, &s_dock_front_target_count,
          pos_y + distance_error * cosf(yaw_rad));
    }
  }

  if ((medical_dock_get_side_sample(&distance_mm, &frame_sequence) != 0U) &&
      (frame_sequence != s_dock_target_last_side_sequence))
  {
    s_dock_target_last_side_sequence = frame_sequence;
    distance_error = (float)distance_mm - MEDICAL_DOCK_SIDE_TARGET_MM;
    if (fabsf(distance_error) <= MEDICAL_DOCK_LASER_MAX_CORRECTION_MM)
    {
      right_delta = (s_current_bed == 1U)
                        ? -distance_error
                        : distance_error;
      medical_docking_append_target(
          s_dock_side_target_samples, &s_dock_side_target_count,
          pos_x + right_delta * cosf(yaw_rad));
    }
  }
}

static uint8_t medical_docking_filtered_target(const float *samples,
                                                uint8_t sample_count,
                                                float *target)
{
  float sorted[MEDICAL_DOCK_SAMPLE_COUNT];
  float best_spread = 1.0e9f;
  uint8_t best_start = 0U;
  uint8_t index;
  uint8_t insert_index;
  float target_sum = 0.0f;

  if ((target == NULL) ||
      (sample_count < MEDICAL_DOCK_TRACK_MIN_SAMPLES))
  {
    return 0U;
  }

  memcpy(sorted, samples, sample_count * sizeof(sorted[0]));
  for (index = 1U; index < sample_count; index++)
  {
    float value = sorted[index];
    insert_index = index;
    while ((insert_index > 0U) &&
           (sorted[insert_index - 1U] > value))
    {
      sorted[insert_index] = sorted[insert_index - 1U];
      insert_index--;
    }
    sorted[insert_index] = value;
  }

  for (index = 0U;
       index <= (sample_count - MEDICAL_DOCK_TRACK_MIN_SAMPLES);
       index++)
  {
    float spread = sorted[index + MEDICAL_DOCK_TRACK_MIN_SAMPLES - 1U] -
                   sorted[index];
    if (spread < best_spread)
    {
      best_spread = spread;
      best_start = index;
    }
  }

  if (best_spread > MEDICAL_DOCK_TRACK_SPREAD_MM)
  {
    return 0U;
  }

  for (index = 0U; index < MEDICAL_DOCK_TRACK_MIN_SAMPLES; index++)
  {
    target_sum += sorted[best_start + index];
  }
  *target = target_sum / (float)MEDICAL_DOCK_TRACK_MIN_SAMPLES;
  return 1U;
}

static void medical_docking_sample_reset(void)
{
  uint16_t distance_mm;

  s_dock_front_sample_count = 0U;
  s_dock_side_sample_count = 0U;
  s_dock_last_front_sequence = 0U;
  s_dock_last_side_sequence = 0U;
  (void)STP23L_GetSampleB(&distance_mm, &s_dock_last_front_sequence);
  (void)medical_dock_get_side_sample(&distance_mm,
                                     &s_dock_last_side_sequence);
  s_dock_phase_enter_ms = HAL_GetTick();
}

static void medical_docking_reset(void)
{
  ChassisCtrl_Enable(false);
  DJI_Chassis_SetSpeedLocationGain(0.0f);
  s_dock_phase = MEDICAL_DOCK_HANDOFF;
  s_dock_correction_count = 0U;
  s_dock_phase_enter_ms = HAL_GetTick();
}

static uint8_t medical_docking_collect_samples(void)
{
  uint16_t distance_mm;
  uint32_t frame_sequence;

  if ((HAL_GetTick() - s_dock_phase_enter_ms) <
      MEDICAL_DOCK_SAMPLE_SETTLE_MS)
  {
    return 0U;
  }

  if ((s_dock_front_sample_count < MEDICAL_DOCK_SAMPLE_COUNT) &&
      (STP23L_GetSampleB(&distance_mm, &frame_sequence) != 0U) &&
      (frame_sequence != s_dock_last_front_sequence))
  {
    s_dock_last_front_sequence = frame_sequence;
    s_dock_front_samples[s_dock_front_sample_count++] = distance_mm;
  }

  if ((s_dock_side_sample_count < MEDICAL_DOCK_SAMPLE_COUNT) &&
      (medical_dock_get_side_sample(&distance_mm, &frame_sequence) != 0U) &&
      (frame_sequence != s_dock_last_side_sequence))
  {
    s_dock_last_side_sequence = frame_sequence;
    s_dock_side_samples[s_dock_side_sample_count++] = distance_mm;
  }

  return ((s_dock_front_sample_count == MEDICAL_DOCK_SAMPLE_COUNT) &&
          (s_dock_side_sample_count == MEDICAL_DOCK_SAMPLE_COUNT))
             ? 1U
             : 0U;
}

static uint8_t medical_docking_collect_front_samples(void)
{
  uint16_t distance_mm;
  uint32_t frame_sequence;

  if ((HAL_GetTick() - s_dock_phase_enter_ms) <
      MEDICAL_DOCK_SAMPLE_SETTLE_MS)
  {
    return 0U;
  }

  if ((s_dock_front_sample_count < MEDICAL_DOCK_SAMPLE_COUNT) &&
      (STP23L_GetSampleB(&distance_mm, &frame_sequence) != 0U) &&
      (frame_sequence != s_dock_last_front_sequence))
  {
    s_dock_last_front_sequence = frame_sequence;
    s_dock_front_samples[s_dock_front_sample_count++] = distance_mm;
  }

  return (s_dock_front_sample_count == MEDICAL_DOCK_SAMPLE_COUNT) ? 1U : 0U;
}

static uint16_t medical_docking_filtered_distance(const uint16_t *samples)
{
  uint16_t sorted[MEDICAL_DOCK_SAMPLE_COUNT];
  uint32_t sum = 0U;
  uint8_t index;
  uint8_t insert_index;
  uint8_t retained_count = MEDICAL_DOCK_SAMPLE_COUNT -
                           (2U * MEDICAL_DOCK_SAMPLE_TRIM_COUNT);

  memcpy(sorted, samples, sizeof(sorted));
  for (index = 1U; index < MEDICAL_DOCK_SAMPLE_COUNT; index++)
  {
    uint16_t value = sorted[index];
    insert_index = index;
    while ((insert_index > 0U) &&
           (sorted[insert_index - 1U] > value))
    {
      sorted[insert_index] = sorted[insert_index - 1U];
      insert_index--;
    }
    sorted[insert_index] = value;
  }

  for (index = MEDICAL_DOCK_SAMPLE_TRIM_COUNT;
       index < (MEDICAL_DOCK_SAMPLE_COUNT -
                MEDICAL_DOCK_SAMPLE_TRIM_COUNT);
       index++)
  {
    sum += sorted[index];
  }
  return (uint16_t)(sum / retained_count);
}

static void medical_docking_finish(void)
{
  ChassisCtrl_Enable(false);
  DJI_Chassis_SetSpeedLocationGain(0.0f);
  medical_set_state((s_state == MEDICAL_TASK_DOCK_BED1)
                        ? MEDICAL_TASK_SCAN_BED1
                        : MEDICAL_TASK_SCAN_BED3);
}

static void medical_docking_timeout_finish(void)
{
  medical_docking_timeout_beep_start();
  medical_docking_finish();
}

static void medical_docking_start_move(float pos_x,
                                       float pos_y,
                                       float yaw_clockwise_deg,
                                       float forward_delta,
                                       float right_delta,
                                       MedicalDockPhase move_phase)
{
  float yaw_rad = yaw_clockwise_deg * MEDICAL_DOCK_PI / 180.0f;
  float limited_forward_delta = medical_docking_clamp(
      forward_delta,
      -MEDICAL_DOCK_LASER_MAX_CORRECTION_MM,
      MEDICAL_DOCK_LASER_MAX_CORRECTION_MM);
  float limited_right_delta = medical_docking_clamp(
      right_delta,
      -MEDICAL_DOCK_LASER_MAX_CORRECTION_MM,
      MEDICAL_DOCK_LASER_MAX_CORRECTION_MM);
  float field_x_delta = limited_right_delta * cosf(yaw_rad) +
                        limited_forward_delta * sinf(yaw_rad);
  float field_y_delta = -limited_right_delta * sinf(yaw_rad) +
                        limited_forward_delta * cosf(yaw_rad);

  ChassisCtrl_MoveTarget(pos_x + field_x_delta,
                         pos_y + field_y_delta,
                         0.0f,
                         pos_x,
                         pos_y,
                         yaw_clockwise_deg);
  DJI_Chassis_SetSpeedLocationGain(MEDICAL_DOCK_SPEED_LOC_K);
  ChassisCtrl_Enable(true);
  s_dock_correction_count++;
  s_dock_phase = move_phase;
}

static void medical_docking_start_absolute_move(
    float target_x,
    float target_y,
    float pos_x,
    float pos_y,
    float yaw_clockwise_deg,
    MedicalDockPhase move_phase)
{
  target_x = pos_x + medical_docking_clamp(
                         target_x - pos_x,
                         -MEDICAL_DOCK_LASER_MAX_CORRECTION_MM,
                         MEDICAL_DOCK_LASER_MAX_CORRECTION_MM);
  target_y = pos_y + medical_docking_clamp(
                         target_y - pos_y,
                         -MEDICAL_DOCK_LASER_MAX_CORRECTION_MM,
                         MEDICAL_DOCK_LASER_MAX_CORRECTION_MM);
  ChassisCtrl_MoveTarget(target_x,
                         target_y,
                         0.0f,
                         pos_x,
                         pos_y,
                         yaw_clockwise_deg);
  DJI_Chassis_SetSpeedLocationGain(MEDICAL_DOCK_SPEED_LOC_K);
  ChassisCtrl_Enable(true);
  s_dock_correction_count++;
  s_dock_phase = move_phase;
}

static uint8_t medical_docking_get_tracked_target(float *target_x,
                                                   float *target_y)
{
  uint8_t front_valid = medical_docking_filtered_target(
      s_dock_front_target_samples, s_dock_front_target_count,
      target_y);
  uint8_t side_valid = medical_docking_filtered_target(
      s_dock_side_target_samples, s_dock_side_target_count,
      target_x);

  return ((front_valid != 0U) && (side_valid != 0U)) ? 1U : 0U;
}

static void medical_docking_handoff_reset(void)
{
  s_dock_handoff_slow_since_ms = 0U;
  medical_docking_target_samples_reset();
}

static uint8_t medical_docking_handoff_target(
    uint8_t pose_valid,
    float pos_x,
    float pos_y,
    float yaw_clockwise_deg,
    float measured_forward_mm_s,
    float measured_left_mm_s,
    float *target_x,
    float *target_y)
{
  float planar_speed = sqrtf(
      measured_forward_mm_s * measured_forward_mm_s +
      measured_left_mm_s * measured_left_mm_s);
  uint32_t now_ms = HAL_GetTick();

  if ((pose_valid == 0U) ||
      (medical_docking_in_safe_corridor(
           pos_x, pos_y, yaw_clockwise_deg) == 0U) ||
      (planar_speed >= MEDICAL_DOCK_HANDOFF_MAX_SPEED_MM_S))
  {
    if ((s_dock_handoff_slow_since_ms != 0U) ||
        (s_dock_front_target_count != 0U) ||
        (s_dock_side_target_count != 0U))
    {
      medical_docking_handoff_reset();
    }
    return 0U;
  }

  if (s_dock_handoff_slow_since_ms == 0U)
  {
    medical_docking_target_samples_reset();
    s_dock_handoff_slow_since_ms = now_ms;
  }
  medical_docking_capture_targets(pos_x, pos_y, yaw_clockwise_deg);

  if ((now_ms - s_dock_handoff_slow_since_ms) <
      MEDICAL_DOCK_HANDOFF_STABLE_MS)
  {
    return 0U;
  }
  return medical_docking_get_tracked_target(target_x, target_y);
}

static void medical_docking_finish_or_correct_tracked(
    float pos_x,
    float pos_y,
    float yaw_clockwise_deg)
{
  float target_x;
  float target_y;

  if (medical_docking_get_tracked_target(&target_x, &target_y) == 0U)
  {
    s_dock_phase = MEDICAL_DOCK_SAMPLE_VERIFY;
    medical_docking_sample_reset();
    return;
  }

  if (((fabsf(target_x - pos_x) <= MEDICAL_DOCK_TOLERANCE_MM) &&
       (fabsf(target_y - pos_y) <= MEDICAL_DOCK_TOLERANCE_MM)) ||
      (s_dock_correction_count >= MEDICAL_DOCK_MAX_CORRECTIONS))
  {
    medical_docking_finish();
    return;
  }

  medical_docking_start_absolute_move(
      target_x, target_y,
      pos_x, pos_y, yaw_clockwise_deg,
      MEDICAL_DOCK_MOVE_TRACKED_CORRECTION);
}

static void medical_scan_reset(void)
{
  NUC_Nav_ClearScanResult();
}

static void medical_store_last_scan(const char *value)
{
  uint32_t primask;

  if (value == NULL)
  {
    return;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  strncpy(s_last_scan, value, sizeof(s_last_scan) - 1U);
  s_last_scan[sizeof(s_last_scan) - 1U] = '\0';
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void medical_store_bed_scan(uint8_t context, const char *value)
{
  char *destination;
  uint32_t primask;

  if (value == NULL)
  {
    return;
  }
  if (context == (uint8_t)NUC_SCAN_CONTEXT_BED1)
  {
    destination = s_bed1_scan;
  }
  else if (context == (uint8_t)NUC_SCAN_CONTEXT_BED3)
  {
    destination = s_bed3_scan;
  }
  else
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  strncpy(destination, value, MEDICAL_TASK_SCAN_CODE_MAX - 1U);
  destination[MEDICAL_TASK_SCAN_CODE_MAX - 1U] = '\0';
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint8_t medical_bed_scan_available(uint8_t bed)
{
  const char *source = (bed == 1U) ? s_bed1_scan : s_bed3_scan;
  uint8_t available;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  available = (source[0] != '\0') ? 1U : 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return available;
}

static void medical_scan_beep_start(void)
{
  if ((s_buzzer_phase == MEDICAL_BUZZER_DOCK_TIMEOUT_FIRST_ON) ||
      (s_buzzer_phase == MEDICAL_BUZZER_DOCK_TIMEOUT_GAP) ||
      (s_buzzer_phase == MEDICAL_BUZZER_DOCK_TIMEOUT_SECOND_ON))
  {
    return;
  }
  BUZZ_On();
  s_buzzer_phase_start_ms = HAL_GetTick();
  s_buzzer_phase = MEDICAL_BUZZER_SCAN_ON;
}

static void medical_docking_timeout_beep_start(void)
{
  BUZZ_On();
  s_buzzer_phase_start_ms = HAL_GetTick();
  s_buzzer_phase = MEDICAL_BUZZER_DOCK_TIMEOUT_FIRST_ON;
}

static void medical_buzzer_update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - s_buzzer_phase_start_ms;

  switch (s_buzzer_phase)
  {
    case MEDICAL_BUZZER_SCAN_ON:
      if (elapsed >= MEDICAL_SCAN_BEEP_MS)
      {
        BUZZ_Off();
        s_buzzer_phase = MEDICAL_BUZZER_IDLE;
      }
      break;

    case MEDICAL_BUZZER_DOCK_TIMEOUT_FIRST_ON:
      if (elapsed >= MEDICAL_DOCK_TIMEOUT_BEEP_FIRST_MS)
      {
        BUZZ_Off();
        s_buzzer_phase_start_ms = now;
        s_buzzer_phase = MEDICAL_BUZZER_DOCK_TIMEOUT_GAP;
      }
      break;

    case MEDICAL_BUZZER_DOCK_TIMEOUT_GAP:
      if (elapsed >= MEDICAL_DOCK_TIMEOUT_BEEP_GAP_MS)
      {
        BUZZ_On();
        s_buzzer_phase_start_ms = now;
        s_buzzer_phase = MEDICAL_BUZZER_DOCK_TIMEOUT_SECOND_ON;
      }
      break;

    case MEDICAL_BUZZER_DOCK_TIMEOUT_SECOND_ON:
      if (elapsed >= MEDICAL_DOCK_TIMEOUT_BEEP_SECOND_MS)
      {
        BUZZ_Off();
        s_buzzer_phase = MEDICAL_BUZZER_IDLE;
      }
      break;

    case MEDICAL_BUZZER_IDLE:
    default:
      break;
  }
}

static uint8_t medical_scan_value_allowed(uint8_t format, const char *code)
{
  static const char *const order_codes[] = {"11", "13", "31", "33"};
  static const char *const bed_codes[] = {
      "6946522463487", "6921361255288", "6911345321863",
      "6944060407291", "6906841121017", "6938237700261"};
  const char *const *codes;
  uint8_t count;
  uint8_t i;

  if (format == (uint8_t)NUC_SCAN_FORMAT_QR)
  {
    codes = order_codes;
    count = (uint8_t)(sizeof(order_codes) / sizeof(order_codes[0]));
  }
  else if (format == (uint8_t)NUC_SCAN_FORMAT_CODE128)
  {
    codes = bed_codes;
    count = (uint8_t)(sizeof(bed_codes) / sizeof(bed_codes[0]));
  }
  else
  {
    return 0U;
  }

  for (i = 0U; i < count; i++)
  {
    if (strcmp(code, codes[i]) == 0)
    {
      return 1U;
    }
  }
  return 0U;
}

static uint8_t medical_take_scan(uint8_t expected_context,
                                 uint8_t expected_format,
                                 char *confirmed,
                                 uint16_t confirmed_size)
{
  NUC_NavScanResult nuc_scan;
  NUC_ScanAckStatus ack_status;

  if (confirmed == NULL || confirmed_size == 0U)
  {
    return 0U;
  }

  /* The NUC camera is primary. Its own temporal confirmation is repeated
   * here with task-state and whitelist validation before any state change. */
  if (NUC_Nav_TakeScanResult(&nuc_scan) != 0U)
  {
    if (nuc_scan.context != expected_context ||
        nuc_scan.format != expected_format)
    {
      ack_status = NUC_SCAN_ACK_WRONG_STATE;
    }
    else if (medical_scan_value_allowed(nuc_scan.format, nuc_scan.value) == 0U)
    {
      ack_status = NUC_SCAN_ACK_INVALID_CODE;
    }
    else
    {
      ack_status = NUC_SCAN_ACK_ACCEPTED;
    }

    (void)NUC_Nav_SendScanAck(nuc_scan.scan_id, ack_status);
    if (ack_status == NUC_SCAN_ACK_ACCEPTED)
    {
      strncpy(confirmed, nuc_scan.value, confirmed_size - 1U);
      confirmed[confirmed_size - 1U] = '\0';
      medical_store_last_scan(confirmed);
      medical_store_bed_scan(expected_context, confirmed);
      medical_scan_beep_start();
      return 1U;
    }
  }

  return 0U;
}

static MedicineBox medical_box_for_bed(uint8_t bed)
{
  return (bed == 1U) ? s_bed1_box : s_bed3_box;
}

static void medical_box_open(MedicineBox box)
{
  if (box == MEDICINE_BOX_LEFT)
  {
    SERVO1_OPEN();
  }
  else
  {
    SERVO2_OPEN();
  }
  s_box_open_ms[(uint8_t)box] = HAL_GetTick();
  s_box_close_pending[(uint8_t)box] = 1U;
}

static void medical_box_close(MedicineBox box)
{
  if (box == MEDICINE_BOX_LEFT)
  {
    SERVO1_CLOSE();
  }
  else
  {
    SERVO2_CLOSE();
  }
  s_box_close_pending[(uint8_t)box] = 0U;
}

static void medical_box_update(void)
{
  uint8_t box_index;
  uint32_t now = HAL_GetTick();

  for (box_index = 0U; box_index < MEDICAL_BOX_COUNT; box_index++)
  {
    if ((s_box_close_pending[box_index] != 0U) &&
        ((now - s_box_open_ms[box_index]) >= MEDICAL_DISPENSE_HOLD_MS))
    {
      medical_box_close((MedicineBox)box_index);
    }
  }
}

static void medical_box_close_all(void)
{
  medical_box_close(MEDICINE_BOX_LEFT);
  medical_box_close(MEDICINE_BOX_RIGHT);
}

static void medical_dispense_announcement_update(void)
{
  if ((s_dispense_announcement_started != 0U) ||
      ((HAL_GetTick() - s_dispense_start_ms) <
       MEDICAL_DISPENSE_ANNOUNCE_DELAY_MS))
  {
    return;
  }

  if (NUC_Nav_RequestTTS(s_current_bed) == HAL_OK)
  {
    s_dispense_announcement_started = 1U;
  }
}

static void medical_set_state(MedicalTaskState next)
{
  s_state = next;
  s_state_enter_ms = HAL_GetTick();

  switch (next)
  {
    case MEDICAL_TASK_NAV_NURSE:
      medical_arm_set_deployed(0U);
      NUC_Nav_ClearVelocity();
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_NURSE);
      break;

    case MEDICAL_TASK_WAIT_START:
      medical_arm_set_deployed(0U);
      NUC_Nav_ClearVelocity();
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_NURSE);
      break;

    case MEDICAL_TASK_WAIT_BED1_START:
      medical_arm_set_deployed(0U);
      NUC_Nav_ClearVelocity();
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_BED1);
      break;

    case MEDICAL_TASK_WAIT_BED3_START:
      medical_arm_set_deployed(0U);
      NUC_Nav_ClearVelocity();
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_BED3);
      break;

    case MEDICAL_TASK_NAV_BED1:
      s_current_bed = 1U;
      medical_arm_set_deployed(0U);
      medical_docking_handoff_reset();
      NUC_Nav_ClearVelocity();
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_BED1);
      break;

    case MEDICAL_TASK_NAV_BED3:
      s_current_bed = 3U;
      medical_arm_set_deployed(0U);
      medical_docking_handoff_reset();
      NUC_Nav_ClearVelocity();
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_BED3);
      break;

    case MEDICAL_TASK_DOCK_BED1:
    case MEDICAL_TASK_DOCK_BED3:
      /* Stable moving STP23L samples or Nav2 arrival transfer chassis
       * ownership to the STM32 for final correction while the arm deploys. */
      NUC_Nav_ClearVelocity();
      medical_arm_set_deployed(1U);
      medical_docking_reset();
      break;

    case MEDICAL_TASK_NAV_HOME:
      medical_arm_set_deployed(0U);
      NUC_Nav_ClearVelocity();
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_HOME);
      break;

    case MEDICAL_TASK_SCAN_ORDER:
      NUC_Nav_ClearVelocity();
      medical_scan_reset();
      break;

    case MEDICAL_TASK_SCAN_BED1:
    case MEDICAL_TASK_SCAN_BED3:
      NUC_Nav_ClearVelocity();
      medical_scan_reset();
      medical_box_open(medical_box_for_bed(s_current_bed));
      NUC_Nav_ClearTTS();
      s_dispense_start_ms = HAL_GetTick();
      s_dispense_announcement_started = 0U;
      break;

    case MEDICAL_TASK_DISPENSE_BED1:
    case MEDICAL_TASK_DISPENSE_BED3:
      NUC_Nav_ClearVelocity();
      break;

    case MEDICAL_TASK_COMPLETE:
    case MEDICAL_TASK_NAV_ERROR:
      medical_arm_set_deployed(0U);
      NUC_Nav_ClearVelocity();
      NUC_Nav_ClearTTS();
      medical_box_close_all();
      break;

    default:
      break;
  }
}

static void medical_handle_nav_result(MedicalTaskState reached_state)
{
  NUC_NavStatus nav_status = NUC_Nav_GetStatus();

  if (nav_status == NUC_NAV_REACHED)
  {
    medical_set_state(reached_state);
  }
  else if (nav_status == NUC_NAV_ERROR)
  {
    medical_set_state(MEDICAL_TASK_NAV_ERROR);
  }
}

static uint8_t medical_decode_order(const char *code)
{
  if (strcmp(code, "11") == 0)
  {
    s_first_bed = 1U;
    s_second_bed = 3U;
    s_bed1_box = MEDICINE_BOX_LEFT;
    s_bed3_box = MEDICINE_BOX_RIGHT;
  }
  else if (strcmp(code, "13") == 0)
  {
    s_first_bed = 1U;
    s_second_bed = 3U;
    s_bed1_box = MEDICINE_BOX_RIGHT;
    s_bed3_box = MEDICINE_BOX_LEFT;
  }
  else if (strcmp(code, "31") == 0)
  {
    s_first_bed = 3U;
    s_second_bed = 1U;
    s_bed3_box = MEDICINE_BOX_LEFT;
    s_bed1_box = MEDICINE_BOX_RIGHT;
  }
  else if (strcmp(code, "33") == 0)
  {
    s_first_bed = 3U;
    s_second_bed = 1U;
    s_bed3_box = MEDICINE_BOX_RIGHT;
    s_bed1_box = MEDICINE_BOX_LEFT;
  }
  else
  {
    return 0U;
  }
  return 1U;
}

static void medical_request_bed(uint8_t bed)
{
  medical_set_state((bed == 1U) ? MEDICAL_TASK_NAV_BED1
                                : MEDICAL_TASK_NAV_BED3);
}

void MedicalTask_Init(void)
{
  s_first_bed = 0U;
  s_second_bed = 0U;
  s_current_bed = 0U;
  s_delivered_count = 0U;
  s_bed1_box = MEDICINE_BOX_LEFT;
  s_bed3_box = MEDICINE_BOX_RIGHT;
  s_last_scan[0] = '\0';
  s_bed1_scan[0] = '\0';
  s_bed3_scan[0] = '\0';
  s_buzzer_phase = MEDICAL_BUZZER_IDLE;
  s_buzzer_phase_start_ms = 0U;
  s_arm_target_deployed = 0U;
  s_dispense_start_ms = 0U;
  s_dispense_announcement_started = 0U;
  NUC_Nav_ClearTTS();
  s_start_requested = 0U;
  s_start_button_down =
      (HAL_GPIO_ReadPin(BTN_C_GPIO_Port, BTN_C_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
  s_start_button_armed = (s_start_button_down == 0U) ? 1U : 0U;
  s_start_button_change_ms = HAL_GetTick();
  BUZZ_Off();
  medical_scan_reset();
  medical_docking_reset();
  medical_box_close_all();
  /* Plan the first route immediately, but keep the chassis locked until C. */
  medical_set_state(MEDICAL_TASK_WAIT_START);
}

void MedicalTask_Update(void)
{
#if !MEDICAL_TEST_AUTO_SKIP_SCAN
  char confirmed_code[MEDICAL_TASK_SCAN_CODE_MAX];
#endif

  medical_arm_update();
  medical_box_update();
  medical_buzzer_update();

  if (medical_start_button_event() != 0U &&
      (s_state == MEDICAL_TASK_WAIT_START ||
       s_state == MEDICAL_TASK_WAIT_BED1_START ||
       s_state == MEDICAL_TASK_WAIT_BED3_START) &&
      (NUC_Nav_GetStatus() == NUC_NAV_FOLLOWING))
  {
    s_start_requested = 1U;
  }

  switch (s_state)
  {
    case MEDICAL_TASK_WAIT_START:
      /* Scan the order at the origin, but keep its value off the dashboard.
       * A confirmed order replaces the provisional nurse route with a direct
       * route to the first requested bed. */
#if MEDICAL_TEST_AUTO_SKIP_SCAN
      if ((s_start_requested != 0U) &&
          (NUC_Nav_GetStatus() == NUC_NAV_FOLLOWING))
      {
        s_start_requested = 0U;
        medical_set_state(MEDICAL_TASK_NAV_NURSE);
      }
#else
      if (medical_take_scan((uint8_t)NUC_SCAN_CONTEXT_ORDER,
                            (uint8_t)NUC_SCAN_FORMAT_QR,
                            confirmed_code, sizeof(confirmed_code)) &&
          medical_decode_order(confirmed_code))
      {
        /* The destination changed; require a fresh A press only after the
         * corresponding bed path has been planned. */
        s_start_requested = 0U;
        medical_set_state((s_first_bed == 1U)
                              ? MEDICAL_TASK_WAIT_BED1_START
                              : MEDICAL_TASK_WAIT_BED3_START);
      }
      else if ((s_start_requested != 0U) &&
               (NUC_Nav_GetStatus() == NUC_NAV_FOLLOWING))
      {
        s_start_requested = 0U;
        medical_set_state(MEDICAL_TASK_NAV_NURSE);
      }
#endif
      break;

    case MEDICAL_TASK_WAIT_BED1_START:
      if ((s_start_requested != 0U) &&
          (NUC_Nav_GetStatus() == NUC_NAV_FOLLOWING))
      {
        s_start_requested = 0U;
        medical_set_state(MEDICAL_TASK_NAV_BED1);
      }
      break;

    case MEDICAL_TASK_WAIT_BED3_START:
      if ((s_start_requested != 0U) &&
          (NUC_Nav_GetStatus() == NUC_NAV_FOLLOWING))
      {
        s_start_requested = 0U;
        medical_set_state(MEDICAL_TASK_NAV_BED3);
      }
      break;

    case MEDICAL_TASK_NAV_NURSE:
#if MEDICAL_TEST_AUTO_SKIP_SCAN
      medical_handle_nav_result(MEDICAL_TASK_SCAN_ORDER);
#else
      if (medical_take_scan((uint8_t)NUC_SCAN_CONTEXT_ORDER,
                            (uint8_t)NUC_SCAN_FORMAT_QR,
                            confirmed_code, sizeof(confirmed_code)) &&
          medical_decode_order(confirmed_code))
      {
        medical_set_state(MEDICAL_TASK_SCAN_ORDER);
      }
      else
      {
        medical_handle_nav_result(MEDICAL_TASK_SCAN_ORDER);
      }
#endif
      break;

    case MEDICAL_TASK_SCAN_ORDER:
#if MEDICAL_TEST_AUTO_SKIP_SCAN
      if (s_first_bed == 0U)
      {
        s_first_bed = 1U;
        s_second_bed = 3U;
        s_bed1_box = MEDICINE_BOX_LEFT;
        s_bed3_box = MEDICINE_BOX_RIGHT;
      }
#else
      if (s_first_bed == 0U &&
          medical_take_scan((uint8_t)NUC_SCAN_CONTEXT_ORDER,
                            (uint8_t)NUC_SCAN_FORMAT_QR,
                            confirmed_code, sizeof(confirmed_code)) &&
          medical_decode_order(confirmed_code))
      {
        medical_set_state(MEDICAL_TASK_SCAN_ORDER);
      }
#endif
      if (s_first_bed != 0U &&
          (HAL_GetTick() - s_state_enter_ms) >= MEDICAL_ORDER_HANDOFF_STOP_MS)
      {
        medical_request_bed(s_first_bed);
      }
      break;

    case MEDICAL_TASK_NAV_BED1:
#if !MEDICAL_TEST_AUTO_SKIP_SCAN
      if (medical_bed_scan_available(1U) == 0U)
      {
        (void)medical_take_scan((uint8_t)NUC_SCAN_CONTEXT_BED1,
                                (uint8_t)NUC_SCAN_FORMAT_CODE128,
                                confirmed_code, sizeof(confirmed_code));
      }
#endif
      medical_handle_nav_result(MEDICAL_TASK_DOCK_BED1);
      break;

    case MEDICAL_TASK_SCAN_BED1:
      medical_dispense_announcement_update();
#if MEDICAL_TEST_AUTO_SKIP_SCAN
      medical_set_state(MEDICAL_TASK_DISPENSE_BED1);
#else
      if ((medical_bed_scan_available(1U) != 0U) ||
          medical_take_scan((uint8_t)NUC_SCAN_CONTEXT_BED1,
                            (uint8_t)NUC_SCAN_FORMAT_CODE128,
                            confirmed_code, sizeof(confirmed_code)))
      {
        medical_set_state(MEDICAL_TASK_DISPENSE_BED1);
      }
#endif
      break;

    case MEDICAL_TASK_NAV_BED3:
#if !MEDICAL_TEST_AUTO_SKIP_SCAN
      if (medical_bed_scan_available(3U) == 0U)
      {
        (void)medical_take_scan((uint8_t)NUC_SCAN_CONTEXT_BED3,
                                (uint8_t)NUC_SCAN_FORMAT_CODE128,
                                confirmed_code, sizeof(confirmed_code));
      }
#endif
      medical_handle_nav_result(MEDICAL_TASK_DOCK_BED3);
      break;

    case MEDICAL_TASK_DOCK_BED1:
    case MEDICAL_TASK_DOCK_BED3:
      break;

    case MEDICAL_TASK_SCAN_BED3:
      medical_dispense_announcement_update();
#if MEDICAL_TEST_AUTO_SKIP_SCAN
      medical_set_state(MEDICAL_TASK_DISPENSE_BED3);
#else
      if ((medical_bed_scan_available(3U) != 0U) ||
          medical_take_scan((uint8_t)NUC_SCAN_CONTEXT_BED3,
                            (uint8_t)NUC_SCAN_FORMAT_CODE128,
                            confirmed_code, sizeof(confirmed_code)))
      {
        medical_set_state(MEDICAL_TASK_DISPENSE_BED3);
      }
#endif
      break;

    case MEDICAL_TASK_DISPENSE_BED1:
    case MEDICAL_TASK_DISPENSE_BED3:
      medical_dispense_announcement_update();
      if (s_dispense_announcement_started == 0U)
      {
        break;
      }
      if (NUC_Nav_GetTTSStatus() != NUC_TTS_COMPLETED)
      {
        if (NUC_Nav_GetTTSStatus() != NUC_TTS_ERROR &&
            (HAL_GetTick() - s_dispense_start_ms) <
                MEDICAL_DISPENSE_TTS_TIMEOUT_MS)
        {
          break;
        }
        NUC_Nav_ClearTTS();
      }
#if !MEDICAL_TEST_AUTO_SKIP_SCAN
      if (medical_bed_scan_available(s_current_bed) == 0U)
      {
        break;
      }
#endif
      s_delivered_count++;
      if (s_delivered_count < 2U)
      {
        medical_request_bed(s_second_bed);
      }
      else
      {
        medical_set_state(MEDICAL_TASK_NAV_HOME);
      }
      break;

    case MEDICAL_TASK_NAV_HOME:
      medical_handle_nav_result(MEDICAL_TASK_COMPLETE);
      break;

    case MEDICAL_TASK_COMPLETE:
    case MEDICAL_TASK_NAV_ERROR:
    case MEDICAL_TASK_INIT:
    default:
      break;
  }
}

uint8_t MedicalTask_GetState(void)
{
  return (uint8_t)s_state;
}

uint8_t MedicalTask_GetLastScan(char *value, uint16_t value_size)
{
  uint32_t primask;

  if (value == NULL || value_size == 0U)
  {
    return 0U;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  strncpy(value, s_last_scan, value_size - 1U);
  value[value_size - 1U] = '\0';
  if (primask == 0U)
  {
    __enable_irq();
  }
  return value[0] != '\0' ? 1U : 0U;
}

uint8_t MedicalTask_GetBedScan(uint8_t bed,
                               char *value,
                               uint16_t value_size)
{
  const char *source;
  uint32_t primask;

  if ((value == NULL) || (value_size == 0U) ||
      ((bed != 1U) && (bed != 3U)))
  {
    return 0U;
  }
  source = (bed == 1U) ? s_bed1_scan : s_bed3_scan;
  primask = __get_PRIMASK();
  __disable_irq();
  strncpy(value, source, value_size - 1U);
  value[value_size - 1U] = '\0';
  if (primask == 0U)
  {
    __enable_irq();
  }
  return (value[0] != '\0') ? 1U : 0U;
}

uint8_t MedicalTask_AllowsMotion(void)
{
  switch (s_state)
  {
    case MEDICAL_TASK_NAV_NURSE:
    case MEDICAL_TASK_NAV_BED1:
    case MEDICAL_TASK_NAV_BED3:
    case MEDICAL_TASK_NAV_HOME:
    case MEDICAL_TASK_DOCK_BED1:
    case MEDICAL_TASK_DOCK_BED3:
      return 1U;

    default:
      return 0U;
  }
}

uint8_t MedicalTask_DockingControl(uint8_t pose_valid,
                                   float pos_x,
                                   float pos_y,
                                   float yaw_clockwise_deg,
                                   float measured_forward_mm_s,
                                   float measured_left_mm_s)
{
  float tracked_target_x;
  float tracked_target_y;
  uint16_t front_mm;
  uint16_t side_mm;
  int32_t front_error;
  int32_t side_error;
  float forward_delta;
  float right_delta;
  uint8_t front_valid;

  if (medical_is_bed_navigation_state() != 0U)
  {
    if (medical_docking_handoff_target(
            pose_valid,
            pos_x,
            pos_y,
            yaw_clockwise_deg,
            measured_forward_mm_s,
            measured_left_mm_s,
            &tracked_target_x,
            &tracked_target_y) != 0U)
    {
      medical_set_state((s_state == MEDICAL_TASK_NAV_BED1)
                            ? MEDICAL_TASK_DOCK_BED1
                            : MEDICAL_TASK_DOCK_BED3);
      medical_docking_start_absolute_move(
          tracked_target_x, tracked_target_y,
          pos_x, pos_y, yaw_clockwise_deg,
          MEDICAL_DOCK_MOVE_TRACKED);
      return 1U;
    }
    return 0U;
  }

  if (medical_is_docking_state() == 0U)
  {
    return 0U;
  }

  /* Keep refreshing the tracked target during docking, as in a187cec.
   * The moving handoff guards above apply before chassis ownership changes;
   * they must not prevent the stopped sampling fallback after Nav2 arrival. */
  if (pose_valid != 0U)
  {
    medical_docking_capture_targets(pos_x, pos_y, yaw_clockwise_deg);
  }

  if ((HAL_GetTick() - s_state_enter_ms) >= MEDICAL_DOCK_TIMEOUT_MS)
  {
    medical_docking_timeout_finish();
    return 1U;
  }

  if (s_dock_phase == MEDICAL_DOCK_HANDOFF)
  {
    ChassisCtrl_Enable(false);
    if (pose_valid == 0U)
    {
      return 1U;
    }

    if (medical_docking_get_tracked_target(
            &tracked_target_x,
            &tracked_target_y) != 0U)
    {
      medical_docking_start_absolute_move(
          tracked_target_x, tracked_target_y,
          pos_x, pos_y, yaw_clockwise_deg,
          MEDICAL_DOCK_MOVE_TRACKED);
    }
    else
    {
      s_dock_phase = MEDICAL_DOCK_SAMPLE_INITIAL;
      medical_docking_sample_reset();
    }
    return 1U;
  }

  if ((s_dock_phase == MEDICAL_DOCK_MOVE_TRACKED) ||
      (s_dock_phase == MEDICAL_DOCK_MOVE_TRACKED_CORRECTION) ||
      (s_dock_phase == MEDICAL_DOCK_MOVE_COMBINED) ||
      (s_dock_phase == MEDICAL_DOCK_MOVE_SPLIT_SIDE) ||
      (s_dock_phase == MEDICAL_DOCK_MOVE_SPLIT_FRONT))
  {
    if (pose_valid == 0U)
    {
      ChassisCtrl_Enable(false);
      DJI_Chassis_SetSpeedLocationGain(0.0f);
    }
    else
    {
      DJI_Chassis_SetSpeedLocationGain(MEDICAL_DOCK_SPEED_LOC_K);
      ChassisCtrl_Enable(true);
      if (ChassisCtrl_Update(pos_x, pos_y, yaw_clockwise_deg))
      {
        ChassisCtrl_Enable(false);
        DJI_Chassis_SetSpeedLocationGain(0.0f);
        if ((s_dock_phase == MEDICAL_DOCK_MOVE_TRACKED) ||
            (s_dock_phase == MEDICAL_DOCK_MOVE_TRACKED_CORRECTION))
        {
          medical_docking_finish_or_correct_tracked(
              pos_x, pos_y, yaw_clockwise_deg);
        }
        else if (s_dock_phase == MEDICAL_DOCK_MOVE_SPLIT_SIDE)
        {
          s_dock_phase = MEDICAL_DOCK_SAMPLE_SPLIT_FRONT;
          medical_docking_sample_reset();
        }
        else if (s_dock_phase == MEDICAL_DOCK_MOVE_SPLIT_FRONT)
        {
          medical_docking_finish();
        }
        else
        {
          s_dock_phase = MEDICAL_DOCK_SAMPLE_VERIFY;
          medical_docking_sample_reset();
        }
      }
    }
    return 1U;
  }

  ChassisCtrl_Enable(false);
  if (s_dock_phase == MEDICAL_DOCK_SAMPLE_SPLIT_FRONT)
  {
    if (medical_docking_collect_front_samples() == 0U)
    {
      if ((HAL_GetTick() - s_dock_phase_enter_ms) >=
          MEDICAL_DOCK_SAMPLE_TIMEOUT_MS)
      {
        medical_docking_timeout_finish();
      }
      return 1U;
    }

    front_mm = medical_docking_filtered_distance(s_dock_front_samples);
    front_error = (int32_t)front_mm - MEDICAL_DOCK_FRONT_TARGET_MM;
    front_valid = (abs(front_error) <=
                   (int32_t)MEDICAL_DOCK_LASER_MAX_CORRECTION_MM)
                      ? 1U
                      : 0U;
    if ((front_valid == 0U) ||
        (abs(front_error) <= MEDICAL_DOCK_TOLERANCE_MM) ||
        (s_dock_correction_count >= MEDICAL_DOCK_MAX_CORRECTIONS))
    {
      medical_docking_finish();
      return 1U;
    }
    if (pose_valid == 0U)
    {
      return 1U;
    }

    medical_docking_start_move(pos_x,
                               pos_y,
                               yaw_clockwise_deg,
                               (float)front_error,
                               0.0f,
                               MEDICAL_DOCK_MOVE_SPLIT_FRONT);
    return 1U;
  }

  if (medical_docking_collect_samples() == 0U)
  {
    if ((HAL_GetTick() - s_dock_phase_enter_ms) <
        MEDICAL_DOCK_SAMPLE_TIMEOUT_MS)
    {
      return 1U;
    }

    if ((s_dock_phase == MEDICAL_DOCK_SAMPLE_INITIAL) &&
        (s_dock_side_sample_count == MEDICAL_DOCK_SAMPLE_COUNT) &&
        (pose_valid != 0U))
    {
      side_mm = medical_docking_filtered_distance(s_dock_side_samples);
      if (side_mm > MEDICAL_DOCK_SIDE_SPLIT_THRESHOLD_MM)
      {
        side_error = (int32_t)side_mm - MEDICAL_DOCK_SIDE_TARGET_MM;
        right_delta = (s_state == MEDICAL_TASK_DOCK_BED1)
                          ? -(float)side_error
                          : (float)side_error;
        medical_docking_start_move(pos_x,
                                   pos_y,
                                   yaw_clockwise_deg,
                                   0.0f,
                                   right_delta,
                                   MEDICAL_DOCK_MOVE_SPLIT_SIDE);
        return 1U;
      }
    }

    medical_docking_timeout_finish();
    return 1U;
  }

  front_mm = medical_docking_filtered_distance(s_dock_front_samples);
  side_mm = medical_docking_filtered_distance(s_dock_side_samples);
  front_error = (int32_t)front_mm - MEDICAL_DOCK_FRONT_TARGET_MM;
  side_error = (int32_t)side_mm - MEDICAL_DOCK_SIDE_TARGET_MM;
  front_valid = (abs(front_error) <=
                 (int32_t)MEDICAL_DOCK_LASER_MAX_CORRECTION_MM)
                    ? 1U
                    : 0U;

  if ((s_dock_phase == MEDICAL_DOCK_SAMPLE_INITIAL) &&
      (side_mm > MEDICAL_DOCK_SIDE_SPLIT_THRESHOLD_MM))
  {
    if (pose_valid == 0U)
    {
      return 1U;
    }
    right_delta = (s_state == MEDICAL_TASK_DOCK_BED1)
                      ? -(float)side_error
                      : (float)side_error;
    medical_docking_start_move(pos_x,
                               pos_y,
                               yaw_clockwise_deg,
                               0.0f,
                               right_delta,
                               MEDICAL_DOCK_MOVE_SPLIT_SIDE);
    return 1U;
  }

  if ((((front_valid == 0U) ||
        (abs(front_error) <= MEDICAL_DOCK_TOLERANCE_MM)) &&
       (abs(side_error) <= MEDICAL_DOCK_TOLERANCE_MM)) ||
      (s_dock_correction_count >= MEDICAL_DOCK_MAX_CORRECTIONS))
  {
    medical_docking_finish();
    return 1U;
  }

  if (pose_valid == 0U)
  {
    return 1U;
  }

  forward_delta = (front_valid != 0U) ? (float)front_error : 0.0f;
  right_delta = (s_state == MEDICAL_TASK_DOCK_BED1)
                    ? -(float)side_error
                    : (float)side_error;
  medical_docking_start_move(pos_x,
                             pos_y,
                             yaw_clockwise_deg,
                             forward_delta,
                             right_delta,
                             MEDICAL_DOCK_MOVE_COMBINED);
  return 1U;
}
