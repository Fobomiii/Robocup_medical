/**
 * @file MedicalTask.c
 * @brief Scanner/medicine workflow adapter for the Nav2 navigation stack.
 */
#include "MedicalTask.h"

#include "ASR_Pro.h"
#include "GM65.h"
#include "NUC_Obstacle.h"
#include "Servo.h"
#include "STP23L.h"
#include "main.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Preserve the original four-read confirmation used for the GM65 scanner. */
#define MEDICAL_SCAN_CONFIRM_COUNT 4U
/* Allow the selected compartment time to release the medicine before moving. */
#define MEDICAL_DISPENSE_HOLD_MS 3000U

/*
 * Final docking targets are sensor-face distances.  The field geometry uses
 * 1300 mm from the vehicle centre to the side reference and 500 mm to the
 * front reference; the STP23L heads are 155 mm from the vehicle centre.
 */
#define MEDICAL_DOCK_FRONT_TARGET_MM 345
#define MEDICAL_DOCK_SIDE_TARGET_MM 1145
#define MEDICAL_DOCK_TOLERANCE_MM 35
#define MEDICAL_DOCK_MAX_SPEED_MM_S 120
#define MEDICAL_DOCK_MIN_SPEED_MM_S 20
#define MEDICAL_DOCK_KP 0.30f
#define MEDICAL_DOCK_SETTLE_MS 500U
#define MEDICAL_DOCK_TIMEOUT_MS 8000U
/*
 * Bench/field navigation test mode:
 *   nurse -> bed1 -> bed3 -> home
 * without waiting for any GM65 barcode.
 *
 * Set this to 0 before the formal medicine-delivery run to restore the
 * four-read order and bedside barcode checks below.
 */
#define MEDICAL_TEST_AUTO_SKIP_SCAN 1U

typedef enum {
  MEDICINE_BOX_LEFT = 0,
  MEDICINE_BOX_RIGHT = 1
} MedicineBox;

static MedicalTaskState s_state = MEDICAL_TASK_INIT;
static uint32_t s_state_enter_ms;
static uint8_t s_first_bed;
static uint8_t s_second_bed;
static uint8_t s_current_bed;
static uint8_t s_delivered_count;
static MedicineBox s_bed1_box;
static MedicineBox s_bed3_box;
static char s_scan_candidate[GM65_CODE_MAX];
static uint8_t s_scan_candidate_count;
static uint32_t s_dock_in_tolerance_since_ms;

static void medical_set_state(MedicalTaskState next);

static int16_t medical_dock_speed_from_error(int32_t error_mm)
{
  int32_t magnitude;
  int32_t speed;

  if ((error_mm <= MEDICAL_DOCK_TOLERANCE_MM) &&
      (error_mm >= -MEDICAL_DOCK_TOLERANCE_MM))
  {
    return 0;
  }

  magnitude = (int32_t)(fabsf((float)error_mm) * MEDICAL_DOCK_KP);
  if (magnitude < MEDICAL_DOCK_MIN_SPEED_MM_S)
  {
    magnitude = MEDICAL_DOCK_MIN_SPEED_MM_S;
  }
  if (magnitude > MEDICAL_DOCK_MAX_SPEED_MM_S)
  {
    magnitude = MEDICAL_DOCK_MAX_SPEED_MM_S;
  }
  speed = (error_mm < 0) ? -magnitude : magnitude;
  return (int16_t)speed;
}

static uint8_t medical_is_docking_state(void)
{
  return (s_state == MEDICAL_TASK_DOCK_BED1 ||
          s_state == MEDICAL_TASK_DOCK_BED3) ? 1U : 0U;
}

static void medical_docking_reset(void)
{
  s_dock_in_tolerance_since_ms = 0U;
}

static uint8_t medical_docking_update(void)
{
  uint16_t front_mm;
  uint16_t side_mm;
  uint8_t side_online;
  uint32_t now = HAL_GetTick();

  if (!medical_is_docking_state())
  {
    return 0U;
  }

  if ((now - s_state_enter_ms) >= MEDICAL_DOCK_TIMEOUT_MS)
  {
    /* Field-run policy: do not abort the complete route when a docking
     * sensor is unavailable or the final correction takes too long.  Leave
     * the docking loop and continue with this bed's next task. */
    medical_set_state((s_state == MEDICAL_TASK_DOCK_BED1)
                          ? MEDICAL_TASK_SCAN_BED1
                          : MEDICAL_TASK_SCAN_BED3);
    return 1U;
  }

  side_online = (s_state == MEDICAL_TASK_DOCK_BED1)
                    ? STP23L_IsOnlineC()
                    : STP23L_IsOnlineA();
  if ((STP23L_IsOnlineB() == 0U) || (side_online == 0U))
  {
    medical_docking_reset();
    return 0U;
  }

  front_mm = STP32_getB();
  side_mm = (s_state == MEDICAL_TASK_DOCK_BED1)
                ? STP32_getC()
                : STP32_getA();
  if ((front_mm == 0U) || (side_mm == 0U))
  {
    medical_docking_reset();
    return 0U;
  }

  if ((abs((int32_t)front_mm - MEDICAL_DOCK_FRONT_TARGET_MM) <=
       MEDICAL_DOCK_TOLERANCE_MM) &&
      (abs((int32_t)side_mm - MEDICAL_DOCK_SIDE_TARGET_MM) <=
       MEDICAL_DOCK_TOLERANCE_MM))
  {
    if (s_dock_in_tolerance_since_ms == 0U)
    {
      s_dock_in_tolerance_since_ms = now;
    }
    else if ((now - s_dock_in_tolerance_since_ms) >= MEDICAL_DOCK_SETTLE_MS)
    {
      medical_set_state((s_state == MEDICAL_TASK_DOCK_BED1)
                            ? MEDICAL_TASK_SCAN_BED1
                            : MEDICAL_TASK_SCAN_BED3);
      return 1U;
    }
  }
  else
  {
    medical_docking_reset();
  }
  return 0U;
}

static void medical_scan_reset(void)
{
  s_scan_candidate[0] = '\0';
  s_scan_candidate_count = 0U;
  /* Do not let a code captured while driving satisfy a scan state. */
  GM65_ClearFrameReady();
}

static uint8_t medical_scan_confirmed(char *confirmed, uint16_t confirmed_size)
{
  char code[GM65_CODE_MAX];
  const char *source;
  uint32_t primask;

  if (confirmed == NULL || confirmed_size == 0U || !GM65_FrameReady())
  {
    return 0U;
  }

  /* GM65 publishes its buffer from the UART callback. Copy one coherent frame. */
  primask = __get_PRIMASK();
  __disable_irq();
  source = GM65_GetLastCode();
  strncpy(code, source, sizeof(code) - 1U);
  code[sizeof(code) - 1U] = '\0';
  GM65_ClearFrameReady();
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (code[0] == '\0')
  {
    return 0U;
  }

  if (strcmp(code, s_scan_candidate) == 0)
  {
    if (s_scan_candidate_count < 0xFFU)
    {
      s_scan_candidate_count++;
    }
  }
  else
  {
    strncpy(s_scan_candidate, code, sizeof(s_scan_candidate) - 1U);
    s_scan_candidate[sizeof(s_scan_candidate) - 1U] = '\0';
    s_scan_candidate_count = 1U;
  }

  if (s_scan_candidate_count < MEDICAL_SCAN_CONFIRM_COUNT)
  {
    return 0U;
  }

  strncpy(confirmed, s_scan_candidate, confirmed_size - 1U);
  confirmed[confirmed_size - 1U] = '\0';
  medical_scan_reset();
  return 1U;
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
}

static void medical_set_state(MedicalTaskState next)
{
  s_state = next;
  s_state_enter_ms = HAL_GetTick();

  switch (next)
  {
    case MEDICAL_TASK_NAV_NURSE:
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_NURSE);
      break;

    case MEDICAL_TASK_NAV_BED1:
      s_current_bed = 1U;
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_BED1);
      break;

    case MEDICAL_TASK_NAV_BED3:
      s_current_bed = 3U;
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_BED3);
      break;

    case MEDICAL_TASK_DOCK_BED1:
    case MEDICAL_TASK_DOCK_BED3:
      /* Nav2 has reached the coarse bed goal.  The STM32 now owns the
       * chassis for the final STP23L distance correction. */
      NUC_Nav_ClearVelocity();
      medical_docking_reset();
      break;

    case MEDICAL_TASK_NAV_HOME:
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_HOME);
      break;

    case MEDICAL_TASK_SCAN_ORDER:
    case MEDICAL_TASK_SCAN_BED1:
    case MEDICAL_TASK_SCAN_BED3:
      NUC_Nav_ClearVelocity();
      medical_scan_reset();
      break;

    case MEDICAL_TASK_DISPENSE_BED1:
      NUC_Nav_ClearVelocity();
      (void)ASR_Pro_AnnounceBed1();
      medical_box_open(medical_box_for_bed(s_current_bed));
      break;

    case MEDICAL_TASK_DISPENSE_BED3:
      NUC_Nav_ClearVelocity();
      (void)ASR_Pro_AnnounceBed3();
      medical_box_open(medical_box_for_bed(s_current_bed));
      break;

    case MEDICAL_TASK_COMPLETE:
    case MEDICAL_TASK_NAV_ERROR:
      NUC_Nav_ClearVelocity();
      SERVO1_CLOSE();
      SERVO2_CLOSE();
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
  medical_scan_reset();
  medical_docking_reset();
  SERVO1_CLOSE();
  SERVO2_CLOSE();
  medical_set_state(MEDICAL_TASK_NAV_NURSE);
}

void MedicalTask_Update(void)
{
#if !MEDICAL_TEST_AUTO_SKIP_SCAN
  char confirmed_code[GM65_CODE_MAX];
#endif

  switch (s_state)
  {
    case MEDICAL_TASK_NAV_NURSE:
      medical_handle_nav_result(MEDICAL_TASK_SCAN_ORDER);
      break;

    case MEDICAL_TASK_SCAN_ORDER:
#if MEDICAL_TEST_AUTO_SKIP_SCAN
      s_first_bed = 1U;
      s_second_bed = 3U;
      s_bed1_box = MEDICINE_BOX_LEFT;
      s_bed3_box = MEDICINE_BOX_RIGHT;
      medical_request_bed(s_first_bed);
#else
      if (medical_scan_confirmed(confirmed_code, sizeof(confirmed_code)) &&
          medical_decode_order(confirmed_code))
      {
        medical_request_bed(s_first_bed);
      }
#endif
      break;

    case MEDICAL_TASK_NAV_BED1:
      medical_handle_nav_result(MEDICAL_TASK_DOCK_BED1);
      break;

    case MEDICAL_TASK_SCAN_BED1:
#if MEDICAL_TEST_AUTO_SKIP_SCAN
      medical_set_state(MEDICAL_TASK_DISPENSE_BED1);
#else
      /* The rule-specific bed barcode value is not yet calibrated.  Requiring
       * four equal reads still prevents a transient/noisy frame from dispensing.
       */
      if (medical_scan_confirmed(confirmed_code, sizeof(confirmed_code)))
      {
        medical_set_state(MEDICAL_TASK_DISPENSE_BED1);
      }
#endif
      break;

    case MEDICAL_TASK_NAV_BED3:
      medical_handle_nav_result(MEDICAL_TASK_DOCK_BED3);
      break;

    case MEDICAL_TASK_DOCK_BED1:
    case MEDICAL_TASK_DOCK_BED3:
      (void)medical_docking_update();
      break;

    case MEDICAL_TASK_SCAN_BED3:
#if MEDICAL_TEST_AUTO_SKIP_SCAN
      medical_set_state(MEDICAL_TASK_DISPENSE_BED3);
#else
      if (medical_scan_confirmed(confirmed_code, sizeof(confirmed_code)))
      {
        medical_set_state(MEDICAL_TASK_DISPENSE_BED3);
      }
#endif
      break;

    case MEDICAL_TASK_DISPENSE_BED1:
    case MEDICAL_TASK_DISPENSE_BED3:
      if ((HAL_GetTick() - s_state_enter_ms) >= MEDICAL_DISPENSE_HOLD_MS)
      {
        medical_box_close(medical_box_for_bed(s_current_bed));
        s_delivered_count++;
        if (s_delivered_count < 2U)
        {
          medical_request_bed(s_second_bed);
        }
        else
        {
          medical_set_state(MEDICAL_TASK_NAV_HOME);
        }
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

uint8_t MedicalTask_GetDockVelocity(int16_t *forward_mm_s,
                                    int16_t *left_mm_s,
                                    int16_t *yaw_ccw_cdeg_s)
{
  uint16_t front_mm;
  uint16_t side_mm;
  uint8_t side_online;
  int32_t front_error;
  int32_t side_error;

  if ((forward_mm_s == NULL) || (left_mm_s == NULL) ||
      (yaw_ccw_cdeg_s == NULL))
  {
    return 0U;
  }
  *forward_mm_s = 0;
  *left_mm_s = 0;
  *yaw_ccw_cdeg_s = 0;

  if (!medical_is_docking_state())
  {
    return 0U;
  }

  side_online = (s_state == MEDICAL_TASK_DOCK_BED1)
                    ? STP23L_IsOnlineC()
                    : STP23L_IsOnlineA();
  if ((STP23L_IsOnlineB() == 0U) || (side_online == 0U))
  {
    /* Never drive blind.  The timeout policy in medical_docking_update()
     * decides whether the route continues after this hold. */
    return 1U;
  }

  front_mm = STP32_getB();
  side_mm = (s_state == MEDICAL_TASK_DOCK_BED1)
                ? STP32_getC()
                : STP32_getA();
  if ((front_mm == 0U) || (side_mm == 0U))
  {
    return 1U;
  }

  front_error = (int32_t)front_mm - MEDICAL_DOCK_FRONT_TARGET_MM;
  side_error = (int32_t)side_mm - MEDICAL_DOCK_SIDE_TARGET_MM;

  /* Larger front distance means the robot must move forward. */
  *forward_mm_s = medical_dock_speed_from_error(front_error);

  /* For the left sensor (bed 1), positive left velocity approaches the
   * boundary.  For the right sensor (bed 3), the sign is reversed. */
  *left_mm_s = medical_dock_speed_from_error(side_error);
  if (s_state == MEDICAL_TASK_DOCK_BED3)
  {
    *left_mm_s = (int16_t)-(*left_mm_s);
  }
  return 1U;
}
