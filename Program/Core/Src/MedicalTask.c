/**
 * @file MedicalTask.c
 * @brief Scanner/medicine workflow adapter for the Nav2 navigation stack.
 */
#include "MedicalTask.h"

#include "ASR_Pro.h"
#include "GM65.h"
#include "NUC_Obstacle.h"
#include "Servo.h"
#include "main.h"

#include <string.h>

/* Preserve the original four-read confirmation used for the GM65 scanner. */
#define MEDICAL_SCAN_CONFIRM_COUNT 4U
/* Allow the selected compartment time to release the medicine before moving. */
#define MEDICAL_DISPENSE_HOLD_MS 3000U
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
  SERVO1_CLOSE();
  SERVO2_CLOSE();
  medical_set_state(MEDICAL_TASK_NAV_NURSE);
}

void MedicalTask_Update(void)
{
  char confirmed_code[GM65_CODE_MAX];

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
      medical_handle_nav_result(MEDICAL_TASK_SCAN_BED1);
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
      medical_handle_nav_result(MEDICAL_TASK_SCAN_BED3);
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
      return 1U;

    default:
      return 0U;
  }
}
