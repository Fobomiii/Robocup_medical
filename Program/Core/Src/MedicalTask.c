/**
 * @file MedicalTask.c
 * @brief Scanner/medicine workflow adapter for the Nav2 navigation stack.
 */
#include "MedicalTask.h"

#include "ASR_Pro.h"
#include "Board.h"
#include "ChassisCtrl.h"
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
#define MEDICAL_SCAN_BEEP_MS 200U
#define MEDICAL_ORDER_HANDOFF_STOP_MS 500U
/* Allow the selected compartment time to release the medicine before moving. */
#define MEDICAL_DISPENSE_HOLD_MS 3000U

/*
 * Final docking targets are sensor-face distances.  The field geometry uses
 * 1300 mm from the vehicle centre to the side reference and 500 mm to the
 * front reference; the STP23L heads are 153 mm from the vehicle centre.
 */
#define MEDICAL_DOCK_FRONT_TARGET_MM 347
#define MEDICAL_DOCK_SIDE_TARGET_MM 1147
#define MEDICAL_DOCK_TOLERANCE_MM 35
#define MEDICAL_DOCK_SAMPLE_COUNT 12U
#define MEDICAL_DOCK_SAMPLE_TRIM_COUNT 1U
#define MEDICAL_DOCK_SAMPLE_SETTLE_MS 150U
#define MEDICAL_DOCK_MAX_CORRECTIONS 2U
#define MEDICAL_DOCK_TIMEOUT_MS 8000U
#define MEDICAL_DOCK_PI 3.14159265358979323846f
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
  MEDICAL_DOCK_SAMPLE_INITIAL = 0,
  MEDICAL_DOCK_MOVE = 1,
  MEDICAL_DOCK_SAMPLE_VERIFY = 2
} MedicalDockPhase;

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
static char s_last_scan[MEDICAL_TASK_SCAN_CODE_MAX];
static char s_bed1_scan[MEDICAL_TASK_SCAN_CODE_MAX];
static char s_bed3_scan[MEDICAL_TASK_SCAN_CODE_MAX];
static uint32_t s_scan_beep_start_ms;
static uint8_t s_scan_beep_active;
static MedicalDockPhase s_dock_phase;
static uint16_t s_dock_front_samples[MEDICAL_DOCK_SAMPLE_COUNT];
static uint16_t s_dock_side_samples[MEDICAL_DOCK_SAMPLE_COUNT];
static uint8_t s_dock_front_sample_count;
static uint8_t s_dock_side_sample_count;
static uint8_t s_dock_correction_count;
static uint32_t s_dock_last_front_sequence;
static uint32_t s_dock_last_side_sequence;
static uint32_t s_dock_phase_enter_ms;

static void medical_set_state(MedicalTaskState next);

static uint8_t medical_is_docking_state(void)
{
  return (s_state == MEDICAL_TASK_DOCK_BED1 ||
          s_state == MEDICAL_TASK_DOCK_BED3) ? 1U : 0U;
}

static uint8_t medical_dock_get_side_sample(uint16_t *distance_mm,
                                            uint32_t *frame_sequence)
{
  if (s_state == MEDICAL_TASK_DOCK_BED1)
  {
    return STP23L_GetSampleC(distance_mm, frame_sequence);
  }
  return STP23L_GetSampleA(distance_mm, frame_sequence);
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
  s_dock_phase = MEDICAL_DOCK_SAMPLE_INITIAL;
  s_dock_correction_count = 0U;
  medical_docking_sample_reset();
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
  medical_set_state((s_state == MEDICAL_TASK_DOCK_BED1)
                        ? MEDICAL_TASK_SCAN_BED1
                        : MEDICAL_TASK_SCAN_BED3);
}

static void medical_scan_reset(void)
{
  s_scan_candidate[0] = '\0';
  s_scan_candidate_count = 0U;
  /* Do not let a code captured while driving satisfy a scan state. */
  GM65_ClearFrameReady();
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
  BUZZ_On();
  s_scan_beep_start_ms = HAL_GetTick();
  s_scan_beep_active = 1U;
}

static void medical_scan_beep_update(void)
{
  if ((s_scan_beep_active != 0U) &&
      ((HAL_GetTick() - s_scan_beep_start_ms) >= MEDICAL_SCAN_BEEP_MS))
  {
    BUZZ_Off();
    s_scan_beep_active = 0U;
  }
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

  /* Keep the hardware GM65 as an independent fallback. It still needs four
   * identical frames, then passes through the same competition whitelist. */
  if (medical_scan_confirmed(confirmed, confirmed_size) != 0U &&
      medical_scan_value_allowed(expected_format, confirmed) != 0U)
  {
    medical_store_last_scan(confirmed);
    medical_store_bed_scan(expected_context, confirmed);
    medical_scan_beep_start();
    return 1U;
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
      NUC_Nav_ClearVelocity();
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_NURSE);
      break;

    case MEDICAL_TASK_NAV_BED1:
      s_current_bed = 1U;
      NUC_Nav_ClearVelocity();
      NUC_Nav_RequestGoal(NUC_NAV_GOAL_BED1);
      break;

    case MEDICAL_TASK_NAV_BED3:
      s_current_bed = 3U;
      NUC_Nav_ClearVelocity();
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
      NUC_Nav_ClearVelocity();
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
  s_last_scan[0] = '\0';
  s_bed1_scan[0] = '\0';
  s_bed3_scan[0] = '\0';
  s_scan_beep_start_ms = 0U;
  s_scan_beep_active = 0U;
  BUZZ_Off();
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

  medical_scan_beep_update();

  switch (s_state)
  {
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
                                   float yaw_clockwise_deg)
{
  uint16_t front_mm;
  uint16_t side_mm;
  int32_t front_error;
  int32_t side_error;
  float forward_delta;
  float right_delta;
  float yaw_rad;
  float field_x_delta;
  float field_y_delta;

  if (!medical_is_docking_state())
  {
    return 0U;
  }

  if ((HAL_GetTick() - s_state_enter_ms) >= MEDICAL_DOCK_TIMEOUT_MS)
  {
    medical_docking_finish();
    return 1U;
  }

  if (s_dock_phase == MEDICAL_DOCK_MOVE)
  {
    if (pose_valid == 0U)
    {
      ChassisCtrl_Enable(false);
    }
    else
    {
      ChassisCtrl_Enable(true);
      if (ChassisCtrl_Update(pos_x, pos_y, yaw_clockwise_deg))
      {
        ChassisCtrl_Enable(false);
        s_dock_phase = MEDICAL_DOCK_SAMPLE_VERIFY;
        medical_docking_sample_reset();
      }
    }
    return 1U;
  }

  ChassisCtrl_Enable(false);
  if (medical_docking_collect_samples() == 0U)
  {
    return 1U;
  }

  front_mm = medical_docking_filtered_distance(s_dock_front_samples);
  side_mm = medical_docking_filtered_distance(s_dock_side_samples);
  front_error = (int32_t)front_mm - MEDICAL_DOCK_FRONT_TARGET_MM;
  side_error = (int32_t)side_mm - MEDICAL_DOCK_SIDE_TARGET_MM;

  if (((abs(front_error) <= MEDICAL_DOCK_TOLERANCE_MM) &&
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

  forward_delta = (float)front_error;
  right_delta = (s_state == MEDICAL_TASK_DOCK_BED1)
                    ? -(float)side_error
                    : (float)side_error;
  yaw_rad = yaw_clockwise_deg * MEDICAL_DOCK_PI / 180.0f;
  field_x_delta = right_delta * cosf(yaw_rad) +
                  forward_delta * sinf(yaw_rad);
  field_y_delta = -right_delta * sinf(yaw_rad) +
                  forward_delta * cosf(yaw_rad);

  ChassisCtrl_MoveTarget(pos_x + field_x_delta,
                         pos_y + field_y_delta,
                         0.0f,
                         pos_x,
                         pos_y,
                         yaw_clockwise_deg);
  ChassisCtrl_Enable(true);
  s_dock_correction_count++;
  s_dock_phase = MEDICAL_DOCK_MOVE;
  return 1U;
}
