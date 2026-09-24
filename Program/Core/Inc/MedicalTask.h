/**
 * @file MedicalTask.h
 * @brief Medical delivery task sequencer driven exclusively by Nav2 goals.
 *
 * This module deliberately contains no chassis, waypoint-following or
 * obstacle-avoidance code.  It only sequences the scanner/servos and asks the
 * NUC for one of the calibrated Nav2 goals.
 */
#ifndef MEDICAL_TASK_H
#define MEDICAL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MEDICAL_TASK_SCAN_CODE_MAX 32U

typedef enum {
  MEDICAL_TASK_INIT = 0,
  MEDICAL_TASK_NAV_NURSE = 1,
  MEDICAL_TASK_SCAN_ORDER = 2,
  MEDICAL_TASK_NAV_BED1 = 3,
  MEDICAL_TASK_SCAN_BED1 = 4,
  MEDICAL_TASK_DISPENSE_BED1 = 5,
  MEDICAL_TASK_NAV_BED3 = 6,
  MEDICAL_TASK_SCAN_BED3 = 7,
  MEDICAL_TASK_DISPENSE_BED3 = 8,
  MEDICAL_TASK_NAV_HOME = 9,
  MEDICAL_TASK_COMPLETE = 10,
  MEDICAL_TASK_NAV_ERROR = 11,
  /* Final centimetre-scale docking, controlled from the STM32 STP23L data. */
  MEDICAL_TASK_DOCK_BED1 = 12,
  MEDICAL_TASK_DOCK_BED3 = 13
} MedicalTaskState;

/** Reset the delivery sequence and request the nurse-station Nav2 goal. */
void MedicalTask_Init(void);

/** Periodic non-blocking update. Call from the 100 Hz NUC service task. */
void MedicalTask_Update(void);

/** State byte included in the STM32 -> NUC pose telemetry. */
uint8_t MedicalTask_GetState(void);

/** Copy the latest task-validated QR/CODE128 value for local display. */
uint8_t MedicalTask_GetLastScan(char *value, uint16_t value_size);

/** Copy the accepted CODE128 value for bed 1 or bed 3. */
uint8_t MedicalTask_GetBedScan(uint8_t bed,
                               char *value,
                               uint16_t value_size);

/**
 * Return 1 only while the delivery sequence is actively navigating.
 * Scanner, dispensing, completed and error states must never accept motion.
 */
uint8_t MedicalTask_AllowsMotion(void);

uint8_t MedicalTask_DockingControl(uint8_t pose_valid,
                                   float pos_x,
                                   float pos_y,
                                   float yaw_clockwise_deg);

#ifdef __cplusplus
}
#endif

#endif /* MEDICAL_TASK_H */
