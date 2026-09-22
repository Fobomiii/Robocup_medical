/**
 * @file NUC_Obstacle.h
 * @brief NUC navigation UART communication @ UART8 PE1=TX PE0=RX 115200 8N1
 *
 * Frame format (8 bytes):
 * AA BB XH XL YH YL CHK 55
 * - AA BB: frame header
 * - XH XL: forward distance, int16 big-endian, unit mm (positive = forward)
 * - YH YL: lateral offset, int16 big-endian, unit mm (positive = left, negative = right)
 * - CHK: checksum, (XH+XL+YH+YL) & 0xFF
 * - 55: frame tail
 *
 * Link probe:
 *   STM32 -> NUC: A5 5A 01 00
 *   NUC   -> STM32: A5 5A 01 01
 *
 * Usage:
 *   main: MX_UART8_Init(); NUC_Obstacle_Init();
 *         while (!NUC_IsOnline()) {
 *           NUC_Obstacle_SendPing();
 *           HAL_Delay(100);
 *         }
 *   other: int16_t x = NUC_GetObstacleX(); int16_t y = NUC_GetObstacleY();
 *          uint8_t valid = NUC_IsObstacleValid();
 */
#ifndef NUC_OBSTACLE_H
#define NUC_OBSTACLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define NUC_NAV_MAX_WAYPOINTS 16U
#define NUC_NAV_SCAN_CODE_MAX 32U

typedef enum {
  NUC_NAV_GOAL_NONE  = 0,
  NUC_NAV_GOAL_HOME  = 1,
  NUC_NAV_GOAL_NURSE = 2,
  NUC_NAV_GOAL_BED1  = 3,
  NUC_NAV_GOAL_BED3  = 4
} NUC_NavGoal;

typedef enum {
  NUC_NAV_IDLE = 0,
  NUC_NAV_WAIT_PATH = 1,
  NUC_NAV_FOLLOWING = 2,
  NUC_NAV_REACHED = 3,
  NUC_NAV_ERROR = 4
} NUC_NavStatus;

typedef enum {
  NUC_SCAN_CONTEXT_ORDER = 1,
  NUC_SCAN_CONTEXT_BED1 = 2,
  NUC_SCAN_CONTEXT_BED3 = 3
} NUC_ScanContext;

typedef enum {
  NUC_SCAN_FORMAT_QR = 1,
  NUC_SCAN_FORMAT_CODE128 = 2
} NUC_ScanFormat;

typedef enum {
  NUC_SCAN_ACK_ACCEPTED = 1,
  NUC_SCAN_ACK_WRONG_STATE = 2,
  NUC_SCAN_ACK_INVALID_CODE = 3
} NUC_ScanAckStatus;

typedef struct {
  int32_t x_mm;
  int32_t y_mm;
  int16_t yaw_cdeg;
  uint16_t speed_mm_s;
} NUC_NavWaypoint;

typedef struct {
  uint16_t scan_id;
  uint8_t context;
  uint8_t format;
  uint8_t length;
  char value[NUC_NAV_SCAN_CODE_MAX];
} NUC_NavScanResult;

extern UART_HandleTypeDef huart8;

/* Debug variables */
extern volatile uint8_t g_nuc_debug_buf[8];
extern volatile uint8_t g_nuc_parse_fail_reason;
extern volatile uint32_t g_nuc_rx_count;
extern volatile uint8_t g_nuc_last_byte;
extern volatile uint8_t g_nuc_rx_history[16];
extern volatile uint8_t g_nuc_history_idx;
extern volatile uint32_t g_nuc_uart_error_count;
extern volatile uint32_t g_nuc_last_uart_error;
extern volatile uint8_t g_nuc_online;

void NUC_Obstacle_Init(void);

/** Send the link probe to NUC (A5 5A 01 00). */
HAL_StatusTypeDef NUC_Obstacle_SendPing(void);

/** Return 1 after NUC has replied to a link probe. */
uint8_t NUC_IsOnline(void);

/** Send probes until NUC replies or timeout_ms expires. */
uint8_t NUC_Obstacle_WaitForOnline(uint32_t timeout_ms);

/** Get obstacle forward distance in mm (0 if no valid data) */
int16_t NUC_GetObstacleX(void);

/** Get obstacle lateral offset in mm (0 if no valid data) */
int16_t NUC_GetObstacleY(void);

/** Check if obstacle data is valid (received within last 200ms) */
uint8_t NUC_IsObstacleValid(void);

/** Start or maintain a navigation request. A new goal increments request_id. */
void NUC_Nav_RequestGoal(NUC_NavGoal goal);

/** Periodic bidirectional telemetry service; call from the 100 Hz nav task. */
void NUC_Nav_Service(int32_t x_mm, int32_t y_mm, int16_t yaw_cdeg,
                     uint8_t task_state, NUC_NavStatus nav_status,
                     uint16_t path_id, uint8_t waypoint_index);

/** Send the three STP23L ranges at 10 Hz on the navigation UART. */
void NUC_Nav_ServiceSTP23L(void);

/** Access the most recently committed path for the active request. */
uint8_t NUC_Nav_HasRequestedPath(void);
uint16_t NUC_Nav_GetPathGeneration(void);
uint16_t NUC_Nav_GetPathId(void);
uint8_t NUC_Nav_GetPathCount(void);
uint8_t NUC_Nav_GetWaypoint(uint8_t index, NUC_NavWaypoint *waypoint);

/**
 * Read the latest fresh Nav2 body velocity command.
 * Units/signs follow ROS: forward mm/s, left mm/s, counter-clockwise cdeg/s.
 * Returns 0 and writes zeros when no valid command arrived in the last 250 ms.
 */
uint8_t NUC_Nav_GetVelocityCommand(int16_t *forward_mm_s,
                                   int16_t *left_mm_s,
                                   int16_t *yaw_ccw_cdeg_s);

/** Invalidate the current Nav2 velocity command immediately. */
void NUC_Nav_ClearVelocity(void);

/** Atomically consume the latest camera scan received from the NUC. */
uint8_t NUC_Nav_TakeScanResult(NUC_NavScanResult *result);

/** Drop a scan captured before the current task state was entered. */
void NUC_Nav_ClearScanResult(void);

/** Acknowledge a camera scan after the medical task validates it. */
HAL_StatusTypeDef NUC_Nav_SendScanAck(uint16_t scan_id,
                                      NUC_ScanAckStatus status);

/** Last navigation action status reported by the NUC. */
NUC_NavStatus NUC_Nav_GetStatus(void);

/** New-protocol activity within the last 500 ms. */
uint8_t NUC_Nav_IsOnline(void);

/** Internal: UART8 RX interrupt complete, called by HAL_UART_RxCpltCallback */
void NUC_Obstacle_OnUartRxCplt(void);

/** Internal: UART8 error callback, used to recover blocking receive errors */
void NUC_Obstacle_OnUartError(void);

#ifdef __cplusplus
}
#endif

#endif /* NUC_OBSTACLE_H */
